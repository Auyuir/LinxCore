from __future__ import annotations

from dataclasses import dataclass

from pycircuit import Circuit, module
from pycircuit.dsl import Signal

from ..common.exec_uop import exec_uop_comb
from ..common.isa import (
    BK_CALL,
    BK_COND,
    BK_DIRECT,
    BK_FALL,
    BK_ICALL,
    BK_IND,
    BK_RET,
    OP_BSTART_STD_COND,
    OP_BSTART_STD_CALL,
    OP_BSTART_STD_DIRECT,
    OP_BSTART_STD_FALL,
    OP_C_BSTART_COND,
    OP_C_BSTART_DIRECT,
    OP_C_BSTART_STD,
    OP_C_BSTOP,
    OP_C_LDI,
    OP_C_LWI,
    OP_C_SETC_NE,
    OP_C_SDI,
    OP_C_SWI,
    OP_C_SETC_EQ,
    OP_C_SETC_TGT,
    OP_EBREAK,
    OP_FENTRY,
    OP_FEXIT,
    OP_FRET_RA,
    OP_FRET_STK,
    OP_INVALID,
    OP_HL_LB_PCR,
    OP_HL_LBU_PCR,
    OP_HL_LD_PCR,
    OP_HL_LH_PCR,
    OP_HL_LHU_PCR,
    OP_HL_LW_PCR,
    OP_HL_LWU_PCR,
    OP_HL_SB_PCR,
    OP_HL_SD_PCR,
    OP_HL_SH_PCR,
    OP_HL_SW_PCR,
    OP_LB,
    OP_LBI,
    OP_LBU,
    OP_LBUI,
    OP_LD,
    OP_LH,
    OP_LHI,
    OP_LHU,
    OP_LHUI,
    OP_LW,
    OP_LWU,
    OP_LWUI,
    OP_SB,
    OP_SBI,
    OP_SD,
    OP_SH,
    OP_SHI,
    OP_SW,
    OP_LWI,
    OP_LDI,
    OP_SETC_AND,
    OP_SETC_ANDI,
    OP_SETC_EQ,
    OP_SETC_EQI,
    OP_SETC_GE,
    OP_SETC_GEI,
    OP_SETC_GEU,
    OP_SETC_GEUI,
    OP_SETC_LT,
    OP_SETC_LTI,
    OP_SETC_LTU,
    OP_SETC_LTUI,
    OP_SETC_NE,
    OP_SETC_NEI,
    OP_SETC_OR,
    OP_SETC_ORI,
    OP_SDI,
    OP_SWI,
    REG_INVALID,
)
from ..common.util import lshr_var, make_consts
from ..common.decode_f4 import decode_f4_bundle
from .code_template_unit import build_code_template_unit
from .helpers import alloc_from_free_mask, mask_bit, mux_by_uindex, onehot_from_tag
from .params import OooParams
from .state import make_core_ctrl_regs, make_iq_regs, make_prf, make_rename_regs, make_rob_regs


@dataclass(frozen=True)
class BccOooExports:
    clk: Signal
    rst: Signal
    block_cmd_valid: Signal
    block_cmd_kind: Signal
    block_cmd_payload: Signal
    block_cmd_tile: Signal
    block_cmd_tag: Signal
    cycles: Signal
    halted: Signal


def build_bcc_ooo(m: Circuit, *, mem_bytes: int, params: OooParams | None = None) -> BccOooExports:
    p = params or OooParams()

    clk = m.clock("clk")
    rst = m.reset("rst")

    boot_pc = m.input("boot_pc", width=64)
    boot_sp = m.input("boot_sp", width=64)
    boot_ra = m.input("boot_ra", width=64)

    # Frontend handoff (F4 bundle + ready/redirect handshake).
    f4_valid_i = m.input("f4_valid_i", width=1)
    f4_pc_i = m.input("f4_pc_i", width=64)
    f4_window_i = m.input("f4_window_i", width=64)
    f4_checkpoint_i = m.input("f4_checkpoint_i", width=6)

    # Data-memory read data from LinxCoreMem2R1W.
    dmem_rdata_i = m.input("dmem_rdata_i", width=64)

    c = m.const
    consts = make_consts(m)

    def op_is(op, *codes: int):
        v = consts.zero1
        for code in codes:
            v = v | op.eq(c(code, width=12))
        return v

    tag0 = c(0, width=p.ptag_w)

    # --- core state (architectural) ---
    state = make_core_ctrl_regs(m, clk, rst, boot_pc=boot_pc, consts=consts, p=p)

    base_can_run = (~state.halted.out()) & (~state.flush_pending.out())
    do_flush = state.flush_pending.out()

    # --- physical register file (PRF) ---
    prf = make_prf(m, clk, rst, boot_sp=boot_sp, boot_ra=boot_ra, consts=consts, p=p)

    # --- rename state ---
    ren = make_rename_regs(m, clk, rst, consts=consts, p=p)
    ckpt_entries = len(ren.ckpt_valid)
    ckpt_w = (ckpt_entries - 1).bit_length()

    # --- ROB (in-order commit) ---
    rob = make_rob_regs(m, clk, rst, consts=consts, p=p)

    # --- issue queues (bring-up split) ---
    iq_alu = make_iq_regs(m, clk, rst, consts=consts, p=p, name="iq_alu")
    iq_bru = make_iq_regs(m, clk, rst, consts=consts, p=p, name="iq_bru")
    iq_lsu = make_iq_regs(m, clk, rst, consts=consts, p=p, name="iq_lsu")

    # --- committed store buffer (drains stores to D-memory) ---
    with m.scope("stbuf"):
        stbuf_head = m.out("head", clk=clk, rst=rst, width=p.sq_w, init=c(0, width=p.sq_w), en=consts.one1)
        stbuf_tail = m.out("tail", clk=clk, rst=rst, width=p.sq_w, init=c(0, width=p.sq_w), en=consts.one1)
        stbuf_count = m.out("count", clk=clk, rst=rst, width=p.sq_w + 1, init=c(0, width=p.sq_w + 1), en=consts.one1)
        stbuf_valid = []
        stbuf_addr = []
        stbuf_data = []
        stbuf_size = []
        for i in range(p.sq_entries):
            stbuf_valid.append(m.out(f"v{i}", clk=clk, rst=rst, width=1, init=consts.zero1, en=consts.one1))
            stbuf_addr.append(m.out(f"a{i}", clk=clk, rst=rst, width=64, init=consts.zero64, en=consts.one1))
            stbuf_data.append(m.out(f"d{i}", clk=clk, rst=rst, width=64, init=consts.zero64, en=consts.one1))
            stbuf_size.append(m.out(f"s{i}", clk=clk, rst=rst, width=4, init=consts.zero4, en=consts.one1))

    # --- commit selection (up to commit_w, stop on redirect/store/halt) ---
    commit_idxs = []
    rob_pcs = []
    rob_valids = []
    rob_dones = []
    rob_ops = []
    rob_lens = []
    rob_dst_kinds = []
    rob_dst_aregs = []
    rob_pdsts = []
    rob_values = []
    rob_is_stores = []
    rob_st_addrs = []
    rob_st_datas = []
    rob_st_sizes = []
    rob_is_loads = []
    rob_ld_addrs = []
    rob_ld_datas = []
    rob_ld_sizes = []
    rob_insn_raws = []
    rob_checkpoint_ids = []
    rob_macro_begins = []
    rob_macro_ends = []
    for slot in range(p.commit_w):
        idx = rob.head.out() + c(slot, width=p.rob_w)
        commit_idxs.append(idx)
        rob_pcs.append(mux_by_uindex(m, idx=idx, items=rob.pc, default=consts.zero64))
        rob_valids.append(mux_by_uindex(m, idx=idx, items=rob.valid, default=consts.zero1))
        rob_dones.append(mux_by_uindex(m, idx=idx, items=rob.done, default=consts.zero1))
        rob_ops.append(mux_by_uindex(m, idx=idx, items=rob.op, default=c(0, width=12)))
        rob_lens.append(mux_by_uindex(m, idx=idx, items=rob.len_bytes, default=consts.zero3))
        rob_dst_kinds.append(mux_by_uindex(m, idx=idx, items=rob.dst_kind, default=c(0, width=2)))
        rob_dst_aregs.append(mux_by_uindex(m, idx=idx, items=rob.dst_areg, default=c(REG_INVALID, width=6)))
        rob_pdsts.append(mux_by_uindex(m, idx=idx, items=rob.pdst, default=tag0))
        rob_values.append(mux_by_uindex(m, idx=idx, items=rob.value, default=consts.zero64))
        rob_is_stores.append(mux_by_uindex(m, idx=idx, items=rob.is_store, default=consts.zero1))
        rob_st_addrs.append(mux_by_uindex(m, idx=idx, items=rob.store_addr, default=consts.zero64))
        rob_st_datas.append(mux_by_uindex(m, idx=idx, items=rob.store_data, default=consts.zero64))
        rob_st_sizes.append(mux_by_uindex(m, idx=idx, items=rob.store_size, default=consts.zero4))
        rob_is_loads.append(mux_by_uindex(m, idx=idx, items=rob.is_load, default=consts.zero1))
        rob_ld_addrs.append(mux_by_uindex(m, idx=idx, items=rob.load_addr, default=consts.zero64))
        rob_ld_datas.append(mux_by_uindex(m, idx=idx, items=rob.load_data, default=consts.zero64))
        rob_ld_sizes.append(mux_by_uindex(m, idx=idx, items=rob.load_size, default=consts.zero4))
        rob_insn_raws.append(mux_by_uindex(m, idx=idx, items=rob.insn_raw, default=consts.zero64))
        rob_checkpoint_ids.append(mux_by_uindex(m, idx=idx, items=rob.checkpoint_id, default=c(0, width=6)))
        rob_macro_begins.append(mux_by_uindex(m, idx=idx, items=rob.macro_begin, default=c(0, width=6)))
        rob_macro_ends.append(mux_by_uindex(m, idx=idx, items=rob.macro_end, default=c(0, width=6)))

    head_op = rob_ops[0]
    head_len = rob_lens[0]
    head_dst_kind = rob_dst_kinds[0]
    head_dst_areg = rob_dst_aregs[0]
    head_pdst = rob_pdsts[0]
    head_value = rob_values[0]
    head_is_store = rob_is_stores[0]
    head_st_addr = rob_st_addrs[0]
    head_st_data = rob_st_datas[0]
    head_st_size = rob_st_sizes[0]
    head_is_load = rob_is_loads[0]
    head_ld_addr = rob_ld_addrs[0]
    head_ld_data = rob_ld_datas[0]
    head_ld_size = rob_ld_sizes[0]
    head_insn_raw = rob_insn_raws[0]
    head_checkpoint_id = rob_checkpoint_ids[0]
    head_macro_begin = rob_macro_begins[0]
    head_macro_end = rob_macro_ends[0]

    # Commit-time branch/control decisions (BlockISA markers) for the head.
    head_is_macro = op_is(head_op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
    head_is_start_marker = (
        op_is(
            head_op,
            OP_C_BSTART_STD,
            OP_C_BSTART_COND,
            OP_C_BSTART_DIRECT,
            OP_BSTART_STD_FALL,
            OP_BSTART_STD_DIRECT,
            OP_BSTART_STD_COND,
            OP_BSTART_STD_CALL,
        )
        | head_is_macro
    )
    head_is_boundary = head_is_start_marker | op_is(head_op, OP_C_BSTOP)

    # Boundary markers are skipped when the previous block takes a control-flow
    # transition at that boundary. Skipped markers must not trigger template
    # macro expansion.
    br_kind_head = state.br_kind.out()
    br_is_cond_head = br_kind_head.eq(c(BK_COND, width=3))
    br_is_call_head = br_kind_head.eq(c(BK_CALL, width=3))
    br_is_ret_head = br_kind_head.eq(c(BK_RET, width=3))
    br_is_direct_head = br_kind_head.eq(c(BK_DIRECT, width=3))
    br_is_ind_head = br_kind_head.eq(c(BK_IND, width=3))
    br_is_icall_head = br_kind_head.eq(c(BK_ICALL, width=3))
    head_br_take = (
        br_is_call_head
        | br_is_direct_head
        | br_is_ind_head
        | br_is_icall_head
        | (br_is_cond_head & state.commit_cond.out())
        | (br_is_ret_head & state.commit_cond.out())
    )
    head_skip = head_is_boundary & head_br_take

    # Template blocks (FENTRY/FEXIT/FRET.*) expand into template-uops through
    # CodeTemplateUnit, which blocks IFU while active/starting.
    ctu = m.instance(
        build_code_template_unit,
        name="code_template_unit",
        base_can_run=base_can_run,
        head_is_macro=head_is_macro,
        head_skip=head_skip,
        head_valid=rob_valids[0],
        head_done=rob_dones[0],
        macro_active_i=state.macro_active.out(),
        macro_wait_commit_i=state.macro_wait_commit.out(),
        macro_phase_i=state.macro_phase.out(),
        macro_op_i=state.macro_op.out(),
        macro_end_i=state.macro_end.out(),
        macro_stacksize_i=state.macro_stacksize.out(),
        macro_reg_i=state.macro_reg.out(),
        macro_i_i=state.macro_i.out(),
        macro_sp_base_i=state.macro_sp_base.out(),
    )
    macro_start = ctu["start_fire"]
    macro_block = ctu["block_ifu"]

    can_run = base_can_run & (~macro_block)

    # Return target for FRET.* (via RA, possibly restored by the macro engine).
    ret_ra_tag = ren.cmap[10].out()
    ret_ra_val = mux_by_uindex(m, idx=ret_ra_tag, items=prf, default=consts.zero64)

    commit_allow = consts.one1
    commit_fires = []
    commit_pcs = []
    commit_next_pcs = []
    commit_enter_new_blocks = []
    commit_implicit_setret_fires = []
    commit_implicit_setret_aregs = []
    commit_implicit_setret_vals = []
    commit_implicit_setret_pdsts = []

    commit_count = c(0, width=3)

    redirect_valid = consts.zero1
    redirect_pc = state.pc.out()
    redirect_checkpoint_id = c(0, width=6)
    replay_redirect_fire = consts.zero1

    commit_store_fire = consts.zero1
    commit_store_addr = consts.zero64
    commit_store_data = consts.zero64
    commit_store_size = consts.zero4
    commit_store_seen = consts.zero1

    pc_live = state.pc.out()
    commit_cond_live = state.commit_cond.out()
    commit_tgt_live = state.commit_tgt.out()
    br_kind_live = state.br_kind.out()
    br_base_live = state.br_base_pc.out()
    br_off_live = state.br_off.out()

    for slot in range(p.commit_w):
        pc_this = pc_live
        commit_pcs.append(pc_this)
        op = rob_ops[slot]
        ln = rob_lens[slot]
        val = rob_values[slot]

        is_macro = op_is(op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
        is_start_marker = (
            op_is(
                op,
                OP_C_BSTART_STD,
                OP_C_BSTART_COND,
                OP_C_BSTART_DIRECT,
                OP_BSTART_STD_FALL,
                OP_BSTART_STD_DIRECT,
                OP_BSTART_STD_COND,
                OP_BSTART_STD_CALL,
            )
            | is_macro
        )
        is_boundary = is_start_marker | op_is(op, OP_C_BSTOP)

        br_is_fall = br_kind_live.eq(c(BK_FALL, width=3))
        br_is_cond = br_kind_live.eq(c(BK_COND, width=3))
        br_is_call = br_kind_live.eq(c(BK_CALL, width=3))
        br_is_ret = br_kind_live.eq(c(BK_RET, width=3))
        br_is_direct = br_kind_live.eq(c(BK_DIRECT, width=3))
        br_is_ind = br_kind_live.eq(c(BK_IND, width=3))
        br_is_icall = br_kind_live.eq(c(BK_ICALL, width=3))

        br_target = br_base_live + br_off_live
        # Dynamic target for RET/IND/ICALL blocks comes from commit_tgt.
        br_target = (br_is_ret | br_is_ind | br_is_icall).select(commit_tgt_live, br_target)
        # Allow SETC.TGT to override fixed targets for DIRECT/CALL/COND blocks.
        br_target = (~(br_is_ret | br_is_ind | br_is_icall) & (~commit_tgt_live.eq(consts.zero64))).select(commit_tgt_live, br_target)

        br_take = (
            br_is_call
            | br_is_direct
            | br_is_ind
            | br_is_icall
            | (br_is_cond & commit_cond_live)
            | (br_is_ret & commit_cond_live)
        )

        pc_inc = pc_this + ln.zext(width=64)
        boundary_fallthrough = pc_inc
        pc_next = is_boundary.select(br_take.select(br_target, boundary_fallthrough), pc_inc)

        fire = can_run & commit_allow & rob_valids[slot] & rob_dones[slot]

        # Template macro blocks (FENTRY/FEXIT/FRET.*) must reach the head of the
        # ROB so the macro microcode engine can run before the macro commits.
        # With commit_w>1, a macro could otherwise commit in the same cycle as
        # an older non-macro (slot>0) and skip the required save/restore.
        if slot != 0:
            fire = fire & (~is_macro)

        # Stop commit on redirect, store, or halt.
        is_halt = op_is(op, OP_EBREAK, OP_INVALID)
        redirect = fire & is_boundary & br_take
        replay_redirect = (
            fire
            & rob_is_stores[slot]
            & state.replay_pending.out()
            & commit_idxs[slot].eq(state.replay_store_rob.out())
        )
        replay_redirect_fire = replay_redirect.select(consts.one1, replay_redirect_fire)

        # FRET.* are explicit control-flow ops (return via RA). They behave like
        # a taken boundary when the marker is entered (i.e., not skipped by a
        # prior taken branch at this boundary).
        is_fret = op_is(op, OP_FRET_RA, OP_FRET_STK)
        fret_redirect = fire & is_fret & (~redirect)
        pc_next = fret_redirect.select(ret_ra_val, pc_next)
        redirect = redirect | fret_redirect
        pc_next = replay_redirect.select(state.replay_pc.out(), pc_next)
        redirect = redirect | replay_redirect
        commit_next_pcs.append(pc_next)
        # BSTART.CALL is modeled as boundary redirect + implicit SETRET imm=0.
        # Model it as a normal architectural writeback event.
        implicit_setret_fire = fire & is_boundary & br_take & br_is_icall
        implicit_setret_areg = c(10, width=6)
        implicit_setret_pdst = mux_by_uindex(m, idx=implicit_setret_areg, items=ren.cmap, default=tag0)
        commit_implicit_setret_fires.append(implicit_setret_fire)
        commit_implicit_setret_aregs.append(implicit_setret_areg)
        commit_implicit_setret_vals.append(pc_this)
        commit_implicit_setret_pdsts.append(implicit_setret_pdst)

        stbuf_has_space = stbuf_count.out().ult(c(p.sq_entries, width=p.sq_w + 1))
        # This milestone keeps a 1-store-per-cycle commit enqueue path while
        # allowing other younger non-store commits in the same cycle.
        fire = fire & ((~rob_is_stores[slot]) | ((~commit_store_seen) & stbuf_has_space))
        store_commit = fire & rob_is_stores[slot]
        stop = redirect | (fire & is_halt)

        commit_fires.append(fire)
        commit_count = commit_count + fire.zext(width=3)

        redirect_valid = redirect.select(consts.one1, redirect_valid)
        redirect_pc = redirect.select(pc_next, redirect_pc)
        redirect_checkpoint_id = redirect.select(rob_checkpoint_ids[slot], redirect_checkpoint_id)

        commit_store_fire = store_commit.select(consts.one1, commit_store_fire)
        commit_store_addr = store_commit.select(rob_st_addrs[slot], commit_store_addr)
        commit_store_data = store_commit.select(rob_st_datas[slot], commit_store_data)
        commit_store_size = store_commit.select(rob_st_sizes[slot], commit_store_size)
        commit_store_seen = store_commit.select(consts.one1, commit_store_seen)

        # --- sequential architectural state updates across commit slots ---
        op_setc_any = op_is(
            op,
            OP_C_SETC_EQ,
            OP_C_SETC_NE,
            OP_SETC_GEUI,
            OP_SETC_EQ,
            OP_SETC_NE,
            OP_SETC_AND,
            OP_SETC_OR,
            OP_SETC_LT,
            OP_SETC_LTU,
            OP_SETC_GE,
            OP_SETC_GEU,
            OP_SETC_EQI,
            OP_SETC_NEI,
            OP_SETC_ANDI,
            OP_SETC_ORI,
            OP_SETC_LTI,
            OP_SETC_GEI,
            OP_SETC_LTUI,
        )
        op_setc_tgt = op_is(op, OP_C_SETC_TGT)
        commit_cond_live = (fire & is_boundary).select(consts.zero1, commit_cond_live)
        commit_tgt_live = (fire & is_boundary).select(consts.zero64, commit_tgt_live)
        commit_cond_live = (fire & op_setc_any).select(val.trunc(width=1), commit_cond_live)
        commit_tgt_live = (fire & op_setc_tgt).select(val, commit_tgt_live)
        commit_cond_live = (fire & op_setc_tgt).select(consts.one1, commit_cond_live)

        br_kind_live = (fire & is_boundary & br_take).select(c(BK_FALL, width=3), br_kind_live)
        br_base_live = (fire & is_boundary & br_take).select(pc_this, br_base_live)
        br_off_live = (fire & is_boundary & br_take).select(consts.zero64, br_off_live)

        enter_new_block = fire & is_start_marker & (~br_take)
        commit_enter_new_blocks.append(enter_new_block)

        is_bstart_cond = op_is(op, OP_C_BSTART_COND)
        br_kind_live = (enter_new_block & is_bstart_cond).select(c(BK_COND, width=3), br_kind_live)
        br_base_live = (enter_new_block & is_bstart_cond).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_bstart_cond).select(val, br_off_live)

        is_bstart_direct = op_is(op, OP_C_BSTART_DIRECT)
        br_kind_live = (enter_new_block & is_bstart_direct).select(c(BK_DIRECT, width=3), br_kind_live)
        br_base_live = (enter_new_block & is_bstart_direct).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_bstart_direct).select(val, br_off_live)

        is_bstart_std_fall = op_is(op, OP_BSTART_STD_FALL)
        br_kind_live = (enter_new_block & is_bstart_std_fall).select(c(BK_FALL, width=3), br_kind_live)
        br_base_live = (enter_new_block & is_bstart_std_fall).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_bstart_std_fall).select(consts.zero64, br_off_live)

        is_bstart_std_direct = op_is(op, OP_BSTART_STD_DIRECT)
        br_kind_live = (enter_new_block & is_bstart_std_direct).select(c(BK_DIRECT, width=3), br_kind_live)
        br_base_live = (enter_new_block & is_bstart_std_direct).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_bstart_std_direct).select(val, br_off_live)

        is_bstart_std_cond = op_is(op, OP_BSTART_STD_COND)
        br_kind_live = (enter_new_block & is_bstart_std_cond).select(c(BK_COND, width=3), br_kind_live)
        br_base_live = (enter_new_block & is_bstart_std_cond).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_bstart_std_cond).select(val, br_off_live)

        is_bstart_call = op_is(op, OP_BSTART_STD_CALL)
        br_kind_live = (enter_new_block & is_bstart_call).select(c(BK_CALL, width=3), br_kind_live)
        br_base_live = (enter_new_block & is_bstart_call).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_bstart_call).select(val, br_off_live)

        brtype = val.trunc(width=3)
        kind_from_brtype = c(BK_FALL, width=3)
        kind_from_brtype = brtype.eq(c(2, width=3)).select(c(BK_DIRECT, width=3), kind_from_brtype)
        kind_from_brtype = brtype.eq(c(3, width=3)).select(c(BK_COND, width=3), kind_from_brtype)
        kind_from_brtype = brtype.eq(c(4, width=3)).select(c(BK_CALL, width=3), kind_from_brtype)
        kind_from_brtype = brtype.eq(c(5, width=3)).select(c(BK_IND, width=3), kind_from_brtype)
        kind_from_brtype = brtype.eq(c(6, width=3)).select(c(BK_ICALL, width=3), kind_from_brtype)
        kind_from_brtype = brtype.eq(c(7, width=3)).select(c(BK_RET, width=3), kind_from_brtype)
        is_bstart_std = op_is(op, OP_C_BSTART_STD)
        br_kind_live = (enter_new_block & is_bstart_std).select(kind_from_brtype, br_kind_live)
        br_base_live = (enter_new_block & is_bstart_std).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_bstart_std).select(consts.zero64, br_off_live)

        # Macro blocks (FENTRY/FEXIT/FRET.*) are treated as standalone fall-through blocks.
        br_kind_live = (enter_new_block & is_macro).select(c(BK_FALL, width=3), br_kind_live)
        br_base_live = (enter_new_block & is_macro).select(pc_this, br_base_live)
        br_off_live = (enter_new_block & is_macro).select(consts.zero64, br_off_live)

        is_bstop = op_is(op, OP_C_BSTOP)
        br_kind_live = (fire & is_bstop).select(c(BK_FALL, width=3), br_kind_live)
        br_base_live = (fire & is_bstop).select(pc_this, br_base_live)
        br_off_live = (fire & is_bstop).select(consts.zero64, br_off_live)

        pc_live = fire.select(pc_next, pc_live)

        commit_allow = commit_allow & fire & (~stop)

    # Canonical retired-store selection from committed slots (oldest first).
    # This is the single source used for memory side effects and same-cycle
    # forwarding to keep side effects aligned with retire trace semantics.
    store_sel_fire = consts.zero1
    store_sel_addr = consts.zero64
    store_sel_data = consts.zero64
    store_sel_size = consts.zero4
    for slot in range(p.commit_w):
        slot_store = commit_fires[slot] & rob_is_stores[slot]
        take = slot_store & (~store_sel_fire)
        store_sel_fire = slot_store.select(consts.one1, store_sel_fire)
        store_sel_addr = take.select(rob_st_addrs[slot], store_sel_addr)
        store_sel_data = take.select(rob_st_datas[slot], store_sel_data)
        store_sel_size = take.select(rob_st_sizes[slot], store_sel_size)
    commit_store_fire = store_sel_fire
    commit_store_addr = store_sel_addr
    commit_store_data = store_sel_data
    commit_store_size = store_sel_size

    commit_fire = commit_fires[0]
    commit_redirect = redirect_valid

    # --- store tracking (for conservative load ordering) ---
    store_pending = consts.zero1
    for i in range(p.rob_depth):
        store_pending = store_pending | (rob.valid[i].out() & rob.is_store[i].out())

    # --- issue selection (up to issue_w ready IQ entries) ---
    #
    # Conservative ordering for loads: do not issue a load if there is an older
    # store still in the ROB. This is evaluated per-IQ entry and excluded from
    # the pick candidates to avoid head-of-line blocking.
    sub_head = (~rob.head.out()) + c(1, width=p.rob_w)

    # --- IQ ready/can-issue ---
    alu_can_issue: list = []
    for i in range(p.iq_depth):
        v = iq_alu.valid[i].out()
        sl = iq_alu.srcl[i].out()
        sr = iq_alu.srcr[i].out()
        sp = iq_alu.srcp[i].out()
        sl_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sl, width=p.pregs)
        sr_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sr, width=p.pregs)
        sp_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sp, width=p.pregs)
        alu_can_issue.append(v & sl_rdy & sr_rdy & sp_rdy)

    bru_can_issue: list = []
    for i in range(p.iq_depth):
        v = iq_bru.valid[i].out()
        sl = iq_bru.srcl[i].out()
        sr = iq_bru.srcr[i].out()
        sp = iq_bru.srcp[i].out()
        sl_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sl, width=p.pregs)
        sr_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sr, width=p.pregs)
        sp_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sp, width=p.pregs)
        bru_can_issue.append(v & sl_rdy & sr_rdy & sp_rdy)

    lsu_is_load: list = []
    lsu_is_store: list = []
    lsu_older_store_pending: list = []
    lsu_can_issue: list = []
    for i in range(p.iq_depth):
        v = iq_lsu.valid[i].out()
        sl = iq_lsu.srcl[i].out()
        sr = iq_lsu.srcr[i].out()
        sp = iq_lsu.srcp[i].out()
        sl_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sl, width=p.pregs)
        sr_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sr, width=p.pregs)
        sp_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=sp, width=p.pregs)
        ready = v & sl_rdy & sr_rdy & sp_rdy

        op_i = iq_lsu.op[i].out()
        # Base LSU readiness. Lane0 load disambiguation/forwarding below applies
        # the memory-first ordering checks against older stores.
        is_load_i = op_is(
            op_i,
            OP_LWI,
            OP_C_LWI,
            OP_LBI,
            OP_LBUI,
            OP_LHI,
            OP_LHUI,
            OP_LWUI,
            OP_LDI,
            OP_C_LDI,
            OP_LB,
            OP_LBU,
            OP_LH,
            OP_LHU,
            OP_LW,
            OP_LWU,
            OP_LD,
            OP_HL_LB_PCR,
            OP_HL_LBU_PCR,
            OP_HL_LH_PCR,
            OP_HL_LHU_PCR,
            OP_HL_LW_PCR,
            OP_HL_LWU_PCR,
            OP_HL_LD_PCR,
        )
        is_store_i = op_is(
            op_i,
            OP_SBI,
            OP_SHI,
            OP_SWI,
            OP_C_SWI,
            OP_SDI,
            OP_C_SDI,
            OP_SB,
            OP_SH,
            OP_SW,
            OP_SD,
            OP_HL_SB_PCR,
            OP_HL_SH_PCR,
            OP_HL_SW_PCR,
            OP_HL_SD_PCR,
        )

        older_store = consts.zero1
        ok = ready

        lsu_is_load.append(is_load_i)
        lsu_is_store.append(is_store_i)
        lsu_older_store_pending.append(older_store)
        lsu_can_issue.append(ok)

    # --- issue selection per IQ ---
    def pick_oldest(*, can_issue: list, iq, width: int):
        issue_valids = []
        issue_idxs = []
        for slot in range(width):
            v = consts.zero1
            idx = c(0, width=p.iq_w)
            best_age = c((1 << p.rob_w) - 1, width=p.rob_w)
            for i in range(p.iq_depth):
                cidx = c(i, width=p.iq_w)
                exclude = consts.zero1
                for prev in range(slot):
                    exclude = exclude | (issue_valids[prev] & issue_idxs[prev].eq(cidx))
                cand = can_issue[i] & (~exclude)
                # Smaller distance-from-head means older in program order.
                age = iq.rob[i].out() + sub_head
                better = (~v) | age.ult(best_age)
                take = cand & better
                v = take.select(consts.one1, v)
                idx = take.select(cidx, idx)
                best_age = take.select(age, best_age)
            issue_valids.append(v)
            issue_idxs.append(idx)
        return issue_valids, issue_idxs

    alu_issue_valids, alu_issue_idxs = pick_oldest(can_issue=alu_can_issue, iq=iq_alu, width=p.alu_w)
    bru_issue_valids, bru_issue_idxs = pick_oldest(can_issue=bru_can_issue, iq=iq_bru, width=p.bru_w)
    lsu_issue_valids, lsu_issue_idxs = pick_oldest(can_issue=lsu_can_issue, iq=iq_lsu, width=p.lsu_w)

    # Slot ordering: LSU, BRU, ALU (stable debug lane0 = LSU).
    issue_valids = lsu_issue_valids + bru_issue_valids + alu_issue_valids
    issue_idxs = lsu_issue_idxs + bru_issue_idxs + alu_issue_idxs
    issue_iqs = ([iq_lsu] * p.lsu_w) + ([iq_bru] * p.bru_w) + ([iq_alu] * p.alu_w)

    issue_fires = []
    for slot in range(p.issue_w):
        issue_fires.append(can_run & (~commit_redirect) & issue_valids[slot])

    # Lane0 retained for trace/debug outputs.
    issue_fire = issue_fires[0]
    issue_idx = issue_idxs[0]

    uop_robs = []
    uop_ops = []
    uop_pcs = []
    uop_imms = []
    uop_sls = []
    uop_srs = []
    uop_srcr_types = []
    uop_shamts = []
    uop_sps = []
    uop_pdsts = []
    uop_has_dsts = []
    for slot in range(p.issue_w):
        iq = issue_iqs[slot]
        idx = issue_idxs[slot]
        uop_robs.append(mux_by_uindex(m, idx=idx, items=iq.rob, default=c(0, width=p.rob_w)))
        uop_ops.append(mux_by_uindex(m, idx=idx, items=iq.op, default=c(0, width=12)))
        uop_pcs.append(mux_by_uindex(m, idx=idx, items=iq.pc, default=consts.zero64))
        uop_imms.append(mux_by_uindex(m, idx=idx, items=iq.imm, default=consts.zero64))
        uop_sls.append(mux_by_uindex(m, idx=idx, items=iq.srcl, default=tag0))
        uop_srs.append(mux_by_uindex(m, idx=idx, items=iq.srcr, default=tag0))
        uop_srcr_types.append(mux_by_uindex(m, idx=idx, items=iq.srcr_type, default=c(0, width=2)))
        uop_shamts.append(mux_by_uindex(m, idx=idx, items=iq.shamt, default=consts.zero6))
        uop_sps.append(mux_by_uindex(m, idx=idx, items=iq.srcp, default=tag0))
        uop_pdsts.append(mux_by_uindex(m, idx=idx, items=iq.pdst, default=tag0))
        uop_has_dsts.append(mux_by_uindex(m, idx=idx, items=iq.has_dst, default=consts.zero1))

    # Lane0 named views (stable trace hooks).
    uop_rob = uop_robs[0]
    uop_op = uop_ops[0]
    uop_pc = uop_pcs[0]
    uop_imm = uop_imms[0]
    uop_sl = uop_sls[0]
    uop_sr = uop_srs[0]
    uop_sp = uop_sps[0]
    uop_pdst = uop_pdsts[0]
    uop_has_dst = uop_has_dsts[0]

    # PRF reads + execute for each issued uop.
    sl_vals = []
    sr_vals = []
    sp_vals = []
    exs = []
    for slot in range(p.issue_w):
        sl_vals.append(mux_by_uindex(m, idx=uop_sls[slot], items=prf, default=consts.zero64))
        sr_vals.append(mux_by_uindex(m, idx=uop_srs[slot], items=prf, default=consts.zero64))
        sp_vals.append(mux_by_uindex(m, idx=uop_sps[slot], items=prf, default=consts.zero64))
        exs.append(
            exec_uop_comb(
                m,
                op=uop_ops[slot],
                pc=uop_pcs[slot],
                imm=uop_imms[slot],
                srcl_val=sl_vals[slot],
                srcr_val=sr_vals[slot],
                srcr_type=uop_srcr_types[slot],
                shamt=uop_shamts[slot],
                srcp_val=sp_vals[slot],
                consts=consts,
            )
        )

    # Lane0 values for debug/trace.
    sl_val = sl_vals[0]
    sr_val = sr_vals[0]
    sp_val = sp_vals[0]

    issue_fires_eff = [issue_fires[i] for i in range(p.issue_w)]

    # Memory disambiguation/forwarding for the LSU lane (lane0).
    lsu_load_fire_raw = issue_fires[0] & exs[0].is_load
    lsu_load_addr = exs[0].addr
    lsu_load_rob = uop_robs[0]
    lsu_load_dist = lsu_load_rob + sub_head
    lsu_older_store_pending_lane0 = consts.zero1
    lsu_forward_hit_lane0 = consts.zero1
    lsu_forward_data_lane0 = consts.zero64

    for i in range(p.rob_depth):
        idx = c(i, width=p.rob_w)
        dist = idx + sub_head
        older = dist.ult(lsu_load_dist)
        st = rob.valid[i].out() & rob.is_store[i].out() & older
        st_pending = st & (~rob.done[i].out())
        st_ready = st & rob.done[i].out()
        st_match = st_ready & rob.store_addr[i].out().eq(lsu_load_addr)
        lsu_older_store_pending_lane0 = lsu_older_store_pending_lane0 | (lsu_load_fire_raw & st_pending)
        lsu_forward_hit_lane0 = lsu_forward_hit_lane0 | (lsu_load_fire_raw & st_match)
        lsu_forward_data_lane0 = (lsu_load_fire_raw & st_match).select(rob.store_data[i].out(), lsu_forward_data_lane0)

    # Forward from committed stores that are waiting in the store buffer.
    for i in range(p.sq_entries):
        st_match = stbuf_valid[i].out() & stbuf_addr[i].out().eq(lsu_load_addr)
        lsu_forward_hit_lane0 = lsu_forward_hit_lane0 | (lsu_load_fire_raw & st_match)
        lsu_forward_data_lane0 = (lsu_load_fire_raw & st_match).select(stbuf_data[i].out(), lsu_forward_data_lane0)

    # Same-cycle head-store commit bypass for younger lane0 loads.
    commit_store_match_lane0 = lsu_load_fire_raw & commit_store_fire & commit_store_addr.eq(lsu_load_addr)
    lsu_forward_hit_lane0 = lsu_forward_hit_lane0 | commit_store_match_lane0
    lsu_forward_data_lane0 = commit_store_match_lane0.select(commit_store_data, lsu_forward_data_lane0)

    lsu_block_lane0 = lsu_load_fire_raw & lsu_older_store_pending_lane0
    issue_fires_eff[0] = issue_fires[0] & (~lsu_block_lane0)
    issue_fire = issue_fires_eff[0]

    load_fires = []
    load_mem_fires = []
    store_fires = []
    any_load_mem_fire = consts.zero1
    load_addr = consts.zero64
    for slot in range(p.issue_w):
        ld = issue_fires_eff[slot] & exs[slot].is_load
        st = issue_fires_eff[slot] & exs[slot].is_store
        ld_mem = ld
        if slot == 0:
            ld_mem = ld & (~lsu_forward_hit_lane0)
        load_fires.append(ld)
        load_mem_fires.append(ld_mem)
        store_fires.append(st)
        any_load_mem_fire = any_load_mem_fire | ld_mem
        load_addr = ld_mem.select(exs[slot].addr, load_addr)

    issued_is_load = load_fires[0]
    issued_is_store = store_fires[0]
    older_store_pending = lsu_older_store_pending_lane0

    # LSU violation replay state (updated after wb metadata is formed).
    replay_set = consts.zero1
    replay_set_store_rob = state.replay_store_rob.out()
    replay_set_pc = state.replay_pc.out()
    lsu_violation_detected = consts.zero1

    # --- template macro engine (FENTRY/FEXIT/FRET.*) ---
    macro_active = state.macro_active.out()
    macro_phase = state.macro_phase.out()
    macro_op = state.macro_op.out()
    macro_begin = state.macro_begin.out()
    macro_end = state.macro_end.out()
    macro_stacksize = state.macro_stacksize.out()
    # Frame adjust follows ISA template immediate directly.
    macro_callframe_size = consts.zero64
    macro_frame_adj = macro_stacksize + macro_callframe_size
    macro_reg = state.macro_reg.out()
    macro_i = state.macro_i.out()
    macro_sp_base = state.macro_sp_base.out()

    macro_is_fentry = ctu["macro_is_fentry"]
    macro_phase_init = ctu["phase_init"]
    macro_phase_mem = ctu["phase_mem"]
    macro_phase_sp = ctu["phase_sp"]
    macro_phase_setc = ctu["phase_setc"]
    macro_off_ok = ctu["off_ok"]
    macro_is_fexit = macro_op.eq(c(OP_FEXIT, width=12))
    macro_is_fret_ra = macro_op.eq(c(OP_FRET_RA, width=12))
    macro_is_fret_stk = macro_op.eq(c(OP_FRET_STK, width=12))

    # CodeTemplateUnit emits one template-uop per cycle while active.
    macro_uop_valid = ctu["uop_valid"]
    macro_uop_kind = ctu["uop_kind"]
    macro_uop_reg = ctu["uop_reg"]
    macro_uop_addr = ctu["uop_addr"]
    macro_uop_is_sp_sub = ctu["uop_is_sp_sub"]
    macro_uop_is_store = ctu["uop_is_store"]
    macro_uop_is_load = ctu["uop_is_load"]
    macro_uop_is_sp_add = ctu["uop_is_sp_add"]
    macro_uop_is_setc_tgt = ctu["uop_is_setc_tgt"]

    # D-memory read arbitration: macro restore-load > LSU load.
    macro_mem_read = macro_uop_is_load
    dmem_raddr = macro_mem_read.select(macro_uop_addr, any_load_mem_fire.select(load_addr, consts.zero64))

    # Macro/template uop operand reads.
    cmap_now = [ren.cmap[i].out() for i in range(p.aregs)]
    macro_reg_tag = mux_by_uindex(m, idx=macro_uop_reg, items=cmap_now, default=tag0)
    macro_reg_val = mux_by_uindex(m, idx=macro_reg_tag, items=prf, default=consts.zero64)
    macro_sp_tag = ren.cmap[1].out()
    macro_sp_val = mux_by_uindex(m, idx=macro_sp_tag, items=prf, default=consts.zero64)
    macro_reg_is_gpr = macro_uop_reg.ult(c(24, width=6))
    macro_reg_not_zero = ~macro_uop_reg.eq(c(0, width=6))
    macro_store_fire = macro_uop_is_store & macro_reg_is_gpr & macro_reg_not_zero
    macro_store_addr = macro_uop_addr
    macro_store_data = macro_reg_val
    macro_store_size = c(8, width=4)

    # MMIO (QEMU virt compatibility).
    #
    # - UART data: 0x1000_0000 (write low byte)
    # - EXIT:      0x1000_0004 (write exit code; stop simulation)
    mmio_uart = commit_store_fire & commit_store_addr.eq(c(0x1000_0000, width=64))
    mmio_exit = commit_store_fire & commit_store_addr.eq(c(0x1000_0004, width=64))
    mmio_any = mmio_uart | mmio_exit

    mmio_uart_data = mmio_uart.select(commit_store_data.trunc(width=8), c(0, width=8))
    mmio_exit_code = mmio_exit.select(commit_store_data.trunc(width=32), c(0, width=32))

    # Preserve store ordering:
    # - If the committed-store buffer already has older entries, enqueue all new
    #   committed stores (unless MMIO) so younger writes cannot bypass older ones.
    # - If macro uses the single write port this cycle, enqueue as well.
    stbuf_empty = stbuf_count.out().eq(c(0, width=p.sq_w + 1))
    commit_store_defer = commit_store_fire & (~mmio_any) & (macro_store_fire | (~stbuf_empty))
    stbuf_enq_fire = commit_store_defer
    stbuf_enq_idx = stbuf_tail.out()
    stbuf_enq_tail = stbuf_tail.out() + c(1, width=p.sq_w)

    stbuf_drain_fire = (~macro_store_fire) & (~commit_store_fire) & (~stbuf_count.out().eq(c(0, width=p.sq_w + 1)))
    stbuf_drain_addr = mux_by_uindex(m, idx=stbuf_head.out(), items=stbuf_addr, default=consts.zero64)
    stbuf_drain_data = mux_by_uindex(m, idx=stbuf_head.out(), items=stbuf_data, default=consts.zero64)
    stbuf_drain_size = mux_by_uindex(m, idx=stbuf_head.out(), items=stbuf_size, default=consts.zero4)
    stbuf_drain_head = stbuf_head.out() + c(1, width=p.sq_w)

    commit_store_write_through = commit_store_fire & (~mmio_any) & (~commit_store_defer)
    mem_wvalid = macro_store_fire | commit_store_write_through | stbuf_drain_fire
    mem_waddr = macro_store_fire.select(
        macro_store_addr,
        commit_store_write_through.select(commit_store_addr, stbuf_drain_addr),
    )
    dmem_wdata = macro_store_fire.select(
        macro_store_data,
        commit_store_write_through.select(commit_store_data, stbuf_drain_data),
    )
    mem_wsize = macro_store_fire.select(
        macro_store_size,
        commit_store_write_through.select(commit_store_size, stbuf_drain_size),
    )

    dmem_wsrc = c(0, width=2)
    dmem_wsrc = macro_store_fire.select(c(1, width=2), dmem_wsrc)
    dmem_wsrc = commit_store_write_through.select(c(2, width=2), dmem_wsrc)
    dmem_wsrc = stbuf_drain_fire.select(c(3, width=2), dmem_wsrc)

    # Store write port (writes at clk edge). Stop-at-store ensures that at most
    # one store commits per cycle in this bring-up model; the macro engine
    # consumes the same single write port.
    wstrb = consts.zero8
    wstrb = mem_wsize.eq(c(1, width=4)).select(c(0x01, width=8), wstrb)
    wstrb = mem_wsize.eq(c(2, width=4)).select(c(0x03, width=8), wstrb)
    wstrb = mem_wsize.eq(c(4, width=4)).select(c(0x0F, width=8), wstrb)
    wstrb = mem_wsize.eq(c(8, width=4)).select(c(0xFF, width=8), wstrb)

    # Store buffer register updates.
    for i in range(p.sq_entries):
        idx = c(i, width=p.sq_w)
        do_enq = stbuf_enq_fire & stbuf_enq_idx.eq(idx)
        do_drain = stbuf_drain_fire & stbuf_head.out().eq(idx)
        v_next = stbuf_valid[i].out()
        v_next = do_drain.select(consts.zero1, v_next)
        v_next = do_enq.select(consts.one1, v_next)
        stbuf_valid[i].set(v_next)
        stbuf_addr[i].set(commit_store_addr, when=do_enq)
        stbuf_data[i].set(commit_store_data, when=do_enq)
        stbuf_size[i].set(commit_store_size, when=do_enq)

    stbuf_head_next = stbuf_head.out()
    stbuf_tail_next = stbuf_tail.out()
    stbuf_count_next = stbuf_count.out()
    stbuf_tail_next = stbuf_enq_fire.select(stbuf_enq_tail, stbuf_tail_next)
    stbuf_count_next = stbuf_enq_fire.select(stbuf_count_next + c(1, width=p.sq_w + 1), stbuf_count_next)
    stbuf_head_next = stbuf_drain_fire.select(stbuf_drain_head, stbuf_head_next)
    stbuf_count_next = stbuf_drain_fire.select(stbuf_count_next - c(1, width=p.sq_w + 1), stbuf_count_next)
    stbuf_head.set(stbuf_head_next)
    stbuf_tail.set(stbuf_tail_next)
    stbuf_count.set(stbuf_count_next)

    dmem_rdata = dmem_rdata_i
    macro_load_fwd_hit = consts.zero1
    macro_load_fwd_data = consts.zero64
    for i in range(p.sq_entries):
        st_match = stbuf_valid[i].out() & stbuf_addr[i].out().eq(macro_uop_addr)
        macro_load_fwd_hit = (macro_uop_is_load & st_match).select(consts.one1, macro_load_fwd_hit)
        macro_load_fwd_data = (macro_uop_is_load & st_match).select(stbuf_data[i].out(), macro_load_fwd_data)
    macro_load_data = macro_load_fwd_hit.select(macro_load_fwd_data, dmem_rdata)
    # FRET.STK must consume the loaded stack RA value. Only FRET.RA uses the
    # saved-RA bypass path.
    macro_restore_ra = macro_uop_is_load & op_is(macro_op, OP_FRET_RA) & macro_uop_reg.eq(c(10, width=6))
    macro_load_data_eff = macro_restore_ra.select(state.macro_saved_ra.out(), macro_load_data)
    # FRET.STK can finish immediately after restoring RA (e.g. [ra~ra]).
    # In that case there is no standalone SETC_TGT phase; consume the restored
    # RA value as return target on the RA-load step.
    macro_setc_from_fret_stk_ra_load = macro_uop_is_load & macro_is_fret_stk & macro_uop_reg.eq(c(10, width=6))
    macro_setc_tgt_fire = macro_uop_is_setc_tgt | macro_setc_from_fret_stk_ra_load
    macro_setc_tgt_data = ret_ra_val
    macro_setc_tgt_data = macro_setc_from_fret_stk_ra_load.select(macro_load_data_eff, macro_setc_tgt_data)
    macro_setc_tgt_data = (macro_uop_is_setc_tgt & macro_is_fret_stk).select(state.macro_saved_ra.out(), macro_setc_tgt_data)

    macro_is_restore = macro_active & (~macro_is_fentry)

    # Macro PRF write port (one write per cycle).
    macro_reg_write = macro_uop_is_load & macro_reg_is_gpr & macro_reg_not_zero
    macro_sp_write_init = macro_uop_is_sp_sub
    macro_sp_write_restore = macro_uop_is_sp_add

    macro_prf_we = macro_reg_write | macro_sp_write_init | macro_sp_write_restore
    macro_prf_tag = macro_sp_tag
    macro_prf_data = consts.zero64
    macro_prf_tag = macro_reg_write.select(macro_reg_tag, macro_prf_tag)
    macro_prf_data = macro_reg_write.select(macro_load_data_eff, macro_prf_data)
    macro_prf_data = macro_sp_write_restore.select(macro_sp_val + macro_frame_adj, macro_prf_data)
    macro_prf_data = macro_sp_write_init.select(macro_sp_val - macro_frame_adj, macro_prf_data)

    # Load result (uses dmem_rdata in the same cycle raddr is set).
    load8 = dmem_rdata.trunc(width=8)
    load16 = dmem_rdata.trunc(width=16)
    load32 = dmem_rdata.trunc(width=32)
    load_lb = load8.sext(width=64)
    load_lbu = load8.zext(width=64)
    load_lh = load16.sext(width=64)
    load_lhu = load16.zext(width=64)
    load_lw = load32.sext(width=64)
    load_lwu = load32.zext(width=64)
    load_ld = dmem_rdata
    lsu_forward_active = (issue_fires_eff[0] & exs[0].is_load) & lsu_forward_hit_lane0
    wb_fires = []
    wb_robs = []
    wb_pdsts = []
    wb_values = []
    wb_fire_has_dsts = []
    wb_onehots = []
    for slot in range(p.issue_w):
        wb_fire = issue_fires_eff[slot]
        wb_rob = uop_robs[slot]
        wb_pdst = uop_pdsts[slot]
        op = uop_ops[slot]
        load_val = load_lw
        load_val = op_is(op, OP_LB, OP_LBI, OP_HL_LB_PCR).select(load_lb, load_val)
        load_val = op_is(op, OP_LBU, OP_LBUI, OP_HL_LBU_PCR).select(load_lbu, load_val)
        load_val = op_is(op, OP_LH, OP_LHI, OP_HL_LH_PCR).select(load_lh, load_val)
        load_val = op_is(op, OP_LHU, OP_LHUI, OP_HL_LHU_PCR).select(load_lhu, load_val)
        load_val = op_is(op, OP_LWI, OP_C_LWI, OP_LW, OP_HL_LW_PCR).select(load_lw, load_val)
        load_val = op_is(op, OP_LWU, OP_LWUI, OP_HL_LWU_PCR).select(load_lwu, load_val)
        load_val = op_is(op, OP_LD, OP_LDI, OP_C_LDI, OP_HL_LD_PCR).select(load_ld, load_val)
        if slot == 0:
            fwd8 = lsu_forward_data_lane0.trunc(width=8)
            fwd16 = lsu_forward_data_lane0.trunc(width=16)
            fwd32 = lsu_forward_data_lane0.trunc(width=32)
            load_fwd = fwd32.sext(width=64)
            load_fwd = op_is(op, OP_LB, OP_LBI, OP_HL_LB_PCR).select(fwd8.sext(width=64), load_fwd)
            load_fwd = op_is(op, OP_LBU, OP_LBUI, OP_HL_LBU_PCR).select(fwd8.zext(width=64), load_fwd)
            load_fwd = op_is(op, OP_LH, OP_LHI, OP_HL_LH_PCR).select(fwd16.sext(width=64), load_fwd)
            load_fwd = op_is(op, OP_LHU, OP_LHUI, OP_HL_LHU_PCR).select(fwd16.zext(width=64), load_fwd)
            load_fwd = op_is(op, OP_LWI, OP_C_LWI, OP_LW, OP_HL_LW_PCR).select(fwd32.sext(width=64), load_fwd)
            load_fwd = op_is(op, OP_LWU, OP_LWUI, OP_HL_LWU_PCR).select(fwd32.zext(width=64), load_fwd)
            load_fwd = op_is(op, OP_LD, OP_LDI, OP_C_LDI, OP_HL_LD_PCR).select(lsu_forward_data_lane0, load_fwd)
            load_val = lsu_forward_active.select(load_fwd, load_val)
        wb_value = load_fires[slot].select(load_val, exs[slot].alu)
        wb_has_dst = uop_has_dsts[slot] & (~store_fires[slot])
        wb_fire_has_dst = wb_fire & wb_has_dst

        wb_fires.append(wb_fire)
        wb_robs.append(wb_rob)
        wb_pdsts.append(wb_pdst)
        wb_values.append(wb_value)
        wb_fire_has_dsts.append(wb_fire_has_dst)
        wb_onehots.append(onehot_from_tag(m, tag=wb_pdst, width=p.pregs, tag_width=p.ptag_w))

    # LSU violation detection: if an older store resolves to the same address
    # as a younger already-executed load, request replay from that load PC.
    for slot in range(p.issue_w):
        st_fire = store_fires[slot]
        st_rob = wb_robs[slot]
        st_addr = exs[slot].addr
        st_dist = st_rob + sub_head

        hit = consts.zero1
        hit_pc = consts.zero64
        hit_age = c((1 << p.rob_w) - 1, width=p.rob_w)
        for i in range(p.rob_depth):
            idx = c(i, width=p.rob_w)
            dist = idx + sub_head
            younger = st_dist.ult(dist)
            ld_done = rob.valid[i].out() & rob.done[i].out() & rob.is_load[i].out()
            addr_match = rob.load_addr[i].out().eq(st_addr)
            cand = st_fire & ld_done & younger & addr_match
            better = (~hit) | dist.ult(hit_age)
            take = cand & better
            hit = take.select(consts.one1, hit)
            hit_age = take.select(dist, hit_age)
            hit_pc = take.select(rob.pc[i].out(), hit_pc)

        set_this = hit & (~state.replay_pending.out()) & (~replay_set)
        replay_set = set_this.select(consts.one1, replay_set)
        replay_set_store_rob = set_this.select(st_rob, replay_set_store_rob)
        replay_set_pc = set_this.select(hit_pc, replay_set_pc)

    lsu_violation_detected = replay_set

    # --- dispatch (decode + rename + enqueue) ---
    f4_valid = f4_valid_i
    f4_pc = f4_pc_i
    f4_window = f4_window_i

    f4_bundle = decode_f4_bundle(m, f4_window)

    disp_valids = []
    disp_pcs = []
    disp_ops = []
    disp_lens = []
    disp_regdsts = []
    disp_srcls = []
    disp_srcrs = []
    disp_srcr_types = []
    disp_shamts = []
    disp_srcps = []
    disp_imms = []
    disp_insn_raws = []
    disp_is_start_marker = []
    disp_push_t = []
    disp_push_u = []
    disp_is_store = []
    disp_dst_is_gpr = []
    disp_need_pdst = []
    disp_dst_kind = []
    disp_checkpoint_ids = []

    for slot in range(p.dispatch_w):
        dec = f4_bundle.dec[slot]
        v = f4_valid & f4_bundle.valid[slot]
        off = f4_bundle.off_bytes[slot]
        pc = f4_pc + off.zext(width=64)

        op = dec.op
        ln = dec.len_bytes
        regdst = dec.regdst
        srcl = dec.srcl
        srcr = dec.srcr
        srcr_type = dec.srcr_type
        shamt = dec.shamt
        srcp = dec.srcp
        imm = dec.imm
        off_sh = off.zext(width=6).shl(amount=3)
        slot_window = lshr_var(m, f4_window, off_sh)
        insn_raw = slot_window
        insn_raw = ln.eq(c(2, width=3)).select(slot_window & c(0xFFFF, width=64), insn_raw)
        insn_raw = ln.eq(c(4, width=3)).select(slot_window & c(0xFFFF_FFFF, width=64), insn_raw)
        insn_raw = ln.eq(c(6, width=3)).select(slot_window & c(0xFFFF_FFFF_FFFF, width=64), insn_raw)

        is_macro = op_is(op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
        is_start = (
            op_is(
                op,
                OP_C_BSTART_STD,
                OP_C_BSTART_COND,
                OP_C_BSTART_DIRECT,
                OP_BSTART_STD_FALL,
                OP_BSTART_STD_DIRECT,
                OP_BSTART_STD_COND,
                OP_BSTART_STD_CALL,
            )
            | is_macro
        )
        push_t = regdst.eq(c(31, width=6)) | op.eq(c(OP_C_LWI, width=12))
        push_u = regdst.eq(c(30, width=6))
        is_store = op_is(
            op,
            OP_SBI,
            OP_SHI,
            OP_SWI,
            OP_C_SWI,
            OP_SDI,
            OP_C_SDI,
            OP_SB,
            OP_SH,
            OP_SW,
            OP_SD,
            OP_HL_SB_PCR,
            OP_HL_SH_PCR,
            OP_HL_SW_PCR,
            OP_HL_SD_PCR,
        )

        dst_is_invalid = regdst.eq(c(REG_INVALID, width=6))
        dst_is_zero = regdst.eq(c(0, width=6))
        dst_is_gpr_range = (~regdst[5]) & (~(regdst[4] & regdst[3]))
        dst_is_gpr = dst_is_gpr_range & (~dst_is_invalid) & (~dst_is_zero) & (~push_t) & (~push_u)
        need_pdst = dst_is_gpr | push_t | push_u

        dk = c(0, width=2)
        dk = dst_is_gpr.select(c(1, width=2), dk)
        dk = push_t.select(c(2, width=2), dk)
        dk = push_u.select(c(3, width=2), dk)

        disp_valids.append(v)
        disp_pcs.append(pc)
        disp_ops.append(op)
        disp_lens.append(ln)
        disp_regdsts.append(regdst)
        disp_srcls.append(srcl)
        disp_srcrs.append(srcr)
        disp_srcr_types.append(srcr_type)
        disp_shamts.append(shamt)
        disp_srcps.append(srcp)
        disp_imms.append(imm)
        disp_insn_raws.append(insn_raw)
        disp_is_start_marker.append(is_start)
        disp_push_t.append(push_t)
        disp_push_u.append(push_u)
        disp_is_store.append(is_store)
        disp_dst_is_gpr.append(dst_is_gpr)
        disp_need_pdst.append(need_pdst)
        disp_dst_kind.append(dk)
        ckpt_tag = is_start.select(f4_checkpoint_i + c(slot, width=6), c(0, width=6))
        disp_checkpoint_ids.append(ckpt_tag)

    # Lane0 decode (stable trace hook).
    dec_op = disp_ops[0]

    # Dispatch count (0..dispatch_w).
    disp_count = c(0, width=3)
    for slot in range(p.dispatch_w):
        disp_count = disp_count + disp_valids[slot].zext(width=3)

    # ROB space check: rob.count + disp_count <= rob_depth.
    rob_cnt_after = rob.count.out() + disp_count.zext(width=p.rob_w + 1)
    rob_space_ok = rob_cnt_after.ult(c(p.rob_depth + 1, width=p.rob_w + 1))

    # IQ routing + allocation: pick distinct free slots per-IQ for each lane.
    disp_to_alu = []
    disp_to_bru = []
    disp_to_lsu = []
    for slot in range(p.dispatch_w):
        op = disp_ops[slot]
        is_macro = op_is(op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
        is_load = op_is(
            op,
            OP_LWI,
            OP_C_LWI,
            OP_LBI,
            OP_LBUI,
            OP_LHI,
            OP_LHUI,
            OP_LWUI,
            OP_LB,
            OP_LBU,
            OP_LH,
            OP_LHU,
            OP_LW,
            OP_LWU,
            OP_LD,
            OP_LDI,
            OP_C_LDI,
            OP_HL_LB_PCR,
            OP_HL_LBU_PCR,
            OP_HL_LH_PCR,
            OP_HL_LHU_PCR,
            OP_HL_LW_PCR,
            OP_HL_LWU_PCR,
            OP_HL_LD_PCR,
        )
        is_store = disp_is_store[slot]
        is_mem = is_load | is_store
        is_bru = op_is(
            op,
            OP_C_BSTART_STD,
            OP_C_BSTART_COND,
            OP_C_BSTART_DIRECT,
            OP_C_BSTOP,
            OP_BSTART_STD_FALL,
            OP_BSTART_STD_DIRECT,
            OP_BSTART_STD_COND,
            OP_BSTART_STD_CALL,
            OP_FENTRY,
            OP_FEXIT,
            OP_FRET_RA,
            OP_FRET_STK,
            OP_C_SETC_EQ,
            OP_C_SETC_NE,
            OP_C_SETC_TGT,
        )
        to_lsu = is_mem
        to_bru = (~to_lsu) & is_bru & (~is_macro)
        to_alu = (~to_lsu) & (~to_bru) & (~is_macro)
        disp_to_alu.append(to_alu)
        disp_to_bru.append(to_bru)
        disp_to_lsu.append(to_lsu)

    alu_alloc_valids = []
    alu_alloc_idxs = []
    bru_alloc_valids = []
    bru_alloc_idxs = []
    lsu_alloc_valids = []
    lsu_alloc_idxs = []
    for slot in range(p.dispatch_w):
        # --- ALU IQ ---
        req_alu = disp_valids[slot] & disp_to_alu[slot]
        v = consts.zero1
        idx = c(0, width=p.iq_w)
        for i in range(p.iq_depth):
            cidx = c(i, width=p.iq_w)
            free = ~iq_alu.valid[i].out()
            exclude = consts.zero1
            for prev in range(slot):
                prev_req = disp_valids[prev] & disp_to_alu[prev]
                exclude = exclude | (prev_req & alu_alloc_valids[prev] & alu_alloc_idxs[prev].eq(cidx))
            cand = req_alu & free & (~exclude)
            take = cand & (~v)
            v = take.select(consts.one1, v)
            idx = take.select(cidx, idx)
        alu_alloc_valids.append(v)
        alu_alloc_idxs.append(idx)

        # --- BRU IQ ---
        req_bru = disp_valids[slot] & disp_to_bru[slot]
        v = consts.zero1
        idx = c(0, width=p.iq_w)
        for i in range(p.iq_depth):
            cidx = c(i, width=p.iq_w)
            free = ~iq_bru.valid[i].out()
            exclude = consts.zero1
            for prev in range(slot):
                prev_req = disp_valids[prev] & disp_to_bru[prev]
                exclude = exclude | (prev_req & bru_alloc_valids[prev] & bru_alloc_idxs[prev].eq(cidx))
            cand = req_bru & free & (~exclude)
            take = cand & (~v)
            v = take.select(consts.one1, v)
            idx = take.select(cidx, idx)
        bru_alloc_valids.append(v)
        bru_alloc_idxs.append(idx)

        # --- LSU IQ ---
        req_lsu = disp_valids[slot] & disp_to_lsu[slot]
        v = consts.zero1
        idx = c(0, width=p.iq_w)
        for i in range(p.iq_depth):
            cidx = c(i, width=p.iq_w)
            free = ~iq_lsu.valid[i].out()
            exclude = consts.zero1
            for prev in range(slot):
                prev_req = disp_valids[prev] & disp_to_lsu[prev]
                exclude = exclude | (prev_req & lsu_alloc_valids[prev] & lsu_alloc_idxs[prev].eq(cidx))
            cand = req_lsu & free & (~exclude)
            take = cand & (~v)
            v = take.select(consts.one1, v)
            idx = take.select(cidx, idx)
        lsu_alloc_valids.append(v)
        lsu_alloc_idxs.append(idx)

    alu_alloc_ok = consts.one1
    bru_alloc_ok = consts.one1
    lsu_alloc_ok = consts.one1
    for slot in range(p.dispatch_w):
        req_alu = disp_valids[slot] & disp_to_alu[slot]
        req_bru = disp_valids[slot] & disp_to_bru[slot]
        req_lsu = disp_valids[slot] & disp_to_lsu[slot]
        alu_alloc_ok = alu_alloc_ok & ((~req_alu) | alu_alloc_valids[slot])
        bru_alloc_ok = bru_alloc_ok & ((~req_bru) | bru_alloc_valids[slot])
        lsu_alloc_ok = lsu_alloc_ok & ((~req_lsu) | lsu_alloc_valids[slot])
    iq_alloc_ok = alu_alloc_ok & bru_alloc_ok & lsu_alloc_ok

    # Physical register allocation (up to dispatch_w per cycle).
    preg_alloc_valids = []
    preg_alloc_tags = []
    preg_alloc_onehots = []
    free_mask_stage = ren.free_mask.out()
    for slot in range(p.dispatch_w):
        req = disp_valids[slot] & disp_need_pdst[slot]
        v, tag, oh = alloc_from_free_mask(m, free_mask=free_mask_stage, width=p.pregs, tag_width=p.ptag_w)
        free_mask_stage = req.select(free_mask_stage & (~oh), free_mask_stage)
        preg_alloc_valids.append(v)
        preg_alloc_tags.append(tag)
        preg_alloc_onehots.append(oh)

    preg_alloc_ok = consts.one1
    for slot in range(p.dispatch_w):
        req = disp_valids[slot] & disp_need_pdst[slot]
        preg_alloc_ok = preg_alloc_ok & ((~req) | preg_alloc_valids[slot])

    disp_pdsts = []
    disp_alloc_mask = c(0, width=p.pregs)
    for slot in range(p.dispatch_w):
        req = disp_valids[slot] & disp_need_pdst[slot]
        pdst = req.select(preg_alloc_tags[slot], tag0)
        oh = req.select(preg_alloc_onehots[slot], c(0, width=p.pregs))
        disp_pdsts.append(pdst)
        disp_alloc_mask = disp_alloc_mask | oh

    frontend_ready = can_run & (~commit_redirect) & rob_space_ok & iq_alloc_ok & preg_alloc_ok
    dispatch_fire = frontend_ready & f4_valid

    # Source PTAGs from SMAP with intra-cycle rename forwarding across lanes.
    smap_live = [ren.smap[i].out() for i in range(p.aregs)]
    disp_srcl_tags = []
    disp_srcr_tags = []
    disp_srcp_tags = []
    for slot in range(p.dispatch_w):
        srcl_areg = disp_srcls[slot]
        srcr_areg = disp_srcrs[slot]
        srcp_areg = disp_srcps[slot]

        srcl_tag = mux_by_uindex(m, idx=srcl_areg, items=smap_live, default=tag0)
        srcr_tag = mux_by_uindex(m, idx=srcr_areg, items=smap_live, default=tag0)
        srcp_tag = mux_by_uindex(m, idx=srcp_areg, items=smap_live, default=tag0)
        srcl_tag = srcl_areg.eq(c(REG_INVALID, width=6)).select(tag0, srcl_tag)
        srcr_tag = srcr_areg.eq(c(REG_INVALID, width=6)).select(tag0, srcr_tag)
        srcp_tag = srcp_areg.eq(c(REG_INVALID, width=6)).select(tag0, srcp_tag)

        disp_srcl_tags.append(srcl_tag)
        disp_srcr_tags.append(srcr_tag)
        disp_srcp_tags.append(srcp_tag)

        lane_fire = dispatch_fire & disp_valids[slot]

        # Snapshot old hand regs for push shifting (uses state after previous lanes).
        t0_old = smap_live[24]
        t1_old = smap_live[25]
        t2_old = smap_live[26]
        u0_old = smap_live[28]
        u1_old = smap_live[29]
        u2_old = smap_live[30]

        smap_next = []
        for i in range(p.aregs):
            nxt = smap_live[i]

            if 24 <= i <= 31:
                nxt = (lane_fire & disp_is_start_marker[slot]).select(tag0, nxt)

            if i == 24:
                nxt = (lane_fire & disp_push_t[slot]).select(disp_pdsts[slot], nxt)
            if i == 25:
                nxt = (lane_fire & disp_push_t[slot]).select(t0_old, nxt)
            if i == 26:
                nxt = (lane_fire & disp_push_t[slot]).select(t1_old, nxt)
            if i == 27:
                nxt = (lane_fire & disp_push_t[slot]).select(t2_old, nxt)

            if i == 28:
                nxt = (lane_fire & disp_push_u[slot]).select(disp_pdsts[slot], nxt)
            if i == 29:
                nxt = (lane_fire & disp_push_u[slot]).select(u0_old, nxt)
            if i == 30:
                nxt = (lane_fire & disp_push_u[slot]).select(u1_old, nxt)
            if i == 31:
                nxt = (lane_fire & disp_push_u[slot]).select(u2_old, nxt)

            if i < 24:
                dst_match = disp_regdsts[slot].eq(c(i, width=6))
                nxt = (lane_fire & disp_dst_is_gpr[slot] & dst_match).select(disp_pdsts[slot], nxt)

            if i == 0:
                nxt = tag0

            smap_next.append(nxt)
        smap_live = smap_next

    rename_free_after_dispatch = dispatch_fire.select(ren.free_mask.out() & (~disp_alloc_mask), ren.free_mask.out())
    rename_ready_after_dispatch = dispatch_fire.select(ren.ready_mask.out() & (~disp_alloc_mask), ren.ready_mask.out())

    # Snapshot rename/freelist state for branch/start-marker recovery.
    ckpt_write = consts.zero1
    ckpt_write_idx = c(0, width=ckpt_w)
    for slot in range(p.dispatch_w):
        lane_fire = dispatch_fire & disp_valids[slot]
        is_ckpt = lane_fire & disp_is_start_marker[slot]
        ckpt_idx = disp_checkpoint_ids[slot].trunc(width=ckpt_w)
        ckpt_write = is_ckpt.select(consts.one1, ckpt_write)
        ckpt_write_idx = is_ckpt.select(ckpt_idx, ckpt_write_idx)

    for ci in range(ckpt_entries):
        ciw = c(ci, width=ckpt_w)
        do_ckpt = ckpt_write & ckpt_write_idx.eq(ciw)
        valid_next = ren.ckpt_valid[ci].out()
        valid_next = do_ckpt.select(consts.one1, valid_next)
        ren.ckpt_valid[ci].set(valid_next)
        ren.ckpt_free_mask[ci].set(rename_free_after_dispatch, when=do_ckpt)
        ren.ckpt_ready_mask[ci].set(rename_ready_after_dispatch, when=do_ckpt)
        for r in range(p.aregs):
            ren.ckpt_smap[ci][r].set(smap_live[r], when=do_ckpt)

    flush_ckpt_idx = state.flush_checkpoint_id.out().trunc(width=ckpt_w)
    flush_ckpt_valid = mux_by_uindex(m, idx=flush_ckpt_idx, items=ren.ckpt_valid, default=consts.zero1)
    flush_free_from_ckpt = mux_by_uindex(m, idx=flush_ckpt_idx, items=ren.ckpt_free_mask, default=ren.free_mask.out())
    flush_ready_from_ckpt = mux_by_uindex(m, idx=flush_ckpt_idx, items=ren.ckpt_ready_mask, default=ren.ready_mask.out())
    # Bring-up fallback: recover rename state from committed map on flush.
    # Checkpoint restore remains wired but disabled until checkpoint parity is stable.
    restore_from_ckpt = consts.zero1

    # --- ready table updates ---
    ready_next = ren.ready_mask.out()
    ready_next = dispatch_fire.select(ready_next & (~disp_alloc_mask), ready_next)

    wb_set_mask = c(0, width=p.pregs)
    for slot in range(p.issue_w):
        wb_set_mask = wb_fire_has_dsts[slot].select(wb_set_mask | wb_onehots[slot], wb_set_mask)
    ready_next = ready_next | wb_set_mask
    ready_next = do_flush.select(c((1 << p.pregs) - 1, width=p.pregs), ready_next)
    ready_next = restore_from_ckpt.select(flush_ready_from_ckpt, ready_next)
    ren.ready_mask.set(ready_next)

    # PRF writes (up to issue_w writebacks per cycle).
    for i in range(p.pregs):
        we = consts.zero1
        wdata = consts.zero64
        for slot in range(p.issue_w):
            hit = wb_fire_has_dsts[slot] & wb_pdsts[slot].eq(c(i, width=p.ptag_w))
            we = we | hit
            wdata = hit.select(wb_values[slot], wdata)
        for slot in range(p.commit_w):
            implicit_hit = commit_implicit_setret_fires[slot] & commit_implicit_setret_pdsts[slot].eq(c(i, width=p.ptag_w))
            we = we | implicit_hit
            wdata = implicit_hit.select(commit_implicit_setret_vals[slot], wdata)
        hit_macro = macro_prf_we & macro_prf_tag.eq(c(i, width=p.ptag_w))
        we = we | hit_macro
        wdata = hit_macro.select(macro_prf_data, wdata)
        prf[i].set(wdata, when=we)

    # --- ROB updates ---
    disp_rob_idxs = []
    disp_fires = []
    for slot in range(p.dispatch_w):
        disp_rob_idxs.append(rob.tail.out() + c(slot, width=p.rob_w))
        disp_fires.append(dispatch_fire & disp_valids[slot])

    for i in range(p.rob_depth):
        idx = c(i, width=p.rob_w)
        commit_hit = consts.zero1
        for slot in range(p.commit_w):
            commit_hit = commit_hit | (commit_fires[slot] & commit_idxs[slot].eq(idx))

        disp_hit = consts.zero1
        for slot in range(p.dispatch_w):
            disp_hit = disp_hit | (disp_fires[slot] & disp_rob_idxs[slot].eq(idx))

        wb_hit = consts.zero1
        for slot in range(p.issue_w):
            wb_hit = wb_hit | (wb_fires[slot] & wb_robs[slot].eq(idx))

        v_next = rob.valid[i].out()
        v_next = do_flush.select(consts.zero1, v_next)
        v_next = commit_hit.select(consts.zero1, v_next)
        v_next = disp_hit.select(consts.one1, v_next)
        rob.valid[i].set(v_next)

        done_next = rob.done[i].out()
        done_next = do_flush.select(consts.zero1, done_next)
        done_next = commit_hit.select(consts.zero1, done_next)
        done_next = disp_hit.select(consts.zero1, done_next)
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            is_macro = op_is(disp_ops[slot], OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
            done_next = (hit & is_macro).select(consts.one1, done_next)
        done_next = wb_hit.select(consts.one1, done_next)
        rob.done[i].set(done_next)

        pc_next = rob.pc[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            pc_next = hit.select(disp_pcs[slot], pc_next)
        rob.pc[i].set(pc_next)

        op_next = rob.op[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            op_next = hit.select(disp_ops[slot], op_next)
        rob.op[i].set(op_next)

        ln_next = rob.len_bytes[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            ln_next = hit.select(disp_lens[slot], ln_next)
        rob.len_bytes[i].set(ln_next)

        insn_next = rob.insn_raw[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            insn_next = hit.select(disp_insn_raws[slot], insn_next)
        rob.insn_raw[i].set(insn_next)

        ckpt_next = rob.checkpoint_id[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            ckpt_next = hit.select(disp_checkpoint_ids[slot], ckpt_next)
        rob.checkpoint_id[i].set(ckpt_next)

        dk_next = rob.dst_kind[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            dk_next = hit.select(disp_dst_kind[slot], dk_next)
        rob.dst_kind[i].set(dk_next)

        da_next = rob.dst_areg[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            da_next = hit.select(disp_regdsts[slot], da_next)
        rob.dst_areg[i].set(da_next)

        pd_next = rob.pdst[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            pd_next = hit.select(disp_pdsts[slot], pd_next)
        rob.pdst[i].set(pd_next)

        val_next = rob.value[i].out()
        val_next = disp_hit.select(consts.zero64, val_next)
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            is_macro = op_is(disp_ops[slot], OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
            val_next = (hit & is_macro).select(disp_imms[slot], val_next)
        for slot in range(p.issue_w):
            hit = wb_fires[slot] & wb_robs[slot].eq(idx)
            val_next = hit.select(wb_values[slot], val_next)
        rob.value[i].set(val_next)

        is_store_next = rob.is_store[i].out()
        is_load_next = rob.is_load[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            is_store_next = hit.select(disp_is_store[slot], is_store_next)
            is_load_next = hit.select(consts.zero1, is_load_next)

        st_addr_next = rob.store_addr[i].out()
        st_data_next = rob.store_data[i].out()
        st_size_next = rob.store_size[i].out()
        st_addr_next = disp_hit.select(consts.zero64, st_addr_next)
        st_data_next = disp_hit.select(consts.zero64, st_data_next)
        st_size_next = disp_hit.select(consts.zero4, st_size_next)
        for slot in range(p.issue_w):
            hit = store_fires[slot] & wb_robs[slot].eq(idx)
            st_addr_next = hit.select(exs[slot].addr, st_addr_next)
            st_data_next = hit.select(exs[slot].wdata, st_data_next)
            st_size_next = hit.select(exs[slot].size, st_size_next)
        rob.store_addr[i].set(st_addr_next)
        rob.store_data[i].set(st_data_next)
        rob.store_size[i].set(st_size_next)

        ld_addr_next = rob.load_addr[i].out()
        ld_data_next = rob.load_data[i].out()
        ld_size_next = rob.load_size[i].out()
        ld_addr_next = disp_hit.select(consts.zero64, ld_addr_next)
        ld_data_next = disp_hit.select(consts.zero64, ld_data_next)
        ld_size_next = disp_hit.select(consts.zero4, ld_size_next)
        for slot in range(p.issue_w):
            hit = load_fires[slot] & wb_robs[slot].eq(idx)
            ld_addr_next = hit.select(exs[slot].addr, ld_addr_next)
            ld_data_next = hit.select(wb_values[slot], ld_data_next)
            ld_size_next = hit.select(exs[slot].size, ld_size_next)
            is_load_next = hit.select(consts.one1, is_load_next)
            is_store_next = hit.select(consts.zero1, is_store_next)
        rob.load_addr[i].set(ld_addr_next)
        rob.load_data[i].set(ld_data_next)
        rob.load_size[i].set(ld_size_next)
        rob.is_load[i].set(is_load_next)
        rob.is_store[i].set(is_store_next)

        mb_next = rob.macro_begin[i].out()
        me_next = rob.macro_end[i].out()
        for slot in range(p.dispatch_w):
            hit = disp_fires[slot] & disp_rob_idxs[slot].eq(idx)
            mb_next = hit.select(disp_srcls[slot], mb_next)
            me_next = hit.select(disp_srcrs[slot], me_next)
        rob.macro_begin[i].set(mb_next)
        rob.macro_end[i].set(me_next)

    # ROB pointers/count.
    head_next = rob.head.out()
    tail_next = rob.tail.out()
    count_next = rob.count.out()

    head_next = do_flush.select(c(0, width=p.rob_w), head_next)
    tail_next = do_flush.select(c(0, width=p.rob_w), tail_next)
    count_next = do_flush.select(c(0, width=p.rob_w + 1), count_next)

    inc_head = commit_fire & (~do_flush)
    inc_tail = dispatch_fire & (~do_flush)

    head_inc = commit_count
    if p.rob_w > head_inc.width:
        head_inc = head_inc.zext(width=p.rob_w)
    elif p.rob_w < head_inc.width:
        head_inc = head_inc.trunc(width=p.rob_w)
    head_next = inc_head.select(rob.head.out() + head_inc, head_next)
    disp_tail_inc = disp_count
    if p.rob_w > disp_tail_inc.width:
        disp_tail_inc = disp_tail_inc.zext(width=p.rob_w)
    elif p.rob_w < disp_tail_inc.width:
        disp_tail_inc = disp_tail_inc.trunc(width=p.rob_w)
    tail_next = inc_tail.select(rob.tail.out() + disp_tail_inc, tail_next)

    commit_dec = commit_count.zext(width=p.rob_w + 1)
    commit_dec_neg = (~commit_dec) + c(1, width=p.rob_w + 1)
    count_next = inc_tail.select(count_next + disp_count.zext(width=p.rob_w + 1), count_next)
    count_next = inc_head.select(count_next + commit_dec_neg, count_next)

    rob.head.set(head_next)
    rob.tail.set(tail_next)
    rob.count.set(count_next)

    # --- IQ updates ---
    def update_iq(*, iq, disp_to: list, alloc_idxs: list, issue_fires_q: list, issue_idxs_q: list) -> None:
        for i in range(p.iq_depth):
            idx = c(i, width=p.iq_w)

            issue_clear = consts.zero1
            for slot in range(len(issue_fires_q)):
                issue_clear = issue_clear | (issue_fires_q[slot] & issue_idxs_q[slot].eq(idx))

            disp_alloc_hit = consts.zero1
            for slot in range(p.dispatch_w):
                disp_alloc_hit = disp_alloc_hit | (disp_fires[slot] & disp_to[slot] & alloc_idxs[slot].eq(idx))

            v_next = iq.valid[i].out()
            v_next = do_flush.select(consts.zero1, v_next)
            v_next = issue_clear.select(consts.zero1, v_next)
            v_next = disp_alloc_hit.select(consts.one1, v_next)
            iq.valid[i].set(v_next)

            robn = iq.rob[i].out()
            opn = iq.op[i].out()
            pcn = iq.pc[i].out()
            imn = iq.imm[i].out()
            sln = iq.srcl[i].out()
            srn = iq.srcr[i].out()
            stn = iq.srcr_type[i].out()
            shn = iq.shamt[i].out()
            spn = iq.srcp[i].out()
            pdn = iq.pdst[i].out()
            hdn = iq.has_dst[i].out()
            for slot in range(p.dispatch_w):
                hit = disp_fires[slot] & disp_to[slot] & alloc_idxs[slot].eq(idx)
                robn = hit.select(disp_rob_idxs[slot], robn)
                opn = hit.select(disp_ops[slot], opn)
                pcn = hit.select(disp_pcs[slot], pcn)
                imn = hit.select(disp_imms[slot], imn)
                sln = hit.select(disp_srcl_tags[slot], sln)
                srn = hit.select(disp_srcr_tags[slot], srn)
                stn = hit.select(disp_srcr_types[slot], stn)
                shn = hit.select(disp_shamts[slot], shn)
                spn = hit.select(disp_srcp_tags[slot], spn)
                pdn = hit.select(disp_pdsts[slot], pdn)
                hdn = hit.select(disp_need_pdst[slot], hdn)
            iq.rob[i].set(robn)
            iq.op[i].set(opn)
            iq.pc[i].set(pcn)
            iq.imm[i].set(imn)
            iq.srcl[i].set(sln)
            iq.srcr[i].set(srn)
            iq.srcr_type[i].set(stn)
            iq.shamt[i].set(shn)
            iq.srcp[i].set(spn)
            iq.pdst[i].set(pdn)
            iq.has_dst[i].set(hdn)

    lsu_base = 0
    bru_base = p.lsu_w
    alu_base = p.lsu_w + p.bru_w
    update_iq(
        iq=iq_lsu,
        disp_to=disp_to_lsu,
        alloc_idxs=lsu_alloc_idxs,
        issue_fires_q=issue_fires_eff[lsu_base : lsu_base + p.lsu_w],
        issue_idxs_q=lsu_issue_idxs,
    )
    update_iq(
        iq=iq_bru,
        disp_to=disp_to_bru,
        alloc_idxs=bru_alloc_idxs,
        issue_fires_q=issue_fires_eff[bru_base : bru_base + p.bru_w],
        issue_idxs_q=bru_issue_idxs,
    )
    update_iq(
        iq=iq_alu,
        disp_to=disp_to_alu,
        alloc_idxs=alu_alloc_idxs,
        issue_fires_q=issue_fires_eff[alu_base : alu_base + p.alu_w],
        issue_idxs_q=alu_issue_idxs,
    )

    # --- SMAP updates (rename) ---
    for i in range(p.aregs):
        nxt = smap_live[i]
        nxt = do_flush.select(ren.cmap[i].out(), nxt)
        ckpt_smap_i = mux_by_uindex(
            m,
            idx=flush_ckpt_idx,
            items=[ren.ckpt_smap[ci][i] for ci in range(ckpt_entries)],
            default=ren.cmap[i].out(),
        )
        nxt = restore_from_ckpt.select(ckpt_smap_i, nxt)
        if i == 0:
            nxt = tag0
        ren.smap[i].set(nxt)

    # --- CMAP + freelist updates (commit) ---
    #
    # Apply up to `commit_w` in-order updates, including the T/U hand-stack and
    # their corresponding freelist frees.
    cmap_live = [ren.cmap[i].out() for i in range(p.aregs)]

    free_live = rename_free_after_dispatch

    for slot in range(p.commit_w):
        fire = commit_fires[slot]
        op = rob_ops[slot]
        dk = rob_dst_kinds[slot]
        areg = rob_dst_aregs[slot]
        pdst = rob_pdsts[slot]

        is_macro = op_is(op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
        is_start_marker = (
            op_is(
                op,
                OP_C_BSTART_STD,
                OP_C_BSTART_COND,
                OP_C_BSTART_DIRECT,
                OP_BSTART_STD_FALL,
                OP_BSTART_STD_DIRECT,
                OP_BSTART_STD_COND,
                OP_BSTART_STD_CALL,
            )
            | is_macro
        )

        # Snapshot old hand regs (from the pre-update state for this slot).
        old_t0 = cmap_live[24]
        old_t1 = cmap_live[25]
        old_t2 = cmap_live[26]
        old_t3 = cmap_live[27]
        old_u0 = cmap_live[28]
        old_u1 = cmap_live[29]
        old_u2 = cmap_live[30]
        old_u3 = cmap_live[31]

        # Start marker clears: free all old hand tags and clear mappings.
        if_free = commit_enter_new_blocks[slot]
        for old in [old_t0, old_t1, old_t2, old_t3, old_u0, old_u1, old_u2, old_u3]:
            oh = onehot_from_tag(m, tag=old, width=p.pregs, tag_width=p.ptag_w)
            free_live = (if_free & (~old.eq(tag0))).select(free_live | oh, free_live)
        for i in range(24, 32):
            cmap_live[i] = if_free.select(tag0, cmap_live[i])

        # Push T: free dropped T3 and shift [T0..T3].
        push_t = fire & dk.eq(c(2, width=2))
        t3_oh = onehot_from_tag(m, tag=old_t3, width=p.pregs, tag_width=p.ptag_w)
        free_live = (push_t & (~old_t3.eq(tag0))).select(free_live | t3_oh, free_live)
        cmap_live[24] = push_t.select(pdst, cmap_live[24])
        cmap_live[25] = push_t.select(old_t0, cmap_live[25])
        cmap_live[26] = push_t.select(old_t1, cmap_live[26])
        cmap_live[27] = push_t.select(old_t2, cmap_live[27])

        # Push U: free dropped U3 and shift [U0..U3].
        push_u = fire & dk.eq(c(3, width=2))
        u3_oh = onehot_from_tag(m, tag=old_u3, width=p.pregs, tag_width=p.ptag_w)
        free_live = (push_u & (~old_u3.eq(tag0))).select(free_live | u3_oh, free_live)
        cmap_live[28] = push_u.select(pdst, cmap_live[28])
        cmap_live[29] = push_u.select(old_u0, cmap_live[29])
        cmap_live[30] = push_u.select(old_u1, cmap_live[30])
        cmap_live[31] = push_u.select(old_u2, cmap_live[31])

        # Normal GPR writes: update mapping and free old dst tag (gpr only, not x0).
        is_gpr = fire & dk.eq(c(1, width=2))
        for i in range(24):
            hit = is_gpr & areg.eq(c(i, width=6))
            old = cmap_live[i]
            old_oh = onehot_from_tag(m, tag=old, width=p.pregs, tag_width=p.ptag_w)
            free_live = (hit & (~old.eq(tag0))).select(free_live | old_oh, free_live)
            cmap_live[i] = hit.select(pdst, cmap_live[i])

        # r0 hardwired to p0.
        cmap_live[0] = tag0

    for i in range(p.aregs):
        ren.cmap[i].set(cmap_live[i])

    # Flush recomputes freelist from CMAP to drop speculative allocations.
    used = c(0, width=p.pregs)
    for i in range(p.aregs):
        used = used | onehot_from_tag(m, tag=ren.cmap[i].out(), width=p.pregs, tag_width=p.ptag_w)
    free_recomputed = ~used
    free_next = do_flush.select(free_recomputed, free_live)
    free_next = restore_from_ckpt.select(flush_free_from_ckpt, free_next)
    ren.free_mask.set(free_next)

    # --- commit state updates (pc/br/control regs) ---
    state.pc.set(pc_live)

    # Track latest frontend PC for debug/tracing.
    fpc_next = state.fpc.out()
    fpc_next = f4_valid.select(f4_pc, fpc_next)
    fpc_next = commit_redirect.select(redirect_pc, fpc_next)
    fpc_next = do_flush.select(state.flush_pc.out(), fpc_next)
    state.fpc.set(fpc_next)

    # Redirect/flush pending.
    state.flush_pc.set(commit_redirect.select(redirect_pc, state.flush_pc.out()))
    state.flush_checkpoint_id.set(
        commit_redirect.select(redirect_checkpoint_id, state.flush_checkpoint_id.out())
    )
    flush_pend_next = state.flush_pending.out()
    flush_pend_next = do_flush.select(consts.zero1, flush_pend_next)
    flush_pend_next = commit_redirect.select(consts.one1, flush_pend_next)
    state.flush_pending.set(flush_pend_next)

    replay_pending_next = state.replay_pending.out()
    replay_pending_next = do_flush.select(consts.zero1, replay_pending_next)
    replay_pending_next = replay_redirect_fire.select(consts.zero1, replay_pending_next)
    replay_pending_next = replay_set.select(consts.one1, replay_pending_next)
    state.replay_pending.set(replay_pending_next)
    state.replay_store_rob.set(replay_set.select(replay_set_store_rob, state.replay_store_rob.out()))
    state.replay_pc.set(replay_set.select(replay_set_pc, state.replay_pc.out()))

    # Halt latch.
    halt_set = consts.zero1
    for slot in range(p.commit_w):
        op = rob_ops[slot]
        is_halt = op.eq(c(OP_EBREAK, width=12)) | op.eq(c(OP_INVALID, width=12))
        halt_set = halt_set | (commit_fires[slot] & is_halt)
    halt_set = halt_set | mmio_exit
    state.halted.set(consts.one1, when=halt_set)

    commit_cond_live = macro_setc_tgt_fire.select(consts.one1, commit_cond_live)
    commit_tgt_live = macro_setc_tgt_fire.select(macro_setc_tgt_data, commit_tgt_live)
    state.cycles.set(state.cycles.out() + consts.one64)
    state.commit_cond.set(commit_cond_live)
    state.commit_tgt.set(commit_tgt_live)
    state.br_kind.set(br_kind_live)
    state.br_base_pc.set(br_base_live)
    state.br_off.set(br_off_live)

    # --- template macro engine state updates ---
    #
    # Implements the bring-up ABI semantics used by QEMU/LLVM:
    # - FENTRY: SP_SUB, then STORE loop.
    # - FEXIT: SP_ADD, then LOAD loop.
    # - FRET.STK: SP_ADD, LOAD ra, SETC.TGT ra, then remaining LOAD loop.
    # - FRET.RA: SETC.TGT ra, SP_ADD, then LOAD loop.
    ph_init = c(0, width=2)
    ph_mem = c(1, width=2)
    ph_sp = c(2, width=2)
    ph_setc = c(3, width=2)

    macro_active_n = macro_active
    macro_phase_n = macro_phase
    macro_op_n = macro_op
    macro_begin_n = state.macro_begin.out()
    macro_end_n = state.macro_end.out()
    macro_stack_n = macro_stacksize
    macro_reg_n = macro_reg
    macro_i_n = macro_i
    macro_sp_base_n = macro_sp_base

    macro_active_n = do_flush.select(consts.zero1, macro_active_n)
    macro_phase_n = do_flush.select(ph_init, macro_phase_n)

    macro_active_n = macro_start.select(consts.one1, macro_active_n)
    macro_phase_n = macro_start.select(ph_init, macro_phase_n)
    macro_op_n = macro_start.select(head_op, macro_op_n)
    macro_begin_n = macro_start.select(head_macro_begin, macro_begin_n)
    macro_end_n = macro_start.select(head_macro_end, macro_end_n)
    macro_stack_n = macro_start.select(head_value, macro_stack_n)
    macro_reg_n = macro_start.select(head_macro_begin, macro_reg_n)
    macro_i_n = macro_start.select(c(0, width=6), macro_i_n)

    macro_phase_is_init = macro_phase_init
    macro_phase_is_mem = macro_phase_mem
    macro_phase_is_sp = macro_phase_sp
    macro_phase_is_setc = macro_phase_setc

    # Init: latch base SP and setup iteration.
    init_fire = macro_active & macro_phase_is_init
    sp_new_init = macro_sp_val - macro_frame_adj
    sp_new_restore = macro_sp_val + macro_frame_adj
    macro_sp_base_n = (init_fire & macro_is_fentry).select(sp_new_init, macro_sp_base_n)
    macro_sp_base_n = (init_fire & (macro_is_fexit | macro_is_fret_stk)).select(sp_new_restore, macro_sp_base_n)
    macro_reg_n = init_fire.select(macro_begin, macro_reg_n)
    macro_i_n = init_fire.select(c(0, width=6), macro_i_n)
    macro_phase_n = (init_fire & (macro_is_fentry | macro_is_fexit | macro_is_fret_stk)).select(ph_mem, macro_phase_n)
    macro_phase_n = (init_fire & macro_is_fret_ra).select(ph_sp, macro_phase_n)

    # Mem loop: iterate regs and offsets; save uses store port, restore uses load port.
    step_fire = ctu["loop_fire"]
    step_done = ctu["loop_done"]
    reg_next = ctu["loop_reg_next"]
    i_next = ctu["loop_i_next"]
    macro_reg_n = (step_fire & (~step_done)).select(reg_next, macro_reg_n)
    macro_i_n = (step_fire & (~step_done)).select(i_next, macro_i_n)

    # FRET.STK requires a SETC.TGT immediately after restoring RA.
    step_ra_restore = step_fire & macro_is_fret_stk & macro_uop_is_load & macro_uop_reg.eq(c(10, width=6))
    macro_phase_n = (step_ra_restore & (~step_done)).select(ph_setc, macro_phase_n)

    done_macro = step_done & (macro_is_fentry | macro_is_fexit | macro_is_fret_stk | macro_is_fret_ra)
    macro_active_n = done_macro.select(consts.zero1, macro_active_n)
    macro_phase_n = done_macro.select(ph_init, macro_phase_n)

    # FRET.RA has an explicit SP_ADD phase before restore loads.
    sp_fire = macro_active & macro_phase_is_sp & macro_is_fret_ra
    macro_sp_base_n = sp_fire.select(sp_new_restore, macro_sp_base_n)
    macro_phase_n = sp_fire.select(ph_mem, macro_phase_n)

    # FRET.STK emits SETC.TGT as a standalone template uop between RA load
    # and the remaining restore-load loop.
    setc_fire = macro_active & macro_phase_is_setc & macro_is_fret_stk
    macro_phase_n = setc_fire.select(ph_mem, macro_phase_n)

    macro_wait_n = state.macro_wait_commit.out()
    macro_wait_n = do_flush.select(consts.zero1, macro_wait_n)
    macro_wait_n = macro_start.select(consts.one1, macro_wait_n)
    macro_committed = consts.zero1
    for slot in range(p.commit_w):
        op = rob_ops[slot]
        fire = commit_fires[slot]
        macro_committed = macro_committed | (fire & op_is(op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK))
    macro_wait_n = macro_committed.select(consts.zero1, macro_wait_n)

    # Suppress one synthetic C.BSTART boundary-dup right after a macro
    # commit handoff (macro commit advances to a new PC).
    macro_handoff = consts.zero1
    for slot in range(p.commit_w):
        op = rob_ops[slot]
        fire = commit_fires[slot]
        is_macro_op = op_is(op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
        macro_handoff = macro_handoff | (fire & is_macro_op & (~commit_next_pcs[slot].eq(commit_pcs[slot])))
    any_commit_fire = consts.zero1
    for slot in range(p.commit_w):
        any_commit_fire = any_commit_fire | commit_fires[slot]
    post_macro_handoff_n = state.post_macro_handoff.out()
    post_macro_handoff_n = do_flush.select(consts.zero1, post_macro_handoff_n)
    post_macro_handoff_n = macro_handoff.select(consts.one1, post_macro_handoff_n)
    post_macro_handoff_n = (any_commit_fire & (~macro_handoff)).select(consts.zero1, post_macro_handoff_n)

    state.macro_active.set(macro_active_n)
    state.macro_wait_commit.set(macro_wait_n)
    state.post_macro_handoff.set(post_macro_handoff_n)
    state.macro_phase.set(macro_phase_n)
    state.macro_op.set(macro_op_n)
    state.macro_begin.set(macro_begin_n)
    state.macro_end.set(macro_end_n)
    state.macro_stacksize.set(macro_stack_n)
    state.macro_reg.set(macro_reg_n)
    state.macro_i.set(macro_i_n)
    state.macro_sp_base.set(macro_sp_base_n)
    macro_saved_ra_n = state.macro_saved_ra.out()
    save_ra_fire = macro_store_fire & macro_uop_reg.eq(c(10, width=6))
    restore_ra_fire = macro_reg_write & macro_uop_reg.eq(c(10, width=6)) & macro_is_fret_stk
    macro_saved_ra_n = save_ra_fire.select(macro_store_data, macro_saved_ra_n)
    macro_saved_ra_n = restore_ra_fire.select(macro_load_data_eff, macro_saved_ra_n)
    state.macro_saved_ra.set(macro_saved_ra_n)

    # --- outputs ---
    a0_tag = ren.cmap[2].out()
    a1_tag = ren.cmap[3].out()
    ra_tag = ren.cmap[10].out()
    sp_tag = ren.cmap[1].out()

    m.output("halted", state.halted)
    m.output("cycles", state.cycles)
    m.output("pc", state.pc)
    m.output("fpc", state.fpc)
    m.output("a0", mux_by_uindex(m, idx=a0_tag, items=prf, default=consts.zero64))
    m.output("a1", mux_by_uindex(m, idx=a1_tag, items=prf, default=consts.zero64))
    m.output("ra", mux_by_uindex(m, idx=ra_tag, items=prf, default=consts.zero64))
    m.output("sp", mux_by_uindex(m, idx=sp_tag, items=prf, default=consts.zero64))
    m.output("commit_op", head_op)
    m.output("commit_fire", commit_fire)
    m.output("commit_value", head_value)
    m.output("commit_dst_kind", head_dst_kind)
    m.output("commit_dst_areg", head_dst_areg)
    m.output("commit_pdst", head_pdst)
    m.output("commit_cond", state.commit_cond)
    m.output("commit_tgt", state.commit_tgt)
    m.output("br_kind", state.br_kind)
    m.output("br_base_pc", state.br_base_pc)
    m.output("br_off", state.br_off)
    m.output("commit_store_fire", commit_store_fire)
    m.output("commit_store_addr", commit_store_addr)
    m.output("commit_store_data", commit_store_data)
    m.output("commit_store_size", commit_store_size)
    m.output("rob_head_valid", rob_valids[0])
    m.output("rob_head_done", rob_dones[0])
    m.output("rob_head_pc", mux_by_uindex(m, idx=rob.head.out(), items=rob.pc, default=consts.zero64))
    m.output("rob_head_insn_raw", head_insn_raw)
    m.output("rob_head_len", head_len)
    m.output("rob_head_op", head_op)

    # Commit slot visibility (bring-up tracing): per-slot PC/op/value/fire.
    #
    # Template blocks (FENTRY/FEXIT/FRET.*) are traced as restartable
    # per-step retire events to match QEMU commit semantics exactly.
    # The internal macro ROB commit is hidden from the trace stream.
    macro_trace_fire = macro_uop_valid
    macro_adj_nonzero = ~macro_frame_adj.eq(consts.zero64)
    macro_trace_pc = state.pc.out()
    macro_trace_seq_pc = macro_trace_pc + head_len.zext(width=64)
    macro_trace_op = macro_op
    macro_trace_val = head_value
    macro_trace_rob = rob.head.out()
    macro_trace_len = head_len
    macro_trace_insn = head_insn_raw

    macro_trace_wb_load = macro_reg_write
    macro_trace_wb_sp_sub = macro_uop_is_sp_sub & macro_adj_nonzero
    macro_trace_wb_sp_add = macro_uop_is_sp_add & macro_adj_nonzero
    macro_trace_wb_valid = macro_trace_fire & (macro_trace_wb_load | macro_trace_wb_sp_sub | macro_trace_wb_sp_add)
    macro_trace_wb_rd = c(0, width=6)
    macro_trace_wb_rd = macro_trace_wb_load.select(macro_uop_reg, macro_trace_wb_rd)
    macro_trace_wb_rd = (macro_trace_wb_sp_sub | macro_trace_wb_sp_add).select(c(1, width=6), macro_trace_wb_rd)
    macro_trace_wb_data = consts.zero64
    macro_trace_wb_data = macro_trace_wb_load.select(macro_load_data_eff, macro_trace_wb_data)
    macro_trace_wb_data = macro_trace_wb_sp_add.select(macro_sp_val + macro_frame_adj, macro_trace_wb_data)
    macro_trace_wb_data = macro_trace_wb_sp_sub.select(macro_sp_val - macro_frame_adj, macro_trace_wb_data)

    macro_trace_mem_store = macro_store_fire
    macro_trace_mem_load = macro_uop_is_load & macro_reg_is_gpr & macro_reg_not_zero
    macro_trace_mem_valid = macro_trace_fire & (macro_trace_mem_store | macro_trace_mem_load)
    macro_trace_mem_is_store = macro_trace_fire & macro_trace_mem_store
    macro_trace_mem_addr = macro_uop_addr
    macro_trace_mem_wdata = macro_trace_mem_store.select(macro_store_data, consts.zero64)
    macro_trace_mem_rdata = macro_trace_mem_load.select(macro_load_data_eff, consts.zero64)
    macro_trace_mem_size = macro_trace_mem_valid.select(c(8, width=4), consts.zero4)

    macro_trace_is_fentry = macro_op.eq(c(OP_FENTRY, width=12))
    macro_trace_is_fexit = macro_op.eq(c(OP_FEXIT, width=12))
    macro_trace_is_fret = op_is(macro_op, OP_FRET_RA, OP_FRET_STK)
    macro_trace_done_fentry = (macro_uop_is_sp_sub & macro_stacksize.eq(consts.zero64)) | (macro_uop_is_store & step_done)
    macro_trace_done_fexit = macro_uop_is_load & step_done & macro_trace_is_fexit
    macro_trace_done_fret = macro_uop_is_load & step_done & macro_trace_is_fret
    macro_trace_next_pc = macro_trace_pc
    macro_trace_next_pc = macro_trace_done_fentry.select(macro_trace_seq_pc, macro_trace_next_pc)
    macro_trace_next_pc = macro_trace_done_fexit.select(macro_trace_seq_pc, macro_trace_next_pc)
    macro_trace_next_pc = macro_trace_done_fret.select(commit_tgt_live, macro_trace_next_pc)
    macro_shadow_fire = macro_trace_fire & macro_trace_is_fret & macro_uop_is_load & state.macro_i.out().eq(c(0, width=6))

    # Keep retire trace strictly instruction-driven; qemu-specific boundary-only
    # metadata commits are filtered in the lockstep runner.
    shadow_boundary_fire = consts.zero1
    shadow_boundary_fire1 = consts.zero1

    # `commit_fire` / `commit_op` / `commit_value` remain lane0-compatible.
    # These additional signals help debug multi-commit cycles where older
    # commits may not appear in a slot0-only log.
    max_commit_slots = 4
    for slot in range(max_commit_slots):
        fire = consts.zero1
        pc = consts.zero64
        rob_idx = c(0, width=p.rob_w)
        op = c(0, width=12)
        val = consts.zero64
        ln = consts.zero3
        insn_raw = consts.zero64
        wb_valid = consts.zero1
        wb_rd = c(0, width=6)
        wb_data = consts.zero64
        mem_valid = consts.zero1
        mem_is_store = consts.zero1
        mem_addr = consts.zero64
        mem_wdata = consts.zero64
        mem_rdata = consts.zero64
        mem_size = consts.zero4
        trap_valid = consts.zero1
        trap_cause = c(0, width=32)
        next_pc = consts.zero64
        checkpoint_id = c(0, width=6)
        if slot < p.commit_w:
            fire_raw = commit_fires[slot]
            pc = rob_pcs[slot]
            rob_idx = commit_idxs[slot]
            op = rob_ops[slot]
            val = rob_values[slot]
            ln = rob_lens[slot]
            insn_raw = rob_insn_raws[slot]
            is_macro_commit = op_is(op, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
            fire = fire_raw & (~is_macro_commit)
            is_gpr_dst = rob_dst_kinds[slot].eq(c(1, width=2))
            wb_trace_suppress = op_is(
                op,
                OP_C_BSTART_STD,
                OP_C_BSTART_COND,
                OP_C_BSTART_DIRECT,
                OP_BSTART_STD_FALL,
                OP_BSTART_STD_DIRECT,
                OP_BSTART_STD_COND,
                OP_BSTART_STD_CALL,
                OP_C_BSTOP,
            )
            rd = rob_dst_aregs[slot]
            wb_valid = fire & is_gpr_dst & (~rd.eq(c(0, width=6))) & (~wb_trace_suppress)
            wb_rd = rd
            wb_data = rob_values[slot]
            next_pc_slot = commit_next_pcs[slot]
            implicit_setret_slot = commit_implicit_setret_fires[slot]
            implicit_setret_areg = commit_implicit_setret_aregs[slot]
            implicit_setret_data = commit_implicit_setret_vals[slot]
            is_store = rob_is_stores[slot]
            is_load = rob_is_loads[slot]
            ld_trace_data = rob_ld_datas[slot]
            for i in range(p.sq_entries):
                st_hit = stbuf_valid[i].out() & stbuf_addr[i].out().eq(rob_ld_addrs[slot])
                ld_trace_data = st_hit.select(stbuf_data[i].out(), ld_trace_data)
            mem_valid = fire & (is_store | is_load)
            mem_is_store = fire & is_store
            mem_addr = is_store.select(rob_st_addrs[slot], rob_ld_addrs[slot])
            mem_wdata = is_store.select(rob_st_datas[slot], consts.zero64)
            mem_rdata = is_load.select(ld_trace_data, consts.zero64)
            mem_size = is_store.select(rob_st_sizes[slot], rob_ld_sizes[slot])
            wb_valid = implicit_setret_slot.select(consts.one1, wb_valid)
            wb_rd = implicit_setret_slot.select(implicit_setret_areg, wb_rd)
            wb_data = implicit_setret_slot.select(implicit_setret_data, wb_data)
            next_pc = next_pc_slot
            checkpoint_id = rob_checkpoint_ids[slot]

        # When `shadow_boundary_fire` is active, shift real retire records up
        # by one slot so slot0 can carry the synthetic boundary marker event.
        if slot > 0 and (slot - 1) < p.commit_w:
            prev = slot - 1
            fire_prev_raw = commit_fires[prev]
            pc_prev = commit_pcs[prev]
            rob_prev = commit_idxs[prev]
            op_prev = rob_ops[prev]
            val_prev = rob_values[prev]
            ln_prev = rob_lens[prev]
            insn_prev = rob_insn_raws[prev]
            is_macro_prev = op_is(op_prev, OP_FENTRY, OP_FEXIT, OP_FRET_RA, OP_FRET_STK)
            fire_prev = fire_prev_raw & (~is_macro_prev)
            is_gpr_prev = rob_dst_kinds[prev].eq(c(1, width=2))
            wb_suppress_prev = op_is(
                op_prev,
                OP_C_BSTART_STD,
                OP_C_BSTART_COND,
                OP_C_BSTART_DIRECT,
                OP_BSTART_STD_FALL,
                OP_BSTART_STD_DIRECT,
                OP_BSTART_STD_COND,
                OP_BSTART_STD_CALL,
                OP_C_BSTOP,
            )
            rd_prev = rob_dst_aregs[prev]
            wb_valid_prev = fire_prev & is_gpr_prev & (~rd_prev.eq(c(0, width=6))) & (~wb_suppress_prev)
            wb_rd_prev = rd_prev
            wb_data_prev = rob_values[prev]
            next_pc_prev = commit_next_pcs[prev]
            is_store_prev = rob_is_stores[prev]
            is_load_prev = rob_is_loads[prev]
            ld_trace_prev = rob_ld_datas[prev]
            for i in range(p.sq_entries):
                st_hit_prev = stbuf_valid[i].out() & stbuf_addr[i].out().eq(rob_ld_addrs[prev])
                ld_trace_prev = st_hit_prev.select(stbuf_data[i].out(), ld_trace_prev)
            mem_valid_prev = fire_prev & (is_store_prev | is_load_prev)
            mem_is_store_prev = fire_prev & is_store_prev
            mem_addr_prev = is_store_prev.select(rob_st_addrs[prev], rob_ld_addrs[prev])
            mem_wdata_prev = is_store_prev.select(rob_st_datas[prev], consts.zero64)
            mem_rdata_prev = is_load_prev.select(ld_trace_prev, consts.zero64)
            mem_size_prev = is_store_prev.select(rob_st_sizes[prev], rob_ld_sizes[prev])
            checkpoint_prev = rob_checkpoint_ids[prev]
            shift_active = shadow_boundary_fire
            if slot > 1:
                shift_active = shift_active | shadow_boundary_fire1

            fire = shift_active.select(fire_prev, fire)
            pc = shift_active.select(pc_prev, pc)
            rob_idx = shift_active.select(rob_prev, rob_idx)
            op = shift_active.select(op_prev, op)
            val = shift_active.select(val_prev, val)
            ln = shift_active.select(ln_prev, ln)
            insn_raw = shift_active.select(insn_prev, insn_raw)
            wb_valid = shift_active.select(wb_valid_prev, wb_valid)
            wb_rd = shift_active.select(wb_rd_prev, wb_rd)
            wb_data = shift_active.select(wb_data_prev, wb_data)
            mem_valid = shift_active.select(mem_valid_prev, mem_valid)
            mem_is_store = shift_active.select(mem_is_store_prev, mem_is_store)
            mem_addr = shift_active.select(mem_addr_prev, mem_addr)
            mem_wdata = shift_active.select(mem_wdata_prev, mem_wdata)
            mem_rdata = shift_active.select(mem_rdata_prev, mem_rdata)
            mem_size = shift_active.select(mem_size_prev, mem_size)
            next_pc = shift_active.select(next_pc_prev, next_pc)
            checkpoint_id = shift_active.select(checkpoint_prev, checkpoint_id)

        if slot == 0:
            fire = shadow_boundary_fire.select(consts.one1, fire)
            pc = shadow_boundary_fire.select(commit_pcs[0], pc)
            rob_idx = shadow_boundary_fire.select(commit_idxs[0], rob_idx)
            op = shadow_boundary_fire.select(rob_ops[0], op)
            val = shadow_boundary_fire.select(rob_values[0], val)
            ln = shadow_boundary_fire.select(rob_lens[0], ln)
            insn_raw = shadow_boundary_fire.select(rob_insn_raws[0], insn_raw)
            wb_valid = shadow_boundary_fire.select(consts.zero1, wb_valid)
            wb_rd = shadow_boundary_fire.select(c(0, width=6), wb_rd)
            wb_data = shadow_boundary_fire.select(consts.zero64, wb_data)
            mem_valid = shadow_boundary_fire.select(consts.zero1, mem_valid)
            mem_is_store = shadow_boundary_fire.select(consts.zero1, mem_is_store)
            mem_addr = shadow_boundary_fire.select(consts.zero64, mem_addr)
            mem_wdata = shadow_boundary_fire.select(consts.zero64, mem_wdata)
            mem_rdata = shadow_boundary_fire.select(consts.zero64, mem_rdata)
            mem_size = shadow_boundary_fire.select(consts.zero4, mem_size)
            next_pc = shadow_boundary_fire.select(commit_pcs[0], next_pc)
            checkpoint_id = shadow_boundary_fire.select(rob_checkpoint_ids[0], checkpoint_id)

            fire = macro_trace_fire.select(consts.one1, fire)
            pc = macro_trace_fire.select(macro_trace_pc, pc)
            rob_idx = macro_trace_fire.select(macro_trace_rob, rob_idx)
            op = macro_trace_fire.select(macro_trace_op, op)
            val = macro_trace_fire.select(macro_trace_val, val)
            ln = macro_trace_fire.select(macro_trace_len, ln)
            insn_raw = macro_trace_fire.select(macro_trace_insn, insn_raw)
            wb_valid = macro_trace_fire.select(macro_trace_wb_valid, wb_valid)
            wb_rd = macro_trace_fire.select(macro_trace_wb_rd, wb_rd)
            wb_data = macro_trace_fire.select(macro_trace_wb_data, wb_data)
            mem_valid = macro_trace_fire.select(macro_trace_mem_valid, mem_valid)
            mem_is_store = macro_trace_fire.select(macro_trace_mem_is_store, mem_is_store)
            mem_addr = macro_trace_fire.select(macro_trace_mem_addr, mem_addr)
            mem_wdata = macro_trace_fire.select(macro_trace_mem_wdata, mem_wdata)
            mem_rdata = macro_trace_fire.select(macro_trace_mem_rdata, mem_rdata)
            mem_size = macro_trace_fire.select(macro_trace_mem_size, mem_size)
            next_pc = macro_trace_fire.select(macro_trace_next_pc, next_pc)
            wb_valid = macro_shadow_fire.select(consts.zero1, wb_valid)
            wb_rd = macro_shadow_fire.select(c(0, width=6), wb_rd)
            wb_data = macro_shadow_fire.select(consts.zero64, wb_data)
            mem_valid = macro_shadow_fire.select(consts.zero1, mem_valid)
            mem_is_store = macro_shadow_fire.select(consts.zero1, mem_is_store)
            mem_addr = macro_shadow_fire.select(consts.zero64, mem_addr)
            mem_wdata = macro_shadow_fire.select(consts.zero64, mem_wdata)
            mem_rdata = macro_shadow_fire.select(consts.zero64, mem_rdata)
            mem_size = macro_shadow_fire.select(consts.zero4, mem_size)
            next_pc = macro_shadow_fire.select(macro_trace_pc, next_pc)
        else:
            if slot == 1 and p.commit_w > 1:
                fire = shadow_boundary_fire1.select(consts.one1, fire)
                pc = shadow_boundary_fire1.select(commit_pcs[1], pc)
                rob_idx = shadow_boundary_fire1.select(commit_idxs[1], rob_idx)
                op = shadow_boundary_fire1.select(rob_ops[1], op)
                val = shadow_boundary_fire1.select(rob_values[1], val)
                ln = shadow_boundary_fire1.select(rob_lens[1], ln)
                insn_raw = shadow_boundary_fire1.select(rob_insn_raws[1], insn_raw)
                wb_valid = shadow_boundary_fire1.select(consts.zero1, wb_valid)
                wb_rd = shadow_boundary_fire1.select(c(0, width=6), wb_rd)
                wb_data = shadow_boundary_fire1.select(consts.zero64, wb_data)
                mem_valid = shadow_boundary_fire1.select(consts.zero1, mem_valid)
                mem_is_store = shadow_boundary_fire1.select(consts.zero1, mem_is_store)
                mem_addr = shadow_boundary_fire1.select(consts.zero64, mem_addr)
                mem_wdata = shadow_boundary_fire1.select(consts.zero64, mem_wdata)
                mem_rdata = shadow_boundary_fire1.select(consts.zero64, mem_rdata)
                mem_size = shadow_boundary_fire1.select(consts.zero4, mem_size)
                next_pc = shadow_boundary_fire1.select(commit_pcs[1], next_pc)
                checkpoint_id = shadow_boundary_fire1.select(rob_checkpoint_ids[1], checkpoint_id)
            if slot == 1:
                fire = macro_shadow_fire.select(consts.one1, fire)
                pc = macro_shadow_fire.select(macro_trace_pc, pc)
                rob_idx = macro_shadow_fire.select(macro_trace_rob, rob_idx)
                op = macro_shadow_fire.select(macro_trace_op, op)
                val = macro_shadow_fire.select(macro_trace_val, val)
                ln = macro_shadow_fire.select(macro_trace_len, ln)
                insn_raw = macro_shadow_fire.select(macro_trace_insn, insn_raw)
                wb_valid = macro_shadow_fire.select(macro_trace_wb_valid, wb_valid)
                wb_rd = macro_shadow_fire.select(macro_trace_wb_rd, wb_rd)
                wb_data = macro_shadow_fire.select(macro_trace_wb_data, wb_data)
                mem_valid = macro_shadow_fire.select(macro_trace_mem_valid, mem_valid)
                mem_is_store = macro_shadow_fire.select(macro_trace_mem_is_store, mem_is_store)
                mem_addr = macro_shadow_fire.select(macro_trace_mem_addr, mem_addr)
                mem_wdata = macro_shadow_fire.select(macro_trace_mem_wdata, mem_wdata)
                mem_rdata = macro_shadow_fire.select(macro_trace_mem_rdata, mem_rdata)
                mem_size = macro_shadow_fire.select(macro_trace_mem_size, mem_size)
                next_pc = macro_shadow_fire.select(macro_trace_next_pc, next_pc)
            macro_slot_keep = consts.zero1
            if slot == 1:
                macro_slot_keep = macro_shadow_fire
            macro_kill = macro_trace_fire & (~macro_slot_keep)
            fire = macro_kill.select(consts.zero1, fire)
            pc = macro_kill.select(consts.zero64, pc)
            rob_idx = macro_kill.select(c(0, width=p.rob_w), rob_idx)
            op = macro_kill.select(c(0, width=12), op)
            val = macro_kill.select(consts.zero64, val)
            ln = macro_kill.select(consts.zero3, ln)
            insn_raw = macro_kill.select(consts.zero64, insn_raw)
            wb_valid = macro_kill.select(consts.zero1, wb_valid)
            wb_rd = macro_kill.select(c(0, width=6), wb_rd)
            wb_data = macro_kill.select(consts.zero64, wb_data)
            mem_valid = macro_kill.select(consts.zero1, mem_valid)
            mem_is_store = macro_kill.select(consts.zero1, mem_is_store)
            mem_addr = macro_kill.select(consts.zero64, mem_addr)
            mem_wdata = macro_kill.select(consts.zero64, mem_wdata)
            mem_rdata = macro_kill.select(consts.zero64, mem_rdata)
            mem_size = macro_kill.select(consts.zero4, mem_size)
            next_pc = macro_kill.select(consts.zero64, next_pc)
            checkpoint_id = macro_kill.select(c(0, width=6), checkpoint_id)
        m.output(f"commit_fire{slot}", fire)
        m.output(f"commit_pc{slot}", pc)
        m.output(f"commit_rob{slot}", rob_idx)
        m.output(f"commit_op{slot}", op)
        m.output(f"commit_value{slot}", val)
        m.output(f"commit_len{slot}", ln)
        m.output(f"commit_insn_raw{slot}", insn_raw)
        m.output(f"commit_wb_valid{slot}", wb_valid)
        m.output(f"commit_wb_rd{slot}", wb_rd)
        m.output(f"commit_wb_data{slot}", wb_data)
        m.output(f"commit_mem_valid{slot}", mem_valid)
        m.output(f"commit_mem_is_store{slot}", mem_is_store)
        m.output(f"commit_mem_addr{slot}", mem_addr)
        m.output(f"commit_mem_wdata{slot}", mem_wdata)
        m.output(f"commit_mem_rdata{slot}", mem_rdata)
        m.output(f"commit_mem_size{slot}", mem_size)
        m.output(f"commit_trap_valid{slot}", trap_valid)
        m.output(f"commit_trap_cause{slot}", trap_cause)
        m.output(f"commit_next_pc{slot}", next_pc)
        m.output(f"commit_checkpoint_id{slot}", checkpoint_id)
    m.output("rob_count", rob.count)

    # Debug: committed vs speculative hand tops (T0/U0).
    ct0_tag = ren.cmap[24].out()
    cu0_tag = ren.cmap[28].out()
    st0_tag = ren.smap[24].out()
    su0_tag = ren.smap[28].out()
    m.output("ct0", mux_by_uindex(m, idx=ct0_tag, items=prf, default=consts.zero64))
    m.output("cu0", mux_by_uindex(m, idx=cu0_tag, items=prf, default=consts.zero64))
    m.output("st0", mux_by_uindex(m, idx=st0_tag, items=prf, default=consts.zero64))
    m.output("su0", mux_by_uindex(m, idx=su0_tag, items=prf, default=consts.zero64))

    # Debug: issue/memory arbitration visibility.
    m.output("issue_fire", issue_fire)
    m.output("issue_op", uop_op)
    m.output("issue_pc", uop_pc)
    m.output("issue_rob", uop_rob)
    m.output("issue_sl", uop_sl)
    m.output("issue_sr", uop_sr)
    m.output("issue_sp", uop_sp)
    m.output("issue_pdst", uop_pdst)
    m.output("issue_sl_val", sl_val)
    m.output("issue_sr_val", sr_val)
    m.output("issue_sp_val", sp_val)
    m.output("issue_is_load", issued_is_load)
    m.output("issue_is_store", issued_is_store)
    m.output("store_pending", store_pending)
    m.output("store_pending_older", older_store_pending)
    m.output("mem_raddr", dmem_raddr)
    m.output("dmem_raddr", dmem_raddr)
    m.output("dmem_wvalid", mem_wvalid)
    m.output("dmem_waddr", mem_waddr)
    m.output("dmem_wdata", dmem_wdata)
    m.output("dmem_wstrb", wstrb)
    m.output("dmem_wsrc", dmem_wsrc)
    m.output("stbuf_enq_fire", stbuf_enq_fire)
    m.output("stbuf_drain_fire", stbuf_drain_fire)
    m.output("macro_store_fire_dbg", macro_store_fire)
    m.output("commit_store_wt_fire_dbg", commit_store_write_through)
    m.output("frontend_ready", frontend_ready)
    m.output("redirect_valid", commit_redirect)
    m.output("redirect_pc", redirect_pc)
    m.output("redirect_checkpoint_id", redirect_checkpoint_id)
    m.output("ctu_block_ifu", macro_block)
    m.output("ctu_uop_valid", macro_uop_valid)
    m.output("ctu_uop_kind", macro_uop_kind)
    m.output("ctu_uop_reg", macro_uop_reg)
    m.output("ctu_uop_addr", macro_uop_addr)

    # Deadlock diagnostics: locate the IQ entry currently backing ROB head.
    head_wait_hit = consts.zero1
    head_wait_kind = c(0, width=2)  # 1=ALU, 2=BRU, 3=LSU
    head_wait_sl = tag0
    head_wait_sr = tag0
    head_wait_sp = tag0
    head_idx = rob.head.out()
    for i in range(p.iq_depth):
        idx = c(i, width=p.iq_w)
        hit_lsu = iq_lsu.valid[i].out() & iq_lsu.rob[i].out().eq(head_idx)
        hit_bru = iq_bru.valid[i].out() & iq_bru.rob[i].out().eq(head_idx)
        hit_alu = iq_alu.valid[i].out() & iq_alu.rob[i].out().eq(head_idx)
        head_wait_hit = (hit_lsu | hit_bru | hit_alu).select(consts.one1, head_wait_hit)
        head_wait_kind = hit_lsu.select(c(3, width=2), head_wait_kind)
        head_wait_kind = hit_bru.select(c(2, width=2), head_wait_kind)
        head_wait_kind = hit_alu.select(c(1, width=2), head_wait_kind)
        head_wait_sl = hit_lsu.select(iq_lsu.srcl[i].out(), head_wait_sl)
        head_wait_sr = hit_lsu.select(iq_lsu.srcr[i].out(), head_wait_sr)
        head_wait_sp = hit_lsu.select(iq_lsu.srcp[i].out(), head_wait_sp)
        head_wait_sl = hit_bru.select(iq_bru.srcl[i].out(), head_wait_sl)
        head_wait_sr = hit_bru.select(iq_bru.srcr[i].out(), head_wait_sr)
        head_wait_sp = hit_bru.select(iq_bru.srcp[i].out(), head_wait_sp)
        head_wait_sl = hit_alu.select(iq_alu.srcl[i].out(), head_wait_sl)
        head_wait_sr = hit_alu.select(iq_alu.srcr[i].out(), head_wait_sr)
        head_wait_sp = hit_alu.select(iq_alu.srcp[i].out(), head_wait_sp)
    head_wait_sl_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=head_wait_sl, width=p.pregs)
    head_wait_sr_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=head_wait_sr, width=p.pregs)
    head_wait_sp_rdy = mask_bit(m, mask=ren.ready_mask.out(), idx=head_wait_sp, width=p.pregs)
    m.output("head_wait_hit", head_wait_hit)
    m.output("head_wait_kind", head_wait_kind)
    m.output("head_wait_sl", head_wait_sl)
    m.output("head_wait_sr", head_wait_sr)
    m.output("head_wait_sp", head_wait_sp)
    m.output("head_wait_sl_rdy", head_wait_sl_rdy)
    m.output("head_wait_sr_rdy", head_wait_sr_rdy)
    m.output("head_wait_sp_rdy", head_wait_sp_rdy)
    # Debug taps for scheduler/LSU replay bring-up.
    any_wakeup = consts.zero1
    for slot in range(p.issue_w):
        any_wakeup = any_wakeup | wb_fire_has_dsts[slot]
    wakeup_reason = c(0, width=8)
    wakeup_reason = any_wakeup.select(wakeup_reason | c(1 << 0, width=8), wakeup_reason)
    wakeup_reason = lsu_forward_active.select(wakeup_reason | c(1 << 1, width=8), wakeup_reason)
    wakeup_reason = commit_redirect.select(wakeup_reason | c(1 << 2, width=8), wakeup_reason)
    wakeup_reason = dispatch_fire.select(wakeup_reason | c(1 << 3, width=8), wakeup_reason)
    replay_cause = c(0, width=8)
    replay_cause = lsu_block_lane0.select(replay_cause | c(1 << 0, width=8), replay_cause)
    replay_cause = (issued_is_load & older_store_pending).select(replay_cause | c(1 << 1, width=8), replay_cause)
    replay_cause = lsu_violation_detected.select(replay_cause | c(1 << 2, width=8), replay_cause)
    replay_cause = replay_redirect_fire.select(replay_cause | c(1 << 3, width=8), replay_cause)
    m.output("wakeup_reason", wakeup_reason)
    m.output("replay_cause", replay_cause)
    m.output("dispatch_fire", dispatch_fire)
    m.output("dec_op", dec_op)

    # Dispatch slot visibility (trace hook): per-slot PC/op/ROB for pipeview tools.
    max_disp_slots = 4
    for slot in range(max_disp_slots):
        fire = consts.zero1
        pc = consts.zero64
        rob_i = c(0, width=p.rob_w)
        op = c(0, width=12)
        if slot < p.dispatch_w:
            fire = disp_fires[slot]
            pc = disp_pcs[slot]
            rob_i = disp_rob_idxs[slot]
            op = disp_ops[slot]
        m.output(f"dispatch_fire{slot}", fire)
        m.output(f"dispatch_pc{slot}", pc)
        m.output(f"dispatch_rob{slot}", rob_i)
        m.output(f"dispatch_op{slot}", op)

    # Issue slot visibility (trace hook): per-slot PC/op/ROB for pipeview tools.
    max_issue_slots = 4
    for slot in range(max_issue_slots):
        fire = consts.zero1
        pc = consts.zero64
        rob_i = c(0, width=p.rob_w)
        op = c(0, width=12)
        if slot < p.issue_w:
            fire = issue_fires_eff[slot]
            pc = uop_pcs[slot]
            rob_i = uop_robs[slot]
            op = uop_ops[slot]
        m.output(f"issue_fire{slot}", fire)
        m.output(f"issue_pc{slot}", pc)
        m.output(f"issue_rob{slot}", rob_i)
        m.output(f"issue_op{slot}", op)

    # MMIO visibility for testbenches (UART + exit).
    m.output("mmio_uart_valid", mmio_uart)
    m.output("mmio_uart_data", mmio_uart_data)
    m.output("mmio_exit_valid", mmio_exit)
    m.output("mmio_exit_code", mmio_exit_code)

    # Block command export for Janus BCtrl/TMU/PE top-level bring-up.
    block_cmd_valid = commit_fire & op_is(head_op, OP_C_BSTART_STD, OP_C_BSTART_COND, OP_BSTART_STD_CALL)
    block_cmd_kind = head_op.eq(c(OP_C_BSTART_COND, width=6)).select(c(1, width=2), c(0, width=2))
    block_cmd_kind = head_op.eq(c(OP_BSTART_STD_CALL, width=6)).select(c(2, width=2), block_cmd_kind)
    block_cmd_payload = head_value
    block_cmd_tile = head_value.trunc(width=6)
    block_cmd_tag = state.cycles.out().trunc(width=8)

    ooo_4wide = c(1 if (p.fetch_w == 4 and p.dispatch_w == 4 and p.issue_w == 4 and p.commit_w == 4) else 0, width=1)
    m.output("ooo_4wide", ooo_4wide)
    m.output("block_cmd_valid", block_cmd_valid)
    m.output("block_cmd_kind", block_cmd_kind)
    m.output("block_cmd_payload", block_cmd_payload)
    m.output("block_cmd_tile", block_cmd_tile)
    m.output("block_cmd_tag", block_cmd_tag)

    return BccOooExports(
        clk=clk,
        rst=rst,
        block_cmd_valid=block_cmd_valid.sig,
        block_cmd_kind=block_cmd_kind.sig,
        block_cmd_payload=block_cmd_payload.sig,
        block_cmd_tile=block_cmd_tile.sig,
        block_cmd_tag=block_cmd_tag.sig,
        cycles=state.cycles.out().sig,
        halted=state.halted.out().sig,
    )


@module(name="LinxCoreBackend")
def build_backend(m: Circuit, *, mem_bytes: int = (1 << 20)) -> None:
    build_bcc_ooo(m, mem_bytes=mem_bytes)
