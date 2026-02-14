from __future__ import annotations

from pycircuit import Circuit

from ..backend.backend import build_backend
from ..frontend.frontend import build_frontend
from ..mem.mem2r1w import build_mem2r1w


def build_top(m: Circuit, *, mem_bytes: int = (1 << 20)) -> None:
    clk = m.clock("clk")
    rst = m.reset("rst")

    boot_pc = m.input("boot_pc", width=64)
    boot_sp = m.input("boot_sp", width=64)
    boot_ra = m.input("boot_ra", width=64)

    host_wvalid = m.input("host_wvalid", width=1)
    host_waddr = m.input("host_waddr", width=64)
    host_wdata = m.input("host_wdata", width=64)
    host_wstrb = m.input("host_wstrb", width=8)

    c = m.const

    backend_ready_w = m.new_wire(width=1)
    redirect_valid_w = m.new_wire(width=1)
    redirect_pc_w = m.new_wire(width=64)
    if_rdata_w = m.new_wire(width=64)
    d_rdata_w = m.new_wire(width=64)

    frontend = m.instance(
        build_frontend,
        name="frontend",
        params={"ibuf_depth": 8, "ftq_depth": 16},
        clk=clk,
        rst=rst,
        boot_pc=boot_pc,
        imem_rdata=if_rdata_w,
        backend_ready=backend_ready_w,
        redirect_valid=redirect_valid_w,
        redirect_pc=redirect_pc_w,
        flush_valid=c(0, width=1),
        flush_pc=c(0, width=64),
    )

    backend = m.instance(
        build_backend,
        name="backend",
        params={"mem_bytes": mem_bytes},
        clk=clk,
        rst=rst,
        boot_pc=boot_pc,
        boot_sp=boot_sp,
        boot_ra=boot_ra,
        f4_valid_i=frontend["f4_valid"],
        f4_pc_i=frontend["f4_pc"],
        f4_window_i=frontend["f4_window"],
        f4_checkpoint_i=frontend["checkpoint_id"],
        dmem_rdata_i=d_rdata_w,
    )

    mem = m.instance(
        build_mem2r1w,
        name="mem2r1w",
        params={"mem_bytes": mem_bytes},
        clk=clk,
        rst=rst,
        if_raddr=frontend["imem_raddr"],
        d_raddr=backend["dmem_raddr"],
        d_wvalid=backend["dmem_wvalid"],
        d_waddr=backend["dmem_waddr"],
        d_wdata=backend["dmem_wdata"],
        d_wstrb=backend["dmem_wstrb"],
        host_wvalid=host_wvalid,
        host_waddr=host_waddr,
        host_wdata=host_wdata,
        host_wstrb=host_wstrb,
    )

    m.assign(backend_ready_w, backend["frontend_ready"])
    m.assign(redirect_valid_w, backend["redirect_valid"])
    m.assign(redirect_pc_w, backend["redirect_pc"])
    m.assign(if_rdata_w, mem["if_rdata"])
    m.assign(d_rdata_w, mem["d_rdata"])

    export_ports = [
        "cycles",
        "halted",
        "pc",
        "fpc",
        "a0",
        "a1",
        "ra",
        "sp",
        "mmio_uart_valid",
        "mmio_uart_data",
        "mmio_exit_valid",
        "mmio_exit_code",
        "dispatch_fire",
        "dec_op",
        "issue_fire",
        "issue_op",
        "issue_pc",
        "issue_rob",
        "issue_sl",
        "issue_sr",
        "issue_sp",
        "issue_pdst",
        "issue_sl_val",
        "issue_sr_val",
        "issue_sp_val",
        "issue_is_load",
        "issue_is_store",
        "store_pending",
        "store_pending_older",
        "mem_raddr",
        "dmem_raddr",
        "dmem_wvalid",
        "dmem_waddr",
        "dmem_wdata",
        "dmem_wstrb",
        "dmem_wsrc",
        "stbuf_enq_fire",
        "stbuf_drain_fire",
        "macro_store_fire_dbg",
        "commit_store_wt_fire_dbg",
        "ooo_4wide",
        "block_cmd_valid",
        "block_cmd_kind",
        "block_cmd_payload",
        "block_cmd_tile",
        "block_cmd_tag",
        "rob_count",
        "rob_head_valid",
        "rob_head_done",
        "rob_head_pc",
        "rob_head_insn_raw",
        "rob_head_len",
        "rob_head_op",
        "ctu_block_ifu",
        "ctu_uop_valid",
        "ctu_uop_kind",
        "ctu_uop_reg",
        "ctu_uop_addr",
        "head_wait_hit",
        "head_wait_kind",
        "head_wait_sl",
        "head_wait_sr",
        "head_wait_sp",
        "head_wait_sl_rdy",
        "head_wait_sr_rdy",
        "head_wait_sp_rdy",
    ]
    for name in export_ports:
        m.output(name, backend[name])

    for slot in range(4):
        m.output(f"dispatch_fire{slot}", backend[f"dispatch_fire{slot}"])
        m.output(f"dispatch_pc{slot}", backend[f"dispatch_pc{slot}"])
        m.output(f"dispatch_rob{slot}", backend[f"dispatch_rob{slot}"])
        m.output(f"dispatch_op{slot}", backend[f"dispatch_op{slot}"])
        m.output(f"issue_fire{slot}", backend[f"issue_fire{slot}"])
        m.output(f"issue_pc{slot}", backend[f"issue_pc{slot}"])
        m.output(f"issue_rob{slot}", backend[f"issue_rob{slot}"])
        m.output(f"issue_op{slot}", backend[f"issue_op{slot}"])
        m.output(f"commit_fire{slot}", backend[f"commit_fire{slot}"])
        m.output(f"commit_pc{slot}", backend[f"commit_pc{slot}"])
        m.output(f"commit_rob{slot}", backend[f"commit_rob{slot}"])
        m.output(f"commit_op{slot}", backend[f"commit_op{slot}"])
        m.output(f"commit_value{slot}", backend[f"commit_value{slot}"])
        m.output(f"commit_len{slot}", backend[f"commit_len{slot}"])
        m.output(f"commit_insn_raw{slot}", backend[f"commit_insn_raw{slot}"])
        m.output(f"commit_wb_valid{slot}", backend[f"commit_wb_valid{slot}"])
        m.output(f"commit_wb_rd{slot}", backend[f"commit_wb_rd{slot}"])
        m.output(f"commit_wb_data{slot}", backend[f"commit_wb_data{slot}"])
        m.output(f"commit_mem_valid{slot}", backend[f"commit_mem_valid{slot}"])
        m.output(f"commit_mem_is_store{slot}", backend[f"commit_mem_is_store{slot}"])
        m.output(f"commit_mem_addr{slot}", backend[f"commit_mem_addr{slot}"])
        m.output(f"commit_mem_wdata{slot}", backend[f"commit_mem_wdata{slot}"])
        m.output(f"commit_mem_rdata{slot}", backend[f"commit_mem_rdata{slot}"])
        m.output(f"commit_mem_size{slot}", backend[f"commit_mem_size{slot}"])
        m.output(f"commit_trap_valid{slot}", backend[f"commit_trap_valid{slot}"])
        m.output(f"commit_trap_cause{slot}", backend[f"commit_trap_cause{slot}"])
        m.output(f"commit_next_pc{slot}", backend[f"commit_next_pc{slot}"])
        m.output(f"commit_checkpoint_id{slot}", backend[f"commit_checkpoint_id{slot}"])

    # Bring-up debug taps.
    m.output("ftq_head", frontend["ftq_head"])
    m.output("ftq_tail", frontend["ftq_tail"])
    m.output("ftq_count", frontend["ftq_count"])
    m.output("checkpoint_id", frontend["checkpoint_id"])
    m.output("redirect_checkpoint_id", backend["redirect_checkpoint_id"])
    m.output("wakeup_reason", backend["wakeup_reason"])
    m.output("replay_cause", backend["replay_cause"])


build_top.__pycircuit_name__ = "linxcore_ooo_pyc"
