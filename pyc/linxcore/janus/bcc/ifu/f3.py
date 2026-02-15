from __future__ import annotations

from pycircuit import Circuit, module

from ....frontend.ibuffer import build_ibuffer


@module(name="JanusBccIfuF3")
def build_janus_bcc_ifu_f3(m: Circuit, *, ibuf_depth: int = 8) -> None:
    clk_top = m.clock("clk")
    rst_top = m.reset("rst")

    f2_to_f3_stage_pc_f2 = m.input("f2_to_f3_stage_pc_f2", width=64)
    f2_to_f3_stage_window_f2 = m.input("f2_to_f3_stage_window_f2", width=64)
    f2_to_f3_stage_valid_f2 = m.input("f2_to_f3_stage_valid_f2", width=1)
    ctrl_to_f3_stage_checkpoint_id_f3 = m.input("ctrl_to_f3_stage_checkpoint_id_f3", width=6)

    backend_ready_top = m.input("backend_ready_top", width=1)
    flush_valid_fls = m.input("flush_valid_fls", width=1)

    c = m.const

    ibuf_f3 = m.instance(
        build_ibuffer,
        name="janus_f3_ibuffer",
        module_name="JanusBccIfuF3IBuffer",
        params={"depth": ibuf_depth},
        clk=clk_top,
        rst=rst_top,
        push_valid=f2_to_f3_stage_valid_f2,
        push_pc=f2_to_f3_stage_pc_f2,
        push_window=f2_to_f3_stage_window_f2,
        pop_ready=backend_ready_top,
        flush_valid=flush_valid_fls,
    )

    m.output("f3_to_f4_stage_pc_f3", ibuf_f3["out_pc"])
    m.output("f3_to_f4_stage_window_f3", ibuf_f3["out_window"])
    m.output("f3_to_f4_stage_valid_f3", ibuf_f3["out_valid"])
    m.output("f3_to_f4_stage_checkpoint_id_f3", ctrl_to_f3_stage_checkpoint_id_f3)
    m.output("f3_ibuf_count_f3", ibuf_f3["count_dbg"])
    m.output("f3_ibuf_ready_f3", ibuf_f3["push_ready"])
    m.output("f3_pop_fire_f3", ibuf_f3["pop_fire"])
    m.output("f3_one_f3", c(1, width=1))
