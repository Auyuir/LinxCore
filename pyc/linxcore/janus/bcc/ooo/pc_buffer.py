from __future__ import annotations

from pycircuit import Circuit, module


@module(name="JanusBccOooPcBuffer")
def build_janus_bcc_ooo_pc_buffer(m: Circuit, *, depth: int = 64, idx_w: int = 6) -> None:
    clk_top = m.clock("clk")
    rst_top = m.reset("rst")

    wr_valid_pcb = m.input("wr_valid_pcb", width=1)
    wr_idx_pcb = m.input("wr_idx_pcb", width=idx_w)
    wr_pc_pcb = m.input("wr_pc_pcb", width=64)
    rd_idx_pcb = m.input("rd_idx_pcb", width=idx_w)

    c = m.const

    pcs_pcb = []
    for i in range(depth):
        pcs_pcb.append(m.out(f"pc{i}_pcb", clk=clk_top, rst=rst_top, width=64, init=c(0, width=64), en=c(1, width=1)))

    rd_pc_pcb = c(0, width=64)
    for i in range(depth):
        idx_hit_pcb = rd_idx_pcb == c(i, width=idx_w)
        wr_hit_pcb = wr_idx_pcb == c(i, width=idx_w)
        rd_pc_pcb = idx_hit_pcb._select_internal(pcs_pcb[i].out(), rd_pc_pcb)
        pcs_pcb[i].set(wr_pc_pcb, when=wr_valid_pcb & wr_hit_pcb)

    m.output("rd_pc_pcb", rd_pc_pcb)
