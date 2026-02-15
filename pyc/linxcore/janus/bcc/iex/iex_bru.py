from __future__ import annotations

from pycircuit import Circuit, module

from ....common.isa import OP_C_BSTART_STD, OP_C_BSTOP


@module(name="JanusBccIexBru")
def build_janus_bcc_iex_bru(m: Circuit) -> None:
    in_valid_i1 = m.input("in_valid_i1", width=1)
    in_op_i1 = m.input("in_op_i1", width=12)
    in_pc_i1 = m.input("in_pc_i1", width=64)
    in_imm_i1 = m.input("in_imm_i1", width=64)

    c = m.const
    is_boundary_e1 = (in_op_i1 == c(OP_C_BSTART_STD, width=12)) | (in_op_i1 == c(OP_C_BSTOP, width=12))
    redirect_valid_e1 = in_valid_i1 & is_boundary_e1
    redirect_pc_e1 = in_pc_i1 + in_imm_i1

    m.output("redirect_valid_e1", redirect_valid_e1)
    m.output("redirect_pc_e1", redirect_pc_e1)
