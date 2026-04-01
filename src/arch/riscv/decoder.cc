/*
 * Copyright (c) 2012 Google
 * Copyright (c) The University of Virginia
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/riscv/decoder.hh"

#include "arch/riscv/insts/zcmt.hh"
#include "arch/riscv/isa.hh"

#ifndef Support_ReadCkpt_TempRegs
#define Support_ReadCkpt_TempRegs 1
#endif

#if Support_ReadCkpt_TempRegs
#include "arch/riscv/regs/int.hh"

#endif
#include "arch/riscv/types.hh"
#include "base/bitfield.hh"
#include "debug/Decode.hh"

namespace gem5
{

namespace RiscvISA
{

Decoder::Decoder(const RiscvDecoderParams &p) : InstDecoder(p, &machInst)
{
    ISA *isa = dynamic_cast<ISA*>(p.isa);
    vlen = isa->getVecLenInBits();
    elen = isa->getVecElemLenInBits();
    _enableZcd = isa->enableZcd();
    reset();
}

void Decoder::reset()
{
    aligned = true;
    mid = false;
    machInst = 0;
    emi = 0;
    jvtEntry = 0;
    squashed = true;
}

void
Decoder::moreBytes(const PCStateBase &pc, Addr fetchPC)
{
    // The MSB of the upper and lower halves of a machine instruction.
    constexpr size_t max_bit = sizeof(machInst) * 8 - 1;
    constexpr size_t mid_bit = sizeof(machInst) * 4 - 1;

    auto inst = letoh(machInst);
    DPRINTF(Decode, "Requesting bytes 0x%08x from address %#x\n", inst,
            fetchPC);

    PCState pc_state = pc.as<PCState>();

    if (GEM5_UNLIKELY(pc_state.zcmtSecondFetch())) {
        if (mid) {
            replaceBits(jvtEntry, sizeof(jvtEntry) * 8 - 1, max_bit + 1, inst);
            mid = false;
            instDone = true;
            outOfBytes = true;
        } else {
            replaceBits(jvtEntry, max_bit, 0, inst);
            mid = (pc_state.rvType() != RV32);
            instDone = (pc_state.rvType() == RV32);
            outOfBytes = true;
        }

        if (instDone && pc_state.rvType() == RV32) {
            jvtEntry = sext<32>(jvtEntry);
        }
        return;
    }

    bool aligned = pc.instAddr() % sizeof(machInst) == 0;
    if (aligned) {
        emi.instBits = inst;
        if (compressed(inst))
            emi.instBits = bits(inst, mid_bit, 0);
        outOfBytes = !compressed(emi);
        instDone = true;
    } else {
        if (mid) {
            assert(bits(emi.instBits, max_bit, mid_bit + 1) == 0);
            replaceBits(emi.instBits, max_bit, mid_bit + 1, inst);
            mid = false;
            outOfBytes = false;
            instDone = true;
        } else {
            emi.instBits = bits(inst, max_bit, mid_bit + 1);
            mid = !compressed(emi);
            outOfBytes = true;
            instDone = compressed(emi);
        }
    }
}

StaticInstPtr
Decoder::decode(ExtMachInst mach_inst, Addr addr)
{
    DPRINTF(Decode, "Decoding instruction 0x%08x at address %#x\n",
            mach_inst.instBits, addr);

#if Support_ReadCkpt_TempRegs
    // Pseudo ops encoded as addi x0, rs1, imm12:
    // imm=9..12:  write rs1 into TMP1..TMP4
    // imm=36..39: read TMP1..TMP4 into rs1
    // imm=64:     jump to TMP(rs1[1:0]+1)
    constexpr uint32_t tempAddInstBits = 0x80b3;  // add x1, x1, x0
    constexpr uint32_t tempJalrInstBits = 0x8067; // jalr x0, 0(x1)

    const bool is_uncompressed = !compressed(mach_inst);
    const uint32_t inst_bits = mach_inst.instBits;
    const uint32_t opcode = bits(inst_bits, 6, 0);
    const uint32_t rd = bits(inst_bits, 11, 7);
    const uint32_t func3 = bits(inst_bits, 14, 12);
    const uint32_t rs1 = bits(inst_bits, 19, 15);
    const uint32_t imm12 = bits(inst_bits, 31, 20);

    const bool is_new_inst =
        is_uncompressed && (opcode == 0x13) && (func3 == 0x0) && (rd == 0);
    const bool is_wtemp = is_new_inst && (imm12 >= 9) && (imm12 <= 12);
    const bool is_rtemp = is_new_inst && (imm12 >= 36) && (imm12 <= 39);
    const bool is_jtemp = is_new_inst && (imm12 == 64);

    if (is_wtemp || is_rtemp || is_jtemp) {
        ExtMachInst rewritten = mach_inst;
        rewritten.instBits = is_jtemp ? tempJalrInstBits : tempAddInstBits;

        StaticInstPtr si = decodeInst(rewritten);
        si->size(4);

        if (is_rtemp) {
            const auto temp_idx = int_reg::_TMP1Idx + (imm12 - 36);
            si->setSrcRegIdx(0, intRegClass[temp_idx]);
            si->setDestRegIdx(0, intRegClass[rs1]);
        } else if (is_wtemp) {
            const auto temp_idx = int_reg::_TMP1Idx + (imm12 - 9);
            si->setSrcRegIdx(0, intRegClass[rs1]);
            si->setDestRegIdx(0, intRegClass[temp_idx]);
        } else {
            const auto temp_idx = int_reg::_TMP1Idx + (rs1 % 4);
            si->setSrcRegIdx(0, intRegClass[temp_idx]);
        }

        DPRINTF(Decode, "Decode: Decoded %s instruction: %#x\n",
                si->getName(), rewritten);
        return si;
    }
#endif

    StaticInstPtr &si = instMap[mach_inst];
    if (!si)
        si = decodeInst(mach_inst);

    si->size(compressed(mach_inst) ? 2 : 4);

    DPRINTF(Decode, "Decode: Decoded %s instruction: %#x\n",
            si->getName(), mach_inst);
    return si;
}

StaticInstPtr
Decoder::decode(PCStateBase &_next_pc)
{
    if (!instDone)
        return nullptr;
    instDone = false;

    auto &next_pc = _next_pc.as<PCState>();

    if (GEM5_UNLIKELY(next_pc.zcmtSecondFetch())) {
        return new ZcmtSecondFetchInst(emi, jvtEntry);
    }

    if (compressed(emi)) {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst) / 2);
        next_pc.compressed(true);
    } else {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst));
        next_pc.compressed(false);
    }

    if (GEM5_UNLIKELY(squashed || next_pc.new_vconf())) {
        squashed = false;
        next_pc.new_vconf(false);
        vl = next_pc.vl();
        vtype = next_pc.vtype();
    } else {
        next_pc.vl(vl);
        next_pc.vtype(vtype);
    }

    emi.vl      = vl;
    emi.vtype8  = vtype & 0xff;
    emi.vill    = vtype.vill;
    emi.rv_type = static_cast<int>(next_pc.rvType());
    emi.enable_zcd = _enableZcd;

    return decode(emi, next_pc.instAddr());
}

} // namespace RiscvISA
} // namespace gem5
