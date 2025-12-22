/*
 * Copyright (c) 2012, 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#include "cpu/o3/decode.hh"

#include "arch/generic/pcstate.hh"
#include "base/trace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/nop_static_inst.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/limits.hh"
#include "debug/Activity.hh"
#include "debug/Decode.hh"
#include "debug/O3PipeView.hh"
#include "debug/Special.hh"
#include "enums/BranchType.hh"
#include "params/BaseO3CPU.hh"
#include "sim/full_system.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::list;

namespace gem5
{

namespace o3
{

Decode::Decode(CPU *_cpu, const BaseO3CPUParams &params)
    : cpu(_cpu),
      branchPred(params.branchPred),
      renameToDecodeDelay(params.renameToDecodeDelay),
      iewToDecodeDelay(params.iewToDecodeDelay),
      commitToDecodeDelay(params.commitToDecodeDelay),
      fetchToDecodeDelay(params.fetchToDecodeDelay),
      decodeWidth(params.decodeWidth),
      numThreads(params.numThreads),
      stats(_cpu)
{
    if (decodeWidth > MaxWidth)
        fatal("decodeWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/limits.hh\n",
             decodeWidth, static_cast<int>(MaxWidth));

    // @todo: Make into a parameter
    skidBufferMax = (fetchToDecodeDelay + 1) *  params.decodeWidth;
    for (int tid = 0; tid < MaxThreads; tid++) {
        stalls[tid] = {false};
        decodeStatus[tid] = Idle;
        bdelayDoneSeqNum[tid] = 0;
        squashInst[tid] = nullptr;
        squashAfterDelaySlot[tid] = 0;
    }
}

void
Decode::startupStage()
{
    resetStage();
}

void
Decode::clearStates(ThreadID tid)
{
    decodeStatus[tid] = Idle;
    stalls[tid].rename = false;

    // Clear out any of this thread's instructions being sent to rename.
    for (int i = -cpu->decodeQueue.getPast();
         i <= cpu->decodeQueue.getFuture(); ++i) {
        DecodeStruct& decode_struct = cpu->decodeQueue[i];
        removeCommThreadInsts(tid, decode_struct);
    }

    // Clear out any of this thread's instructions being sent to fetch.
    for (int i = -cpu->timeBuffer.getPast();
         i <= cpu->timeBuffer.getFuture(); ++i) {
        TimeStruct& time_struct = cpu->timeBuffer[i];
        time_struct.decodeInfo[tid] = {};
        time_struct.decodeBlock[tid] = false;
        time_struct.decodeUnblock[tid] = false;
    }
}

void
Decode::resetStage()
{
    _status = Inactive;

    // Setup status, make sure stall signals are clear.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        decodeStatus[tid] = Idle;

        stalls[tid].rename = false;
    }
}

std::string
Decode::name() const
{
    return cpu->name() + ".decode";
}

Decode::DecodeStats::DecodeStats(CPU *cpu)
    : statistics::Group(cpu, "decode"),
      ADD_STAT(idleCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is idle"),
      ADD_STAT(blockedCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is blocked"),
      ADD_STAT(runCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is running"),
      ADD_STAT(unblockCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is unblocking"),
      ADD_STAT(squashCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is squashing"),
      ADD_STAT(branchResolved, statistics::units::Count::get(),
               "Number of times decode resolved a branch"),
      ADD_STAT(branchMispred, statistics::units::Count::get(),
               "Number of times decode detected a branch misprediction"),
      ADD_STAT(controlMispred, statistics::units::Count::get(),
               "Number of times decode detected an instruction incorrectly "
               "predicted as a control"),
      ADD_STAT(decodedInsts, statistics::units::Count::get(),
               "Number of instructions handled by decode"),
      ADD_STAT(squashedInsts, statistics::units::Count::get(),
               "Number of squashed instructions handled by decode")
{
    idleCycles.prereq(idleCycles);
    blockedCycles.prereq(blockedCycles);
    runCycles.prereq(runCycles);
    unblockCycles.prereq(unblockCycles);
    squashCycles.prereq(squashCycles);
    branchResolved.prereq(branchResolved);
    branchMispred.prereq(branchMispred);
    controlMispred.prereq(controlMispred);
    decodedInsts.prereq(decodedInsts);
    squashedInsts.prereq(squashedInsts);
}

void
Decode::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to write information back to fetch.
    toFetch = timeBuffer->getWire(0);

    // Create wires to get information from proper places in time buffer.
    fromRename = timeBuffer->getWire(-renameToDecodeDelay);
    fromIEW = timeBuffer->getWire(-iewToDecodeDelay);
    fromCommit = timeBuffer->getWire(-commitToDecodeDelay);
}

void
Decode::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    decodeQueue = dq_ptr;

    // Setup wire to write information to proper place in decode queue.
    toRename = decodeQueue->getWire(0);
}

void
Decode::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    fetchQueue = fq_ptr;

    // Setup wire to read information from fetch queue.
    fromFetch = fetchQueue->getWire(-fetchToDecodeDelay);
}

void
Decode::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

void
Decode::drainSanityCheck() const
{
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        assert(insts[tid].empty());
        assert(skidBuffer[tid].empty());
    }
}

bool
Decode::isDrained() const
{
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        if (!insts[tid].empty() || !skidBuffer[tid].empty() ||
                (decodeStatus[tid] != Running && decodeStatus[tid] != Idle))
            return false;
    }
    return true;
}

bool
Decode::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (stalls[tid].rename) {
        DPRINTF(Decode,"[tid:%i] Stall fom Rename stage detected.\n", tid);
        ret_val = true;
    }

    return ret_val;
}

bool
Decode::fetchInstsValid()
{
    return fromFetch->size > 0;
}

bool
Decode::block(ThreadID tid)
{
    DPRINTF(Decode, "[tid:%i] Blocking.\n", tid);

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    // If the decode status is blocked or unblocking then decode has not yet
    // signalled fetch to unblock. In that case, there is no need to tell
    // fetch to block.
    if (decodeStatus[tid] != Blocked) {
        // Set the status to Blocked.
        decodeStatus[tid] = Blocked;

        if (toFetch->decodeUnblock[tid]) {
            toFetch->decodeUnblock[tid] = false;
        } else {
            toFetch->decodeBlock[tid] = true;
            wroteToTimeBuffer = true;
        }

        return true;
    }

    return false;
}

bool
Decode::unblock(ThreadID tid)
{
    // Decode is done unblocking only if the skid buffer is empty.
    if (skidBuffer[tid].empty()) {
        DPRINTF(Decode, "[tid:%i] Done unblocking.\n", tid);
        toFetch->decodeUnblock[tid] = true;
        wroteToTimeBuffer = true;

        decodeStatus[tid] = Running;
        return true;
    }

    DPRINTF(Decode, "[tid:%i] Currently unblocking.\n", tid);

    return false;
}

void
Decode::squash(const DynInstPtr &inst, ThreadID tid)
{
    DPRINTF(Decode, "[tid:%i] [sn:%llu] Squashing due to incorrect branch "
            "prediction detected at decode.\n", tid, inst->seqNum);

    // Send back mispredict information.
    toFetch->decodeInfo[tid].branchMispredict = true;
    toFetch->decodeInfo[tid].predIncorrect = true;
    toFetch->decodeInfo[tid].mispredictInst = inst;
    toFetch->decodeInfo[tid].squash = true;
    toFetch->decodeInfo[tid].doneSeqNum = inst->seqNum;
    set(toFetch->decodeInfo[tid].nextPC, inst->readPredTarg());

    // Looking at inst->pcState().branching()
    // may yield unexpected results if the branch
    // was predicted taken but aliased in the BTB
    // with a branch jumping to the next instruction (mistarget)
    // Using PCState::branching()  will send execution on the
    // fallthrough and this will not be caught at execution (since
    // branch was correctly predicted taken)
    toFetch->decodeInfo[tid].branchTaken = inst->readPredTaken() ||
                                           inst->isUncondCtrl();

    toFetch->decodeInfo[tid].squashInst = inst;

    InstSeqNum squash_seq_num = inst->seqNum;

    // Might have to tell fetch to unblock.
    if (decodeStatus[tid] == Blocked ||
        decodeStatus[tid] == Unblocking) {
        toFetch->decodeUnblock[tid] = 1;
    }

    // Set status to squashing.
    decodeStatus[tid] = Squashing;

    for (int i=0; i<fromFetch->size; i++) {
        if (fromFetch->insts[i]->threadNumber == tid &&
            fromFetch->insts[i]->seqNum > squash_seq_num) {
            fromFetch->insts[i]->setSquashed();
        }
    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    while (!insts[tid].empty()) {
        insts[tid].pop();
    }

    while (!skidBuffer[tid].empty()) {
        skidBuffer[tid].pop();
    }

    // Squash instructions up until this one
    cpu->removeInstsUntil(squash_seq_num, tid);
}

unsigned
Decode::squash(ThreadID tid)
{
    DPRINTF(Decode, "[tid:%i] Squashing.\n",tid);

    if (decodeStatus[tid] == Blocked ||
        decodeStatus[tid] == Unblocking) {
        if (FullSystem) {
            toFetch->decodeUnblock[tid] = 1;
        } else {
            // In syscall emulation, we can have both a block and a squash due
            // to a syscall in the same cycle.  This would cause both signals
            // to be high.  This shouldn't happen in full system.
            // @todo: Determine if this still happens.
            if (toFetch->decodeBlock[tid])
                toFetch->decodeBlock[tid] = 0;
            else
                toFetch->decodeUnblock[tid] = 1;
        }
    }

    // Set status to squashing.
    decodeStatus[tid] = Squashing;

    // Go through incoming instructions from fetch and squash them.
    unsigned squash_count = 0;

    for (int i=0; i<fromFetch->size; i++) {
        if (fromFetch->insts[i]->threadNumber == tid) {
            fromFetch->insts[i]->setSquashed();
            squash_count++;
        }
    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    while (!insts[tid].empty()) {
        insts[tid].pop();
    }

    while (!skidBuffer[tid].empty()) {
        skidBuffer[tid].pop();
    }

    return squash_count;
}

void
Decode::skidInsert(ThreadID tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop();

        assert(tid == inst->threadNumber);

        skidBuffer[tid].push(inst);

        DPRINTF(Decode, "Inserting [tid:%d][sn:%lli] PC: %s into decode "
                "skidBuffer %i\n", inst->threadNumber, inst->seqNum,
                inst->pcState(), skidBuffer[tid].size());
    }

    // @todo: Eventually need to enforce this by not letting a thread
    // fetch past its skidbuffer
    assert(skidBuffer[tid].size() <= skidBufferMax);
}

bool
Decode::skidsEmpty()
{
    for (ThreadID tid : *activeThreads) {
        if (!skidBuffer[tid].empty())
            return false;
    }

    return true;
}

void
Decode::updateStatus()
{
    bool any_unblocking = false;

    for (ThreadID tid : *activeThreads) {
        if (decodeStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // Decode will have activity if it's unblocking.
    if (any_unblocking) {
        if (_status == Inactive) {
            _status = Active;

            DPRINTF(Activity, "Activating stage.\n");

            cpu->activateStage(CPU::DecodeIdx);
        }
    } else {
        // If it's not unblocking, then decode will not have any internal
        // activity.  Switch it to inactive.
        if (_status == Active) {
            _status = Inactive;
            DPRINTF(Activity, "Deactivating stage.\n");

            cpu->deactivateStage(CPU::DecodeIdx);
        }
    }
}

void
Decode::sortInsts()
{
    int insts_from_fetch = fromFetch->size;
    for (int i = 0; i < insts_from_fetch; ++i) {
        insts[fromFetch->insts[i]->threadNumber].push(fromFetch->insts[i]);
    }
}

void
Decode::readStallSignals(ThreadID tid)
{
    if (fromRename->renameBlock[tid]) {
        stalls[tid].rename = true;
    }

    if (fromRename->renameUnblock[tid]) {
        assert(stalls[tid].rename);
        stalls[tid].rename = false;
    }
}

bool
Decode::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is.
    // Check stall signals, block if necessary.
    // If status was blocked
    //     Check if stall conditions have passed
    //         if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.

    // Update the per thread stall statuses.
    readStallSignals(tid);

    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(Decode, "[tid:%i] Squashing instructions due to squash "
                "from commit.\n", tid);

        squash(tid);

        return true;
    }

    if (checkStall(tid)) {
        return block(tid);
    }

    if (decodeStatus[tid] == Blocked) {
        DPRINTF(Decode, "[tid:%i] Done blocking, switching to unblocking.\n",
                tid);

        decodeStatus[tid] = Unblocking;

        unblock(tid);

        return true;
    }

    if (decodeStatus[tid] == Squashing) {
        // Switch status to running if decode isn't being told to block or
        // squash this cycle.
        DPRINTF(Decode, "[tid:%i] Done squashing, switching to running.\n",
                tid);

        decodeStatus[tid] = Running;

        return false;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause decode to change its status.  Decode remains the same as before.
    return false;
}

void
Decode::tick()
{
    wroteToTimeBuffer = false;

    bool status_change = false;

    toRenameIndex = 0;

    sortInsts();

    //Check stall and squash signals.
    for (ThreadID tid : *activeThreads) {
        DPRINTF(Decode,"Processing [tid:%i]\n",tid);
        status_change =  checkSignalsAndUpdate(tid) || status_change;

        decode(status_change, tid);
    }

    if (status_change) {
        updateStatus();
    }

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");

        cpu->activityThisCycle();
    }
}

void
Decode::decode(bool &status_change, ThreadID tid)
{
    // If status is Running or idle,
    //     call decodeInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from fetch
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (decodeStatus[tid] == Blocked) {
        ++stats.blockedCycles;
    } else if (decodeStatus[tid] == Squashing) {
        ++stats.squashCycles;
    }

    // Decode should try to decode as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (decodeStatus[tid] == Running ||
        decodeStatus[tid] == Idle) {
        DPRINTF(Decode, "[tid:%i] Not blocked, so attempting to run "
                "stage.\n",tid);

        decodeInsts(tid);
    } else if (decodeStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        decodeInsts(tid);

        if (fetchInstsValid()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        status_change = unblock(tid) || status_change;
    }
}


void
Decode::pushTmpHistoryToBP(const DynInstPtr& dynInst)
{
    using namespace gem5::branch_prediction;
    if (dynInst->staticInst == nopStaticInstPtr)
    {
        return;
    }

    bool taken = false;
    auto br_type = getBranchType(dynInst->staticInst);
    BPredUnit::PredictorHistory* bp_history =
        static_cast<BPredUnit::PredictorHistory*>(dynInst->tmpBPHistory[0]);
    assert(!dynInst->tmpBPHistory[1]);
    // 如果 bp_history 不为空，并且 btb 预测正确，则将 history 压入
    // branch predictor 中
    if (bp_history)
    {
        assert(br_type != enums::NoBranch);
        assert(bp_history->btbHit);
        auto& hist_queue = branchPred->predHist[dynInst->threadNumber];
        if (!hist_queue.empty())
        {
            assert(hist_queue.front()->seqNum < dynInst->seqNum);
            assert(hist_queue.front()->hist_id < bp_history->hist_id);
        }
        hist_queue.push_front(bp_history);

        if (bp_history->predTaken)
        {
            dynInst->setPredTarg(*bp_history->target);
            taken = true;
        }
    }
    // 在 not taken 的情况下，bp_history 的 target 不一定正确，因为在预测的时候
    // 没有指令信息，不知道指令的长度
    if (!taken)
    {
        dynInst->setPredTarg(dynInst->pcState());
        auto& inst = dynInst->staticInst;
        inst->advancePC(*dynInst->predPC);
        auto pc = dynInst->pcState().instAddr();
        auto last_inst = !inst->isMicroop() || inst->isLastMicroop();
        if (last_inst)
        {
            assert(dynInst->predPC->instAddr() == pc + inst->size());
        }
        else
        {
            assert(dynInst->predPC->instAddr() == pc);
        }
    }
    dynInst->setPredTaken(taken);

    // 清空 tmp bp history
    dynInst->tmpBPHistory[0] = nullptr;
    dynInst->tmpBPHistory[1] = nullptr;
}

bool
Decode::checkInstBP(const DynInstPtr& dynInst, PCStateBase& taken_target)
{
    using namespace gem5::branch_prediction;

    // page fault 时会产生 nop inst, 它没有设置 inst size
    if (dynInst->staticInst == nopStaticInstPtr)
    {
        assert(dynInst->tmpBPHistory[0] == nullptr);
        assert(dynInst->tmpBPHistory[1] == nullptr);
        return false;
    }
    if (dynInst->tmpBPHistory[0])
    {
        DPRINTF(Special, "decode predict, inst: %s, pc: %lx, taken: %d\n",
                dynInst->staticInst->getName(),
                dynInst->pcState().instAddr(),
                static_cast<BPredUnit::PredictorHistory*>(
                    dynInst->tmpBPHistory[0])->predTaken);
    }
    if (dynInst->tmpBPHistory[1])
    {
        DPRINTF(Special, "decode predict 2, inst: %s, pc: %lx, taken: %d\n",
                dynInst->staticInst->getName(),
                dynInst->pcState().instAddr(),
                static_cast<BPredUnit::PredictorHistory*>(
                    dynInst->tmpBPHistory[1])->predTaken);
    }

    auto br_type = getBranchType(dynInst->staticInst);
    bool mis_match = false;
    bool bp1_should_valid = br_type != enums::NoBranch;
    bool bp1_valid = dynInst->tmpBPHistory[0];
    bool bp2_should_valid = false;
    bool bp2_valid = dynInst->tmpBPHistory[1];

    bool taken = false;
    Addr bp_target = 0;

    // RISC-V 的 2 bytes 指令至多包含一个 bp history
    bool compress = dynInst->staticInst->size() == 2;
    assert(!compress || !bp2_valid);

    // flush 的原因可能是 btb mismatch，或者 pred taken 预测的地址不对
    // btb mismatch
    if (bp2_valid != bp2_should_valid)
    {
        mis_match = true;
    }
    else if (bp1_valid != bp1_should_valid)
    {
        if (branchPred->takenOnlyHistory && br_type == enums::DirectCond)
        {
            // btb miss conditional branch，认为是 not taken，不视为 mis match
        }
        else
        {
            mis_match = true;
        }
    }
    else if (bp1_valid || bp2_valid)
    {
        // bp history 的位置正确，检查 branch type 是否匹配
        BPredUnit::PredictorHistory* bp_hist = nullptr;
        if (bp1_valid)
        {
            bp_hist = static_cast<BPredUnit::PredictorHistory*>(
                dynInst->tmpBPHistory[0]);
        }
        else
        {
            bp_hist = static_cast<BPredUnit::PredictorHistory*>(
                dynInst->tmpBPHistory[1]);
        }
        assert(bp_hist);
        // btb mismatch
        if (bp_hist->type != br_type)
        {
            mis_match = true;
        }
        taken = bp_hist->condPred;
        bp_target = bp_hist->target->instAddr();
    }
    else
    {
        // 说明是非 branch 指令，并且 bp history 为空，不需要处理
    }

    // 对于 pc 相对跳转的指令，我们可以在译码阶段拿到真实的跳转目标地址
    if (dynInst->isDirectCtrl())
    {
        set(taken_target, *dynInst->branchTarget());
    }
    // 如果此时 mis_match 为 false，说明 btb 正确预测了指令类型
    if (!mis_match)
    {
        taken |= dynInst->isUncondCtrl();
        if (taken)
        {
            // 检查 taken 的 pc 相对跳转的地址是否正确
            if (dynInst->isDirectCtrl())
            {
                mis_match = taken_target.instAddr() != bp_target;
            }
        }
        // 如果是因为 br target mismatch，则不可能是间接跳转指令
        // 因为间接跳转指令的跳转目标是使用 ras 或者 indirect predictor 获得的
        assert(!mis_match || dynInst->isDirectCtrl());
    }

    return mis_match;
}

void
Decode::decodeInsts(ThreadID tid)
{
    // Instructions can come either from the skid buffer or the list of
    // instructions coming from fetch, depending on decode's status.
    int insts_available = decodeStatus[tid] == Unblocking ?
        skidBuffer[tid].size() : insts[tid].size();

    if (insts_available == 0) {
        DPRINTF(Decode, "[tid:%i] Nothing to do, breaking out"
                " early.\n",tid);
        // Should I change the status to idle?
        ++stats.idleCycles;
        return;
    } else if (decodeStatus[tid] == Unblocking) {
        DPRINTF(Decode, "[tid:%i] Unblocking, removing insts from skid "
                "buffer.\n",tid);
        ++stats.unblockCycles;
    } else if (decodeStatus[tid] == Running) {
        ++stats.runCycles;
    }

    std::queue<DynInstPtr>
        &insts_to_decode = decodeStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    DPRINTF(Decode, "[tid:%i] Sending instruction to rename.\n",tid);

    while (insts_available > 0 && toRenameIndex < decodeWidth) {
        assert(!insts_to_decode.empty());

        DynInstPtr inst = std::move(insts_to_decode.front());

        insts_to_decode.pop();

        DPRINTF(Decode, "[tid:%i] Processing instruction [sn:%lli] with "
                "PC %s\n", tid, inst->seqNum, inst->pcState());

        if (inst->isSquashed()) {
            DPRINTF(Decode, "[tid:%i] Instruction %i with PC %s is "
                    "squashed, skipping.\n",
                    tid, inst->seqNum, inst->pcState());

            ++stats.squashedInsts;

            --insts_available;

            continue;
        }

        // Also check if instructions have no source registers.  Mark
        // them as ready to issue at any time.  Not sure if this check
        // should exist here or at a later stage; however it doesn't matter
        // too much for function correctness.
        if (inst->numSrcRegs() == 0) {
            inst->setCanIssue();
        }

        // This current instruction is valid, so add it into the decode
        // queue.  The next instruction may not be valid, so check to
        // see if branches were predicted correctly.
        toRename->insts[toRenameIndex] = inst;

        ++(toRename->size);
        ++toRenameIndex;
        ++stats.decodedInsts;
        --insts_available;

#if TRACING_ON
        if (debug::O3PipeView) {
            inst->decodeTick = curTick() - inst->fetchTick;
        }
#endif

        // Ensure that if it was predicted as a branch, it really is a
        // branch.
        std::unique_ptr<PCStateBase> taken_target(inst->pcState().clone());
        bool mis_pred = checkInstBP(inst, *taken_target);
        if (mis_pred)
        {
            // 清除所有的 tmp history
            cpu->clearTmpBPHistory(inst->seqNum, tid);
            auto pc = inst->pcState().instAddr();
            if (inst->isIndirectCtrl())
            {
                set(taken_target, inst->pcState());
                branchPred->lookupIndirect(inst->staticInst, inst->seqNum,
                    *taken_target, tid);
            }
            else if (inst->isDirectCtrl())
            {
                assert(taken_target->instAddr() ==
                    inst->branchTarget()->instAddr());
            }
            else
            {
                // 如果不是 branch，那么不需要 taken target
                auto br_type = branch_prediction::getBranchType(
                    inst->staticInst);
                assert(br_type == enums::NoBranch);
            }
            // 修正 btb 表项
            branchPred->btbFixFromDecode(inst->staticInst, inst->seqNum,
                *taken_target, pc, tid);
            // 重新预测
            std::unique_ptr<PCStateBase> predict_pc(inst->pcState().clone());
            auto br_type = branch_prediction::getBranchType(inst->staticInst);
            bool taken = false;
            if (br_type != enums::NoBranch)
            {
                taken = branchPred->predict(inst->staticInst, inst->seqNum,
                    *predict_pc, tid);
                auto* hist = branchPred->predHist[tid].front();
                assert(hist->btbHit);
                hist->btbHit = false;
            }
            else
            {
                inst->staticInst->advancePC(*predict_pc);
            }
            if (taken)
            {
                assert(*taken_target == *predict_pc);
            }
            else
            {
                // 因为都是针对普通指令或者 macro 指令中最后
                // 一条 micro 指令进行预测的，因此不跳转的话新的
                // pc 应该是取指 pc + 指令长度
                assert(predict_pc->instAddr() ==
                    pc + inst->staticInst->size());
            }
            inst->setPredTaken(taken);
            inst->setPredTarg(*predict_pc);
            squash(inst, inst->threadNumber);
            break;
        }
        else
        {
            // 如果 btb 预测正确，将 tmp history 压入 BP
            pushTmpHistoryToBP(inst);
        }
    }

    // If we didn't process all instructions, then we will need to block
    // and put all those instructions into the skid buffer.
    if (!insts_to_decode.empty()) {
        block(tid);
    }

    // Record that decode has written to the time buffer for activity
    // tracking.
    if (toRenameIndex) {
        wroteToTimeBuffer = true;
    }
}

} // namespace o3
} // namespace gem5
