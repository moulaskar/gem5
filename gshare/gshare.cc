/*
 * Copyright (c) 2011, 2014 ARM Limited
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
 *
 * Authors: Kevin Lim
 */

#include "cpu/pred/gshare.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "debug/GshareDEBUG.hh"
GshareBP::GshareBP(const GshareBPParams *params)
    : BPredUnit(params),
      globalHistory(params->numThreads, 0),
      globalHistoryBits(params->globalPredictorSize),
      choicePredictorSize(params->PHTPredictorSize),
      choiceCtrBits(params->PHTCtrBits)
{

    if (!isPowerOf2(choicePredictorSize)) {
        fatal("Invalid choice predictor size!\n");
    }

    // Set up choiceHistoryMask
    // this is equivalent to mask(log2(choicePredictorSize)
    choiceHistoryMask = choicePredictorSize - 1;

    //Setup the array of counters for the choice predictor
    choiceCtrs.resize(choicePredictorSize);

    for (int i = 0; i < choicePredictorSize; ++i)
        choiceCtrs[i].setBits(choiceCtrBits);
    
    //Set up historyRegisterMask
    historyRegisterMask = mask(globalHistoryBits);


    // Set thresholds for the predictors' counters
    // This is equivalent to (2^(Ctr))/2 - 1
    choiceThreshold = (ULL(1) << (choiceCtrBits - 1)) - 1;
}

inline
unsigned
GshareBP::calcLocHistIdx(Addr &branch_addr)
{
    // Get low order bits after removing instruction offset.
    return (branch_addr >> instShiftAmt) & historyRegisterMask;
}

inline
void
GshareBP::updateGlobalHistTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1) | 1;
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
GshareBP::updateGlobalHistNotTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1);
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

void
GshareBP::btbUpdate(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    globalHistory[tid] &= (historyRegisterMask & ~ULL(1));
}

bool
GshareBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    unsigned local_history_idx;
   // bool global_prediction;
    bool choice_prediction;

    //Lookup to get its branch 
    local_history_idx = calcLocHistIdx(branch_addr);

    
    unsigned choice_prediction_idx = (globalHistory[tid] & historyRegisterMask) ^ local_history_idx;

    //Lookup in the choice predictor to see which one to use
    choice_prediction = choiceThreshold <
      choiceCtrs[choice_prediction_idx & choiceHistoryMask].read();

    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid] & historyRegisterMask;
    bp_history = (void *)history;

    if (choice_prediction) 
    {
         updateGlobalHistTaken(tid);
         return true;
     } 
     else 
     {
         updateGlobalHistNotTaken(tid);
         return false;
     }

}

void
GshareBP::uncondBranch(ThreadID tid, Addr pc, void * &bp_history)
{

    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid] & historyRegisterMask;
    bp_history = static_cast<void *>(history);

    updateGlobalHistTaken(tid);
}

void
GshareBP::update(ThreadID tid, Addr branch_addr, bool taken,
                     void *bp_history, bool squashed)
{
    assert(bp_history);

    BPHistory *history = static_cast<BPHistory *>(bp_history);

    unsigned local_history_idx = calcLocHistIdx(branch_addr);

    if (squashed) {
        // Global history restore and update
        globalHistory[tid] = (history->globalHistory << 1) | taken;
        globalHistory[tid] &= historyRegisterMask;

        return;
    }

    // there was a prediction.
    unsigned choice_predictor_idx = (local_history_idx ^ history->globalHistory) & choiceHistoryMask;
    if(taken)
    {
        choiceCtrs[choice_predictor_idx].increment();
    }
    else
    {
        choiceCtrs[choice_predictor_idx].decrement();
    }

    // We're done with this history, now delete it.
    delete history;
}

void
GshareBP::squash(ThreadID tid, void *bp_history)
{

    BPHistory *history = static_cast<BPHistory *>(bp_history);

    // Restore global history to state prior to this branch.
    globalHistory[tid] = history->globalHistory;

    delete history;
}

GshareBP*
GshareBPParams::create()
{
    return new GshareBP(this);
}

unsigned
GshareBP::getGHR(ThreadID tid, void *bp_history) const
{
    return static_cast<BPHistory *>(bp_history)->globalHistory;
}

#ifdef DEBUG
int
GshareBP::BPHistory::newCount = 0;
#endif
