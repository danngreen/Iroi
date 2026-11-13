#pragma once

#include "Commons.h"
#include "Ambience.h"
#include "Filter.h"
#include "Resonator.h"
#include "Echo.h"
#include "Schmitt.h"
#include "TGate.h"
#include "EnvFollower.h"
#include "DcBlockingFilter.h"
#include "SmoothValue.h"
#include "Modulation.h"

class Iroi
{
private:
    PatchCtrls* patchCtrls_;
    PatchCvs* patchCvs_;
    PatchState* patchState_;

    Filter* filter_;
    Resonator* resonator_;
    Echo* echo_;
    Ambience* ambience_;
    
    Modulation* modulation_;

    DcBlockingFilter inputDcFilter_[2];
    StereoDcBlockingFilter* outputDcFilter_;

    EnvFollower* inEnvFollower_[2];
    EnvFollower* outEnvFollower_[2];

    FilterPosition filterPosition_, lastFilterPosition_;

    bool bypass_;

    // Per-block state for per-sample processing, set by BeginBlock()
    size_t blockPos_ = 0;
    float outGain_ = 0.f;

    void UpdateFilterPosition()
    {
        if (patchCtrls_->filterPosition < 0.25f)
        {
            filterPosition_ = FilterPosition::POSITION_1;
        }
        else if (patchCtrls_->filterPosition >= 0.25f && patchCtrls_->filterPosition < 0.5f)
        {
            filterPosition_ = FilterPosition::POSITION_2;
        }
        else if (patchCtrls_->filterPosition >= 0.5f && patchCtrls_->filterPosition < 0.75f)
        {
            filterPosition_ = FilterPosition::POSITION_3;
        }
        else if (patchCtrls_->filterPosition >= 0.75f)
        {
            filterPosition_ = FilterPosition::POSITION_4;
        }

        if (filterPosition_ != lastFilterPosition_)
        {
            lastFilterPosition_ = filterPosition_;
            patchState_->filterPositionFlag = true;
        }
        else
        {
            patchState_->filterPositionFlag = false;
        }
    }

public:
    Iroi(PatchCtrls* patchCtrls, PatchCvs* patchCvs, PatchState* patchState)
    {
        patchCtrls_ = patchCtrls;
        patchCvs_ = patchCvs;
        patchState_ = patchState;

        filter_ = Filter::create(patchCtrls_, patchCvs_, patchState_);
        resonator_ = Resonator::create(patchCtrls_, patchCvs_, patchState_);
        echo_ = Echo::create(patchCtrls_, patchCvs_, patchState_);
        ambience_ = Ambience::create(patchCtrls_, patchCvs_, patchState_);

        modulation_ = Modulation::create(patchCtrls_, patchCvs_, patchState_);

        for (size_t i = 0; i < 2; i++)
        {
            inEnvFollower_[i] = EnvFollower::create();
            inEnvFollower_[i]->setLambda(0.9f);
            outEnvFollower_[i] = EnvFollower::create();
            outEnvFollower_[i]->setLambda(0.9f);
        }

        outputDcFilter_ = StereoDcBlockingFilter::create();

        filterPosition_ = FilterPosition::POSITION_1;
        lastFilterPosition_ = FilterPosition::POSITION_1;
        bypass_ = false;
    }
    ~Iroi()
    {
        Filter::destroy(filter_);
        Resonator::destroy(resonator_);
        Echo::destroy(echo_);
        Ambience::destroy(ambience_);
        Modulation::destroy(modulation_);

        for (size_t i = 0; i < 2; i++)
        {
            EnvFollower::destroy(outEnvFollower_[i]);
            EnvFollower::destroy(inEnvFollower_[i]);
        }
    }

    static Iroi* create(PatchCtrls* patchCtrls, PatchCvs* patchCvs, PatchState* patchState)
    {
        return new Iroi(patchCtrls, patchCvs, patchState);
    }

    static void destroy(Iroi* obj)
    {
        delete obj;
    }

    inline void Process(AudioBuffer &buffer)
    {
        if (bypass_)
        {
            return;
        }

        FloatArray left = buffer.getSamples(LEFT_CHANNEL);
        FloatArray right = buffer.getSamples(RIGHT_CHANNEL);

        const size_t size = buffer.getSize();

        for (size_t i = 0; i < size; i++)
        {
            patchState_->inputLevel[i] = Mix2(inEnvFollower_[0]->process(left[i]), inEnvFollower_[1]->process(right[i]));
        }

        inputDcFilter_[LEFT_CHANNEL].process(left);
        inputDcFilter_[RIGHT_CHANNEL].process(right);

        modulation_->Process();

        UpdateFilterPosition();

        if (FilterPosition::POSITION_1 == filterPosition_)
        {
            filter_->process(buffer, buffer);
        }
        resonator_->process(buffer, buffer);
        if (FilterPosition::POSITION_2 == filterPosition_)
        {
            filter_->process(buffer, buffer);
        }
        echo_->process(buffer, buffer);
        if (FilterPosition::POSITION_3 == filterPosition_)
        {
            filter_->process(buffer, buffer);
        }
        ambience_->process(buffer, buffer);
        if (FilterPosition::POSITION_4 == filterPosition_)
        {
            filter_->process(buffer, buffer);
        }

        buffer.multiply(kOutputMakeupGain * patchState_->outLevel);

        // Level LED.
        for (size_t i = 0; i < size; i++)
        {
            patchState_->outputLevel[i] = Mix2(outEnvFollower_[0]->process(left[i]), outEnvFollower_[1]->process(right[i]));
        }
    }

    // Per-sample processing: the same signal path as Process(), with the control-rate
    // work done once per block in BeginBlock() and the audio done one sample at a time.
    // Each block is BeginBlock(), then patchState->blockSize calls to ProcessSample(),
    // then EndBlock().
    //
    // The modulation's envelope follower reads the input level of the block before,
    // since the block's own input hasn't arrived yet when BeginBlock() runs.
    inline void BeginBlock()
    {
        blockPos_ = 0;

        if (bypass_)
        {
            return;
        }

        modulation_->Process();

        UpdateFilterPosition();

        const size_t size = patchState_->blockSize;
        filter_->prepare(size);
        resonator_->prepare(size);
        echo_->prepare(size);
        ambience_->prepare(size);

        outGain_ = kOutputMakeupGain * patchState_->outLevel;
    }

    inline void ProcessSample(float &left, float &right)
    {
        if (bypass_)
        {
            return;
        }

        const float inLevel = Mix2(inEnvFollower_[0]->process(left), inEnvFollower_[1]->process(right));

        left = inputDcFilter_[LEFT_CHANNEL].process(left);
        right = inputDcFilter_[RIGHT_CHANNEL].process(right);

        if (FilterPosition::POSITION_1 == filterPosition_)
        {
            filter_->processSample(left, right);
        }
        resonator_->processSample(left, right);
        if (FilterPosition::POSITION_2 == filterPosition_)
        {
            filter_->processSample(left, right);
        }
        echo_->processSample(left, right);
        if (FilterPosition::POSITION_3 == filterPosition_)
        {
            filter_->processSample(left, right);
        }
        ambience_->processSample(left, right);
        if (FilterPosition::POSITION_4 == filterPosition_)
        {
            filter_->processSample(left, right);
        }

        left *= outGain_;
        right *= outGain_;

        // Level LED.
        const float outLevel = Mix2(outEnvFollower_[0]->process(left), outEnvFollower_[1]->process(right));

        if (blockPos_ < patchState_->inputLevel.getSize())
        {
            patchState_->inputLevel[blockPos_] = inLevel;
            patchState_->outputLevel[blockPos_] = outLevel;
            blockPos_++;
        }
    }

    inline void EndBlock()
    {
        if (bypass_)
        {
            return;
        }

        echo_->finish();
        ambience_->finish();
    }
};
