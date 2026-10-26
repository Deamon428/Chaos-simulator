// Two-state settle detector: SWINGING until the bob stays quiet for
// settle_window_s, then SETTLED (a late hard bounce flips it back).
#pragma once

#include "config.h"

class SettleDetector {
public:
    enum State { SWINGING, SETTLED };

    explicit SettleDetector(const SetupConfig& cfg) : cfg_(cfg) {}

    // dt: seconds since the previous speed sample; speed: px/s.
    void update(double dt, double speed);
    void reset();

    State state() const { return state_; }
    double quietFor() const { return quiet_s_; }

private:
    SetupConfig cfg_;
    State state_ = SWINGING;
    double quiet_s_ = 0.0;
};
