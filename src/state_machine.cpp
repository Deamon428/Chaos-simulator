#include "state_machine.h"

void SettleDetector::update(double dt, double speed) {
    if (state_ == SETTLED) {
        if (speed > cfg_.bounce_speed_px_s) {
            state_ = SWINGING;
            quiet_s_ = 0.0;
        }
        return;
    }
    if (speed < cfg_.settle_speed_px_s) {
        quiet_s_ += dt;
        if (quiet_s_ >= cfg_.settle_window_s) state_ = SETTLED;
    } else {
        quiet_s_ = 0.0;
    }
}

void SettleDetector::reset() {
    state_ = SWINGING;
    quiet_s_ = 0.0;
}
