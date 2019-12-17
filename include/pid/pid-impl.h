#pragma once

namespace pid {

template <class T>
pidController<TODO>::pidController(T kp, T ki, T kd, T max_integral_error,
                                   T error_margin) {
    T kp_ = kp;
    T ki_ = ki;
    T kd_ = kd;
    T max_integral_error_ = max_integral_error;
    T error_margin_ = error_margin;
    T total_error_ = 0;
    T prev_error_ = 0;
    T prev_time_ = 0;
    bool first_measurement_ = True;
}

template <class T>
pidController<TODO>::~pidController() {}

template <class T>
pidController<TODO>::computeControl(T desired_state, T actual_state,
                                    T curr_time) {
    T error = -(desired_state - actual_state);

    // If first measurement then disregard control terms based on previous
    // timestep
    if (first_measurement_) {
        first_measurement_ = False;
        prev_error_ = error;
        prev_time_ = curr_time;
        return 0;
    }

    // If error is too small, don't compute control
    if (abs(error) < error_margin_) {
        return 0;
    }

    total_error_ = total_error_ + error * (curr_time - prev_time_);

    // Reset total error when max_integral_error reached
    if (total_error_ > max_integral_error_) {
        total_error_ = 0;
    }

    T error_rate = (error - prev_error_) * (curr_time - prev_time_);
    T control = kp_ * error + ki_ * total_error_ + kd_ * error_rate;

    prev_error_ = error;
    prev_time_ = curr_time;

    return control;
}

}  // namespace pid
