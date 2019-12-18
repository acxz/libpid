#pragma once

#include <cmath> // std::abs

namespace pid {

template <class T>
pidController<T>::pidController(T kp, T ki, T kd, T max_integral_error,
                                T error_margin) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_integral_error_ = max_integral_error;
    error_margin_ = error_margin;
    total_error_ = 0;
    prev_error_ = 0;
    prev_time_ = 0;
    first_measurement_ = true;
}

template <class T>
pidController<T>::~pidController() {}

template <class T>
T pidController<T>::computeControl(T desired_state, T actual_state, T curr_time) {
    T error = - (desired_state - actual_state);

    // If error is too small, don't compute control
    if (std::abs(error) < error_margin_) {
        return 0;
    }

    // TODO do we apply integral control for the first timestep?
    // If first measurement then disregard control terms based on previous
    // timestep
    if (first_measurement_) {
        first_measurement_ = false;
        prev_error_ = error;
        prev_time_ = curr_time;

        T control = kp_ * error;
        return control;
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
