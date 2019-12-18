#pragma once

namespace pid {

template <class T>
class pidController {
   public:
    //! constructor
    pidController(T kp, T ki, T kd, T max_integral_error, T error_margin);

    //! destructor
    ~pidController();

    //! compute a control signal
    T computeControl(T desired_state, T actual_state, T curr_time);

   private:
    T kp_;
    T ki_;
    T kd_;
    T max_integral_error_;
    T error_margin_;
    T total_error_;
    T prev_error_;
    T prev_time_;
    bool first_measurement_;
};

}  // namespace pid
