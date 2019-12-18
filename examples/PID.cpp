/*!
 * Example using a PID controller.
 * \example PID.cpp
 */

#include <libpid/pid.h>

#include <iostream>

int main(int argc, char** argv) {
    // Create a PID controller object
    double kp = 1;
    double ki = 0;
    double kd = 0;
    double max_integral_error = 0;
    double error_margin = 0;

    pid::pidController<double> testPID(kp, ki, kd, max_integral_error,
                                       error_margin);

    // Test pid functionality
    double control_one = testPID.computeControl(0, 50, 0);
    double control_two = testPID.computeControl(0, 25, 1);

    std::cout << control_one << std::endl;
    std::cout << control_two << std::endl;
}
