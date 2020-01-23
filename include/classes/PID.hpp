#include "../custom/config.hpp"
#ifndef PID_H
#define PID_H

class PID {
  public:
    double kp;
    double ki;
    double kd;
    double error = 0;
    double prev_error = 0;
    double integral = 0;
    double derivative = 0;
    PID(double P, double I, double D) {
      kp = P;
      ki = I;
      kd = D;
    }

    double Calculate(double target, double input, int limit) {
      prev_error = error;
      error = target - input;
      std::abs((int)error) < limit? integral += error : integral = 0;
      derivative = error - prev_error;
      return kp*error + ki *integral + kd*derivative;
    }
};

#endif
