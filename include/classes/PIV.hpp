#ifndef PIV_H
#define PIV_H

#define IIR_PROB 0.03

#include <cmath>
#include <vector>
class PIV {
  public:
    double kp;
    double ki;
    double kv;
    double error = 0;
    double prev_error = 0;
    double integral = 0;

    double error_velo = 0;

    double filter_velo = 0;

    PIV(double P, double I, double V) {
      kp = P;
      ki = I;
      kv = V;
    }

    double Calculate(double target, double target_velo, double input, double limit, double max) {
      prev_error = error;
      error = target - input;

      std::abs((int)error) < limit? integral += error : integral = 0;

      integral >= 0? integral = fmin(integral, max): integral = fmax(integral, -max);

      error_velo = target_velo - filter_velo;

      return kp*error + ki *integral + kv*error_velo;
    }
};

#endif
