#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "../custom/config.hpp"

using Position = std::vector<double>;

class Odometry {
  public:
    std::vector<double> state = {0,0,0};
    double prev_theta = 0;
    double enc_avg = 0;
    double prev_enc_avg = 0;
    double imu_offset = 0;
    double init_angle = 0;
    Odometry(double x_, double y_, double theta_) {
      //initialise worlds
      state[X] = x_;
      state[Y] = y_;
      state[THETA] = theta_;
      imu_offset = -imu.get_yaw();
      init_angle = theta_;
    }

    void calculate_state() {
      prev_theta = state[THETA];
      //state[THETA] = 0.8 * (-imu.get_yaw() - imu_offset) + 0.2 *  (enc_l.get_value() - enc_r.get_value())/WHEELBASE * RAD_TO_DEG/2 + init_angle; //average enc and gyro
      state[THETA] = (-imu.get_yaw() - imu_offset) + init_angle;
      //may need weighted average

      prev_enc_avg = enc_avg;
      enc_avg = (enc_l.get_value() + enc_r.get_value())/2;

      double d_theta = state[THETA] - prev_theta;

      state[X] += (enc_avg-prev_enc_avg) * cos((prev_theta  + d_theta/2) * DEG_TO_RAD);
      state[Y] += (enc_avg-prev_enc_avg) * sin((prev_theta + d_theta/2) * DEG_TO_RAD);
    }//state is x y and theta


};

#endif
