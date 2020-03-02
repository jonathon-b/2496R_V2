#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H
#include "../custom/config.hpp"

class Motion_Profile{
  private:
    double max_velo;
    double max_accel;
  public:
    Motion_Profile(double v, double a) {//trapezoidal construction
      max_velo = v;
      max_accel = a;
    }

    //1D motion profile
    void trap_1d(double target_dist, double &projected_pos, double &current_v, double &current_a) {
      double ramp_distance = max_velo * max_velo / 2 / max_accel;
      if(projected_pos < ramp_distance) {
        projected_pos += current_v * DT * MS_TO_S + 0.5 * max_accel * DT * DT * MS_TO_S * MS_TO_S;
        current_v += max_accel * DT * MS_TO_S;
        current_v = std::fmin(max_velo, current_v);
        current_a = max_accel;
      }
      else if (projected_pos < target_dist - ramp_distance) {
        projected_pos += current_v * DT * MS_TO_S;
        current_v = max_velo;
        current_a = 0;
      }
      else {

        projected_pos += current_v * DT * MS_TO_S - 0.5 * max_accel * DT * DT * MS_TO_S * MS_TO_S;
        projected_pos = std::fmin(projected_pos, target_dist);
        current_v -= max_accel * DT * MS_TO_S;
        current_v = std::fmax(current_v, 0.0);
        if(current_v == 0){
          current_a = 0;
          projected_pos = target_dist;
        }
        else {
           current_a = -max_accel;
        }
      }
    }

    void traj_profile(std::vector<double>traj, double &current_v, double &current_a) {
      current_a = traj[A];
      current_v += traj[A] * DT * MS_TO_S;
      if(traj[A] > 0) {
        current_v = std::fmin(current_v, traj[V]);
      }
      else if (traj[A] < 0) {
        current_v = std::fmax(current_v, traj[V]);
      }
    }

    double calculate_path_curvature(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3) {
      //does big math to find a circle fitting those three points then calculates curvature given radius
      //math will be in documentation for this code.
      //math will find radius and 1/radius finds curvature
      double k1 = 0.5 * (p1[X] * p1[X] + p1[Y] * p1[Y] - p2[X] * p2[X] - p2[Y] * p2[Y])/(p1[X]-p2[X]);
      double k2 = (p1[Y] - p2[Y])/(p1[X] - p2[X]);

      double center_y = 0.5 * (p2[X] * p2[X] - 2 * p2[X] * k1 + p2[Y] *
         p2[Y] - p3[X] * p3[X] + 2 * p3[X] * k1 - p3[Y] * p3[Y]) /
         (p3[X] * k2 - p3[Y] + p2[Y] - p2[X] * k2); //big sad

      double center_x = k1 - k2 * center_y;

      return 1 / sqrt( pow(p1[X] - center_x, 2) + pow(p1[Y]-center_y,2) ); // curvature = 1/radius
    }

    //this method should take the path you give it and then add all of the appropriate velocity and acceleration values
    std::vector<std::vector<double>> inject_trapezoid(std::vector<std::vector<double>> path_) {
      //the way this algorithm works is shown in motion profile explanation for 2-dimensional paths
     
      std::vector<std::vector<double>> path = path_;
      double curvature = 0;
      double delta_x = 0;
      for(int i = 0; i < path.size(); i++) { //calculating forward acceleration which is the forward "ramp"
        if(i == 0) {
          path[i][V] = 0;
          path[i][A] = max_accel;
        }
        else if(i == path.size()-1) {
          path[i][V] = 0;
          path[i][A] = -max_accel;
        }
        else { //adds the second-iteration to make sure robot can have a balance of translational speed and rotational speed
          curvature = calculate_path_curvature(path[i-1], path[i], path[i+1]);
          delta_x = hypot(path[i-1][X] - path[i][X], path[i-1][Y] - path[i][Y]);
          path[i][V] = std::fmin(1/curvature, sqrt(path[i-1][V] * path[i-1][V] + 2 * max_accel * delta_x) );
          path[i][V] = std::fmin(path[i][V], max_velo);
        }
      }

      for(int i = path.size()-1; i >= 0; i--) {//calculating backward acceleration which is the backward "ramp"
        if(i != 0 && i != path.size()-1) {
          delta_x = hypot(path[i+1][X] - path[i][X], path[i+1][Y] - path[i][Y]);
          path[i][V] = std::fmin(path[i][V], sqrt(path[i+1][V] * path[i+1][V] + 2 * max_accel * delta_x) );

          if(path[i+1][V] - path[i][V] < 0) path[i][A] = -max_accel;
          else if (path[i+1][V] - path[i][V] > 0) path[i][A] = max_accel;
          else path[i][A] = 0;
        }
      }

      return path;
    }

};

#endif
