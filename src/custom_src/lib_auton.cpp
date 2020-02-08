#include "custom/lib_auton.hpp"
#include "custom/config.hpp"
#include "classes/Motion_Profile.hpp"
#include "classes/Odometry.hpp"
#include "classes/Spline.hpp"
#include "classes/PurePursuit.hpp"
#define KVFF_FWD 7.3529
#define KAFF_FWD 2.5
#define KVFF_INT_FWD 1500.0

#define KP_FWD 60
#define KI_FWD 0.2
#define KV_FWD 15
#define LIMIT_FWD 100
#define MAX_FWD 500

#define KVFF_TURN 22.5339
#define KAFF_TURN 5.5
#define KVFF_INT_TURN 1459.96

#define KP_TURN 200
#define KI_TURN 0.8
#define KD_TURN 0
#define LIMIT_TURN 20
#define MAX_TURN 2000.0


void volt_chas(double l_pwr, double r_pwr) {
  mtr_chasBL.move_voltage(l_pwr);
  mtr_chasFL.move_voltage(l_pwr);
  mtr_chasBR.move_voltage(r_pwr);
  mtr_chasFR.move_voltage(r_pwr);
}

double feedforward(double value, double gain, double intercept) {
  return value * gain + intercept;
}

double a = 0, b = 0, d = 0, e = 0, f = 0, g = 0; //for printing
void print(void *param) {
  while(true) {
    printf("encoder_avg: %.2f   projected_pos: %.2f     projected_velo: %.2f      projected_accel: %.2f     voltage: %.2f     ang_velo: %.2f\n", f, b, d, e, a, g);
    delay(50);
  }
}

void forward(double target, double cruise_v, double accel, bool reverse) {
  PIV chassis_piv(KP_FWD, KI_FWD, KV_FWD);
  PID turn_pid(KP_TURN, 0, KD_TURN);
  Motion_Profile profile(std::abs(cruise_v),std::abs(accel));
  double projected_pos = 0, projected_velo = 0, projected_accel = 0;

  double voltage = 0;
  double velocity = 0;
  double prev_encoder_avg = 0;
  double encoder_avg = 0;

  double imu_offset = imu.get_yaw();
  double angle = 0;
  double angle_volt;

  enc_l.reset();
  enc_r.reset();

  while(true) {
    prev_encoder_avg = encoder_avg;
    profile.trap_1d(target, projected_pos, projected_velo, projected_accel);

    encoder_avg = (enc_l.get_value() + enc_r.get_value())/2;

    angle = imu.get_yaw() - imu_offset;

    //chassis_piv.filter((encoder_avg - prev_encoder_avg)/DT*1000);
    chassis_piv.filter_velo = (mtr_chasFL.get_actual_velocity())*8.72727273 * 2;
    printf("POS: %.2f   proj_pos: %.2f    velo: %.2f    proj_velo: %.2f   accel: %.2f\n", encoder_avg, projected_pos, chassis_piv.filter_velo, projected_velo, projected_accel);

    if(!reverse) {
      voltage = chassis_piv.Calculate(projected_pos, projected_velo, std::abs(encoder_avg), LIMIT_FWD, MAX_FWD);
      voltage += feedforward(projected_velo, KVFF_FWD, KVFF_INT_FWD);
      voltage += feedforward(projected_accel, KAFF_FWD, 0);
    }
    else {
      voltage = -chassis_piv.Calculate(projected_pos, projected_velo, std::abs(encoder_avg), LIMIT_FWD, MAX_FWD);
      voltage -= feedforward(projected_velo, KVFF_FWD, KVFF_INT_FWD);
      voltage -= feedforward(projected_accel, KAFF_FWD, 0);
    }

    angle_volt = turn_pid.Calculate(imu_offset,angle,LIMIT_TURN, MAX_TURN);
    volt_chas(voltage + angle_volt, voltage - angle_volt);

    if(std::abs(chassis_piv.error) <= 10 && projected_pos == target) break;

    delay(15);
  }
  volt_chas(0,0);
}

void turn(double target, double cruise_v, double accel, bool reverse) {
  PID turn_pid(KP_TURN, KI_TURN, KD_TURN);
  Motion_Profile profile(std::abs(cruise_v),std::abs(accel));
  double projected_theta = 0, projected_velo = 0, projected_accel = 0;

  Task task1 (print, (void*) "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "CONTROL_TASK" );

  double voltage = 0;

  double angle_offset = imu.get_yaw();
  double angle = 0;
  double prev_angle = 0;
  while(true) {
    profile.trap_1d(target, projected_theta, projected_velo, projected_accel);
    prev_angle = angle;
    angle = imu.get_yaw() - angle_offset;

    if(!reverse) {
      voltage = turn_pid.Calculate(projected_theta, std::abs(angle), LIMIT_TURN, MAX_TURN);
      voltage += feedforward(projected_velo, KVFF_TURN, KVFF_INT_TURN);
      voltage += feedforward(projected_accel, KAFF_TURN, 0);
    }
    else {
      voltage = -turn_pid.Calculate(projected_theta, std::abs(angle), LIMIT_FWD, MAX_FWD);
      voltage -= feedforward(projected_velo, KVFF_TURN, KVFF_INT_TURN);
      voltage -= feedforward(projected_accel, KAFF_TURN, 0);
    }

//    if(std::abs(turn_pid.error) <= 1) break;
    volt_chas(voltage, -voltage);

    delay(15);
  }
}

//arcs not finished
void arc_turns(double target_angle, double radius, double cruise_v, double accel, bool reverse) {
  //basically uses purepursuit to get arc_length
  //mb use velocity pid
  double curvature = 1/radius;
  double target_dist = target_angle * DEG_TO_RAD * radius; // arc = theta * r
  double projected_pos_l = 0, projected_pos_r = 0;
  double projected_velocity = 0;
  double projected_accel = 0;


  while(true) {

    delay(15);
  }
}

int closest_index(Path path, std::vector<double> robot) {
  double dist = 900000;
  double index = 1;
  for(int i = 1; i < path.size(); i++) {

    if(hypot(path[i][X] - robot[X], path[i][Y] - robot[Y]) < dist) {
      dist = hypot(path[i][X] - robot[X], path[i][Y] - robot[Y]);
      index = i;
    }

  }

  return index;
}

Path combine_path(Path path1, Path path2) {
  Path comb;
  comb.reserve(path1.size() + path2.size());
  comb.insert(comb.end(), path1.begin(), path2.end());
  comb.insert(comb.end(), path2.begin(), path2.end());
  return comb;
}

void path_follow(Path path, double v_max, double a_max, double lookahead_radius) {
  Motion_Profile prof(v_max, a_max);
  Odometry odo(0,0,0);
  PurePursuit pursuit(lookahead_radius);
  PIV chassis_left(KP_FWD, KI_FWD, KV_FWD);
  PIV chassis_right(KP_FWD, KI_FWD, KV_FWD);
  path = prof.inject_trapezoid(path); //gives velocities and acceleration

  enc_l.reset();
  enc_r.reset();

  double v_target = 0;
  double proj_posl = 0, proj_posr = 0, proj_vl = 0, proj_vr = 0, proj_a = 0;

  int index_end, index_start;

  double prev_encl = 0, prev_encr = 0;
  double encl = 0, encr = 0;
  double vl = 0, vr = 0;
  double pwm_l = 0, pwm_r = 0;

  while(true) {


    index_end = closest_index(path, odo.state);
    index_start = index_end - 1;

    if(!pursuit.update_lk(path[index_start], path[index_end], odo.state)) {
      index_end -= 1;
      index_start -= 1;
    }

    pursuit.update_lk(path[index_start], path[index_end], odo.state);
    pursuit.find_curvature(odo.state);

    proj_a = path[index_end][A];
    v_target += proj_a * DT / 1000;
    if(proj_a >= 0) v_target = fmin(v_target, path[index_end][V]);
    else if(proj_a < 0) v_target = fmax(v_target, path[index_end][V]);

    prev_encl = encl;
    prev_encr = encr;

    encl = enc_l.get_value();
    encr = enc_r.get_value();

    proj_vl = v_target * (2 + pursuit.curvature * WHEELBASE) / 2;
    proj_vr = v_target * (2 - pursuit.curvature * WHEELBASE) / 2;

    proj_posl = encl + proj_vl * DT / 1000 + 0.5 * proj_a * 0.5 * DT * DT / 1000 / 1000;
    proj_posr = encr + proj_vr * DT / 1000 + 0.5 * proj_a * 0.5 * DT * DT / 1000 / 1000;

    //chassis_left.filter((encl - prev_encl)/DT*1000);
    //chassis_right.filter((encr - prev_encr)/DT*1000);

    pwm_l = chassis_left.Calculate(proj_posl, proj_vl, encl, LIMIT_FWD, MAX_FWD);
    pwm_r = chassis_right.Calculate(proj_posr, proj_vr, encr, LIMIT_FWD, MAX_FWD);

    pwm_l += feedforward(proj_vl, KVFF_FWD, KVFF_INT_FWD);
    pwm_r += feedforward(proj_vr, KVFF_FWD, KVFF_INT_FWD);

    pwm_l += feedforward(proj_a, KAFF_FWD, 0);
    pwm_r += feedforward(proj_a, KAFF_FWD, 0);
    delay(15);
  }
}


void pwr_intake(int pwr) {
  mtr_rollR.move(pwr);
  mtr_rollL.move(pwr);
}

void lift(int pwr, int time){
  mtr_lift.move(pwr);
  delay(time);
  mtr_lift.move(0);

}

void blue_auton(){
  pwr_intake(127);
  lift(90, 300);
  forward(300, 600, 500, false);
  forward(600, 300, 300, false);
  pwr_intake(0);
  forward(600, 400,350, true);


}
