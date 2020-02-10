#include "custom/lib_auton.hpp"
#include "custom/config.hpp"
#include "classes/Motion_Profile.hpp"
#include "classes/Odometry.hpp"
#include "classes/Spline.hpp"
#include "classes/PurePursuit.hpp"
#define KVFF_FWD 7.3529
#define KAFF_FWD 2.5
#define KVFF_INT_FWD 1500.0

#define KP_FWD 80
#define KI_FWD 0.2
#define KV_FWD 7
//80 0.2 7
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

#define RPM_TO_ENCPS 8.72727273 * 2


void volt_chas(double l_pwr, double r_pwr) {
  mtr_chasBL.move_voltage(l_pwr);
  mtr_chasFL.move_voltage(l_pwr);
  mtr_chasBR.move_voltage(r_pwr);
  mtr_chasFR.move_voltage(r_pwr);
}

double feedforward(double value, double gain, double intercept) {
  return value * gain + intercept;
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
    velocity = (mtr_chasFL.get_actual_velocity() + mtr_chasFR.get_actual_velocity())/2 * RPM_TO_ENCPS;
    printf("POS: %.2f   proj_pos: %.2f    velo: %.2f    proj_velo: %.2f   accel: %.2f\n", encoder_avg, projected_pos, velocity, projected_velo, projected_accel);

    if(!reverse) {
      voltage = chassis_piv.Calculate(projected_pos, projected_velo, std::abs(encoder_avg), velocity,  LIMIT_FWD, MAX_FWD);
      voltage += feedforward(projected_velo, KVFF_FWD, KVFF_INT_FWD);
      voltage += feedforward(projected_accel, KAFF_FWD, 0);
    }
    else {
      voltage = -chassis_piv.Calculate(projected_pos, projected_velo, std::abs(encoder_avg), velocity, LIMIT_FWD, MAX_FWD);
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
  Motion_Profile prof(cruise_v, accel);
  PIV chas_l(KP_FWD,KI_FWD,KV_FWD);
  PIV chas_r(KP_FWD,KI_FWD,KV_FWD);
  double curvature = 1/radius;
  double target_dist = target_angle * DEG_TO_RAD * radius; // arc = theta * r
  double projected_pl = 0, projected_pr = 0;
  double projected_vl = 0, projected_vr = 0;
  double projected_pos = 0;
  double projected_velocity = 0;
  double projected_accel = 0;
  double pwm_l = 0, pwm_r = 0;

  enc_l.reset();
  enc_r.reset();

  while(true) {
    prof.trap_1d(target_dist, projected_pos, projected_velocity, projected_accel);
    projected_vl = projected_velocity * (2 + curvature * WHEELBASE)/2;
    projected_vr = projected_velocity * (2 - curvature * WHEELBASE)/2;
    projected_pl += projected_vl * DT * MS_TO_S + 0.5 * projected_accel * DT * DT * MS_TO_S * MS_TO_S;
    projected_pr += projected_vr * DT * MS_TO_S + 0.5 * projected_accel * DT * DT * MS_TO_S * MS_TO_S;

    pwm_l = chas_l.Calculate(projected_pl, projected_vl, enc_l.get_value(), mtr_chasFL.get_actual_velocity() * RPM_TO_ENCPS, LIMIT_FWD, MAX_FWD);
    if(projected_vl > 0) pwm_l += feedforward(projected_vl, KVFF_FWD, KVFF_INT_FWD);
    else if(projected_vl < 0) pwm_l += feedforward(projected_vl, KVFF_FWD, -KVFF_INT_FWD);
    pwm_l += feedforward(projected_accel, KAFF_FWD, 0);


    pwm_r = chas_r.Calculate(projected_pr, projected_vr, enc_r.get_value(), mtr_chasFR.get_actual_velocity() * RPM_TO_ENCPS, LIMIT_FWD, MAX_FWD);
    if(projected_vr > 0) pwm_r += feedforward(projected_vr, KVFF_FWD, KVFF_INT_FWD);
    else if(projected_vr < 0) pwm_r += feedforward(projected_vr, KVFF_FWD, -KVFF_INT_FWD);
    pwm_r += feedforward(projected_accel, KAFF_FWD, 0);

    printf("projected_pl: %.2f    enc_l: %.d    projected_vl: %.2f    angle: %.2f\n", projected_pl,  enc_l.get_value(), projected_vl, (enc_l.get_value() - enc_r.get_value())/WHEELBASE*RAD_TO_DEG );

    volt_chas(pwm_l, pwm_r);
    delay(15);
  }
}

int closest_index(Path path, std::vector<double> robot, int start) {
  double dist = 900000;
  double index = 1;
  for(int i = start; i < path.size()-1; i++) {

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

void pwr_intake(int pwr) {
  mtr_rollR.move(pwr);
  mtr_rollL.move(pwr);
}


void path_follow(Path path, double v_max, double a_max, double lookahead_radius, double theta) {
  Motion_Profile prof(v_max, a_max);
  Odometry odo(0,0,theta);
  PurePursuit pursuit(lookahead_radius);
  PIV chassis_left(KP_FWD, KI_FWD, KV_FWD);
  PIV chassis_right(KP_FWD, KI_FWD, KV_FWD);
  path = prof.inject_trapezoid(path); //gives velocities and acceleration

  enc_l.reset();
  enc_r.reset();

  double v_target = 0;
  double proj_posl = 0, proj_posr = 0, proj_vl = 0, proj_vr = 0, proj_a = 0;

  int index_end = 1, index_start = 0;

  double prev_encl = 0, prev_encr = 0;
  double encl = 0, encr = 0;
  double vl = 0, vr = 0;
  double pwm_l = 0, pwm_r = 0;
  double left_velo = 0, right_velo = 0;

  while(true) {

    //get velocity of left and right wheels
    left_velo = mtr_chasFL.get_actual_velocity() * RPM_TO_ENCPS;
    right_velo = mtr_chasFR.get_actual_velocity() * RPM_TO_ENCPS;

    //calculate robot state (x,y,theta)
    odo.calculate_state();

    //find 2 points to use pure pursuit on
    index_start = closest_index(path, odo.state, index_start);
    index_end = index_start + 1;

    if(!pursuit.update_lk(path[index_start], path[index_end], odo.state)) {
      index_end -= 1;
      index_start -= 1;
    }

    //find intersection between lookahead and path
    //find curvature accordingly
    pursuit.update_lk(path[index_start], path[index_end], odo.state);
    pursuit.find_curvature(odo.state);
    //printf("x: %.2f     y: %.2f     theta: %.2f\n", odo.state[X], odo.state[Y], odo.state[THETA]);

    //assigning acceleration
    //assigning velocity target
    proj_a = path[index_end][A];
    v_target += proj_a * DT / 1000;
    if(proj_a >= 0) v_target = fmin(v_target, path[index_end][V]);
    else if(proj_a < 0) v_target = fmax(v_target, path[index_end][V]);

    //assigning position target by initial position target
    prev_encl = encl;
    prev_encr = encr;
    encl = enc_l.get_value();
    encr = enc_r.get_value();

    //changing velocity_left and velocity_right based on curvature
    proj_vl = v_target * (2 + pursuit.curvature * WHEELBASE) / 2;
    proj_vr = v_target * (2 - pursuit.curvature * WHEELBASE) / 2;

    proj_posl =  encl + proj_vl * DT / 1000 + 0.5 * proj_a * 0.5 * DT * DT / 1000 / 1000;
    proj_posr = encr + proj_vr * DT / 1000 + 0.5 * proj_a * 0.5 * DT * DT / 1000 / 1000;


    //assigning left voltage and right voltage
    pwm_l = chassis_left.Calculate(proj_posl, proj_vl, encl, right_velo, LIMIT_FWD, MAX_FWD);
    pwm_r = chassis_right.Calculate(proj_posr, proj_vr, encr, left_velo, LIMIT_FWD, MAX_FWD);

    if(proj_vl > 0) pwm_l += feedforward(proj_vl, KVFF_FWD, KVFF_INT_FWD);
    else if (proj_vl < 0) pwm_l += feedforward(proj_vl, KVFF_FWD, -KVFF_INT_FWD);
    if(proj_vr > 0) pwm_r += feedforward(proj_vr, KVFF_FWD, KVFF_INT_FWD);
    else if (proj_vr < 0) pwm_r += feedforward(proj_vr, KVFF_FWD, -KVFF_INT_FWD);

    pwm_l += feedforward(proj_a, KAFF_FWD, 0);
    pwm_r += feedforward(proj_a, KAFF_FWD, 0);

    printf("index_start: %d   index_end: %d\n", index_start, index_end);
    printf("v: %.2f   a: %.2f\n", v_target, proj_a);
    printf("proj_vl: %.2f   proj_pl: %.2f\n", proj_vl, proj_posl);
    printf("x: %.2f   y: %.2f   theta: %.2f\n\n", odo.state[X], odo.state[Y], odo.state[THETA]);

    volt_chas(pwm_l, pwm_r);
    //printf("projected_pos_l: %.2f   projected_vl: %.2f    projected_accel: %.2f   curvature: %.2f\n", proj_posl, proj_vl, proj_a, pursuit.curvature);
    delay(DT);

    //need to write a shutdown sequence
  }
}

void pure_pursuiter(Path path, double v, double a, double lookahead, double theta, double max_change, double theta_end) {
  Motion_Profile prof(v,a);
  PurePursuit pursuit(lookahead);
  Odometry odo(0,0,theta);

  PIV chas_l(KP_FWD, KI_FWD, KV_FWD);
  PIV chas_r(KP_FWD, KI_FWD, KV_FWD);

  double index_start = 0;
  double index_end = 0;

  double v_target = 0, a_target = 0;

  double proj_vl = 0, proj_vr = 0;
  double proj_pl = 0, proj_pr = 0;

  double prev_pwm_l, prev_pwm_r;
  double pwm_l, pwm_r;

  double vl = 0, vr = 0;

  enc_l.reset();
  enc_r.reset();

  path = prof.inject_trapezoid(path);

  while(true) {
    //getting position and velocity
    vl = mtr_chasFL.get_actual_velocity() * RPM_TO_ENCPS;
    vr = mtr_chasFR.get_actual_velocity() * RPM_TO_ENCPS;

    odo.calculate_state();

    index_start = closest_index(path, odo.state, index_start);
    index_end = index_start + 1;

    if(index_start != path.size()-2) {
      if(!pursuit.update_lk(path[index_start], path[index_end], odo.state)) {
        index_start -=1;
        index_end -=1;
      }

      pursuit.update_lk(path[index_start], path[index_end], odo.state);
    }
    else {
      pursuit.lk[X] = path[path.size()-1][X] + cos(theta_end) * lookahead;
      pursuit.lk[Y] = path[path.size()-1][Y] + sin(theta_end) * lookahead;
     }

    pursuit.find_curvature(odo.state);

    /*
    if(a_target < path[index_end][A]) {
      a_target += 10;
      a_target = fmin(a_target, path[index_end][A]);
    }
    else if (a_target > path[index_end][A]) {
      a_target -= 10;
      a_target = fmax(a_target, path[index_end][A]);
    }
    */

    a_target = path[index_end][A];
    v_target += a_target * DT * MS_TO_S;
    if(a_target > 0) v_target = fmin(path[index_end][V], v_target);
    else if (a_target < 0) v_target = fmax(path[index_end][V], v_target);
    else v_target = path[index_end][V];



    proj_vl = v_target * (2 + pursuit.curvature * WHEELBASE) / 2;
    proj_vr = v_target * (2 - pursuit.curvature * WHEELBASE) / 2;

    proj_pl = enc_l.get_value() + proj_vl * DT * MS_TO_S + 0.5 * a_target * DT * DT * MS_TO_S * MS_TO_S;
    proj_pr = enc_r.get_value() + proj_vr * DT * MS_TO_S + 0.5 * a_target * DT * DT * MS_TO_S * MS_TO_S;

    prev_pwm_l = pwm_l;
    prev_pwm_r = pwm_r;

    pwm_l = chas_l.Calculate(proj_pl, proj_vl, enc_l.get_value(), vl, LIMIT_FWD, MAX_FWD);
    if(proj_vl > 0) pwm_l += feedforward(proj_vl, KVFF_FWD, KVFF_INT_FWD);
    else if(proj_vl < 0) pwm_l += feedforward(proj_vl, KVFF_FWD, -KVFF_INT_FWD);
    pwm_l += feedforward(a_target, KAFF_FWD, 0);

    pwm_r = chas_r.Calculate(proj_pr, proj_vr, enc_r.get_value(), vr, LIMIT_FWD, MAX_FWD);
    if(proj_vr > 0) pwm_r += feedforward(proj_vr, KVFF_FWD, KVFF_INT_FWD);
    else if(proj_vr < 0) pwm_r += feedforward(proj_vr, KVFF_FWD, -KVFF_INT_FWD);
    pwm_r += feedforward(a_target, KAFF_FWD, 0);

    if( std::abs(pwm_l - prev_pwm_l) > max_change && index_start == 0) {
      pwm_l = prev_pwm_l + (pwm_l / std::abs(pwm_l)) * max_change;
    }
    if( std::abs(pwm_r - prev_pwm_r) > max_change && index_start == 0) {
      pwm_r = prev_pwm_r + (pwm_r / std::abs(pwm_r)) * max_change;
    }

    if(v_target == 0) break;

    volt_chas(pwm_l, pwm_r);

    //if(pursuit.lk[X] == path[path.size()-1][X] && pursuit.lk[Y] == path[path.size()-1][Y] && hypot(pursuit.lk[X] - odo.state[X], pursuit.lk[Y]-odo.state[Y]) < 70) break;

    delay(DT);
  }
  volt_chas(0,0);
}

void blue_auton(){
  pwr_intake(127);
  lift(90, 300);
  forward(300, 600, 500, false);
  forward(600, 300, 300, false);
  pwr_intake(0);
  forward(600, 400,350, true);

}
