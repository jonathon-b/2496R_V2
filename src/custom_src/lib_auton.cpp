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

#define KP_TURN 250
#define KI_TURN 16
#define KD_TURN 0
#define LIMIT_TURN 10
#define MAX_TURN 500.0

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
  double angle_volt = 0;

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
      voltage = -chassis_piv.Calculate(projected_pos, projected_velo, std::abs(encoder_avg), -velocity, LIMIT_FWD, MAX_FWD);
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
      voltage = -turn_pid.Calculate(projected_theta, std::abs(angle), LIMIT_TURN, MAX_TURN);
      voltage -= feedforward(projected_velo, KVFF_TURN, KVFF_INT_TURN);
      voltage -= feedforward(projected_accel, KAFF_TURN, 0);
    }

    if(std::abs(turn_pid.error) <= 1 && projected_theta == target) break;
    volt_chas(voltage, -voltage);

    delay(15);
  }
  volt_chas(0,0);
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

void pwr_lift(int pwr) {
  mtr_lift.move_velocity(pwr);
}

int counter = 0;

void pure_pursuiter(Path path, double v, double a, double lookahead, double theta , double theta_end, double max_change, double extra_dist, bool reverse) {
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

  double sign = 1;
  if(reverse) sign = -1;

  enc_l.reset();
  enc_r.reset();
  double conv = theta_end * DEG_TO_RAD;
  path.push_back({path[path.size()-1][X] + cos(conv) * extra_dist, path[path.size()-1][Y] + sin(conv) * extra_dist, 0, -a});
  path = prof.inject_trapezoid(path);
  path.push_back({path[path.size()-1][X] + cos(conv) * extra_dist, path[path.size()-1][Y] + sin(conv) * extra_dist, 0, -a});
  for(int i = 0; i < path.size(); i++) {
    printf("path[X]: %.2f     path[Y]: %.2f   path[V]: %.2f    path[A]: %.2f\n", path[i][X], path[i][Y], path[i][V], path[i][A]);
  }
  while(true) {
    //getting position and velocity
    if(reverse) {
      vr = -mtr_chasFL.get_actual_velocity() * RPM_TO_ENCPS;
      vl = -mtr_chasFR.get_actual_velocity() * RPM_TO_ENCPS;
    }
    else {
      vl = mtr_chasFL.get_actual_velocity() * RPM_TO_ENCPS;
      vr = mtr_chasFR.get_actual_velocity() * RPM_TO_ENCPS;
    }

    odo.calculate_state();

    index_start = closest_index(path, {sign * odo.state[X], sign * odo.state[Y], odo.state[THETA]}, index_start);
    index_end = index_start + 1;

    if(index_start != path.size()-1-1) {
      if(!pursuit.update_lk(path[index_start], path[index_end], {sign * odo.state[X], sign * odo.state[Y], odo.state[THETA]} )) {
        index_start -=1;
        index_end -=1;
      }

      pursuit.update_lk(path[index_start], path[index_end], { sign * odo.state[X], sign * odo.state[Y], odo.state[THETA]} );
    }
    else {
      pursuit.lk[X] = path[path.size()-1][X];
      pursuit.lk[Y] = path[path.size()-1][Y];
     }

    pursuit.find_curvature( { sign * odo.state[X], sign * odo.state[Y], odo.state[THETA]} );

    a_target = path[index_end][A];
    v_target += a_target * DT * MS_TO_S;
    if(a_target > 0) v_target = fmin(path[index_end][V], v_target);
    else if (a_target < 0) v_target = fmax(path[index_end][V], v_target);
    else v_target = path[index_end][V];

    proj_vl = v_target * (2 + pursuit.curvature * WHEELBASE) / 2;
    proj_vr = v_target * (2 - pursuit.curvature * WHEELBASE) / 2;

    if(reverse) {
      proj_pl = -enc_r.get_value() + proj_vl * DT * MS_TO_S + 0.5 * a_target * DT * DT * MS_TO_S * MS_TO_S;
      proj_pr = -enc_l.get_value() + proj_vr * DT * MS_TO_S + 0.5 * a_target * DT * DT * MS_TO_S * MS_TO_S;
    }
    else {
      proj_pl = enc_l.get_value() + proj_vl * DT * MS_TO_S + 0.5 * a_target * DT * DT * MS_TO_S * MS_TO_S;
      proj_pr = enc_r.get_value() + proj_vr * DT * MS_TO_S + 0.5 * a_target * DT * DT * MS_TO_S * MS_TO_S;
    }


    prev_pwm_l = pwm_l;
    prev_pwm_r = pwm_r;

    if(reverse) {

      pwm_l = chas_l.Calculate(proj_pl, proj_vl, -enc_r.get_value(), vl, LIMIT_FWD, MAX_FWD);
      if(proj_vl > 0) pwm_l += feedforward(proj_vl, KVFF_FWD, KVFF_INT_FWD);
      else if(proj_vl < 0) pwm_l += feedforward(proj_vl, KVFF_FWD, -KVFF_INT_FWD);
      pwm_l += feedforward(a_target, KAFF_FWD, 0);

      pwm_r = chas_r.Calculate(proj_pr, proj_vr, -enc_l.get_value(), vr, LIMIT_FWD, MAX_FWD);
      if(proj_vr > 0) pwm_r += feedforward(proj_vr, KVFF_FWD, KVFF_INT_FWD);
      else if(proj_vr < 0) pwm_r += feedforward(proj_vr, KVFF_FWD, -KVFF_INT_FWD);
      pwm_r += feedforward(a_target, KAFF_FWD, 0);

    }
    else {
      pwm_l = chas_l.Calculate(proj_pl, proj_vl, enc_l.get_value(), vl, LIMIT_FWD, MAX_FWD);
      if(proj_vl > 0) pwm_l += feedforward(proj_vl, KVFF_FWD, KVFF_INT_FWD);
      else if(proj_vl < 0) pwm_l += feedforward(proj_vl, KVFF_FWD, -KVFF_INT_FWD);
      pwm_l += feedforward(a_target, KAFF_FWD, 0);

      pwm_r = chas_r.Calculate(proj_pr, proj_vr, enc_r.get_value(), vr, LIMIT_FWD, MAX_FWD);
      if(proj_vr > 0) pwm_r += feedforward(proj_vr, KVFF_FWD, KVFF_INT_FWD);
      else if(proj_vr < 0) pwm_r += feedforward(proj_vr, KVFF_FWD, -KVFF_INT_FWD);
      pwm_r += feedforward(a_target, KAFF_FWD, 0);
    }



    if( std::abs(pwm_l - prev_pwm_l) > max_change && (index_start == 0 || index_end > path.size()-1-1)) {
      pwm_l = prev_pwm_l + (pwm_l / std::abs(pwm_l)) * max_change;
    }
    if( std::abs(pwm_r - prev_pwm_r) > max_change && (index_start == 0 || index_end > path.size()-1-1)) {
      pwm_r = prev_pwm_r + (pwm_r / std::abs(pwm_r)) * max_change;
    }

    if(v_target == 0) break;

    if(reverse) {
      volt_chas(-pwm_r, -pwm_l);
    }
    else {
      volt_chas(pwm_l, pwm_r);
    }

    //if(pursuit.lk[X] == path[path.size()-4][X] && pursuit.lk[Y] == path[path.size()-4][Y] && hypot(pursuit.lk[X] - odo.state[X], pursuit.lk[Y]-odo.state[Y]) < 20) break;

    printf("odo[X]: %.2f    odo[Y]: %.2f    odo[THETA]: %.2f\n\n", odo.state[X], odo.state[Y], odo.state[THETA]);

    delay(DT);
  }
  PID turn(KP_TURN,KI_TURN,KD_TURN);
  double turn_power = 0;
  while(true) {
    odo.calculate_state();
    turn_power = turn.Calculate(theta_end, odo.state[THETA], LIMIT_TURN, MAX_TURN);
    if(std::abs(turn.error) < .5 && std::abs(turn_power) <= 1000) counter++;
    if(counter >= 5) break;
    volt_chas(-turn_power,turn_power);
    delay(DT);
  }
  volt_chas(0,0);
}

void stack() {
  double tilt_offset = 0;
  while(!bump.get_value()) {
      if(mtr_tilt.get_position() - tilt_offset < 1000) mtr_tilt.move(127);
      else if (mtr_tilt.get_position() - tilt_offset < 2000) mtr_tilt.move(127 - 20);
      else if (mtr_tilt.get_position() - tilt_offset < 3000) mtr_tilt.move(127 - 40);
      else if (mtr_tilt.get_position() - tilt_offset < 3500) mtr_tilt.move(127 - 60);
      else if (mtr_tilt.get_position() - tilt_offset < 3750) mtr_tilt.move(127 - 80);
      else mtr_tilt.move(127 - 110);

      mtr_rollR.move(60);
      mtr_rollL.move(60);
  }
  volt_chas(5000,5000);
  delay(500);
  volt_chas(0,0);
  while(!limit.get_value()) {
    mtr_tilt.move(-127);
    mtr_rollR.move(-20);
    mtr_rollL.move(-20);

  }

  mtr_tilt.move_velocity(0);
  volt_chas(-10000,-19000);
  delay(1000);
  volt_chas(0,0);

}

void lift_task(){
  pwr_lift(127);
  delay(270);
  pwr_lift(-127);
  delay(300);
  pwr_lift(0);
  pwr_intake(127);
}

void blue_auton(){
  pwr_intake(127);
  lift(90, 300);
  forward(300, 600, 500, false);
  forward(600, 300, 300, false);
  pwr_intake(0);
  forward(600, 400,350, true);

}
