#include "custom/lib_drive.hpp"
#include "custom/config.hpp"
#include "classes/PID.hpp"
#define RAW_TO_VOLT 12000/127

#define LIFT_PWR 127

#define INTAKE_PWR 115

#define TILT_PWR 127.0
#define TILT_MAX 4000.0

#define GRAVFF 0
#define KP_LIFT 40
#define KI_LIFT 0.2
#define KD_LIFT 120
#define LIMIT_LIFT 40

//lifth = lift heights : vocab for the professionals
#define LIFTH_BOTTOM 2405
#define LIFTH_TRAY 0
#define LIFTH_SMALLTOWER 1520
#define LIFTH_BIGTOWER 1220


void drive_chassis() {
  double throttle = 127 * sin(control.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y) * PI/127/2);
  double turn = 127 * sin(control.get_analog(E_CONTROLLER_ANALOG_LEFT_X) * PI/127/2);
  double pwr_left = throttle + turn;
  double pwr_right = throttle - turn;
  double sign = 0;
  if(std::abs(pwr_left) > 127) {
    sign = pwr_left / std::abs(pwr_left);
    pwr_right +=  sign * 127 - pwr_left;
    pwr_left = pwr_left / std::abs(pwr_left) * 127;
  }
  else if(std::abs(pwr_right) > 127) {
    sign = pwr_right / std::abs(pwr_right);
    pwr_left += sign * 127 - pwr_right;
    pwr_right = pwr_right / std::abs(pwr_right) * 127;
  }

  mtr_chasBL.move(pwr_left);
  mtr_chasFL.move(pwr_left);
  mtr_chasBR.move(pwr_right);
  mtr_chasFR.move(pwr_right);
}

void drive_lift() {
  if(control.get_digital(E_CONTROLLER_DIGITAL_L1)) mtr_lift.move(-LIFT_PWR);
  else if (control.get_digital(E_CONTROLLER_DIGITAL_R1)) mtr_lift.move(LIFT_PWR);
  else mtr_lift.move_velocity(0);
}
//0 is bottom height -> 3 big height

double preset_heights[3] = {LIFTH_BOTTOM, LIFTH_SMALLTOWER, LIFTH_BIGTOWER};
int index_heights = 0;
PID autolift(KP_LIFT, KI_LIFT, KD_LIFT);
double lift_value = 0;

void pid_lift() {


  if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
    index_heights++;
    index_heights = fmin(index_heights , sizeof(preset_heights) / sizeof(*preset_heights) - 1 );
  }

  if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
    index_heights--;
    index_heights = fmax(index_heights , 0);
  }

  lift_value = -autolift.Calculate(preset_heights[index_heights], pot.get_value(), LIMIT_LIFT) + GRAVFF;
  if(std::abs(autolift.error) <= 20) mtr_lift.move_velocity(0);
  else mtr_lift.move_voltage(lift_value); // it move gamer arm
  //target = preset_heights[index_heights]


}

bool intakeFwd_state = false;
bool intakeRev_state = false;
bool one_in_state = false;

void drive_intake() {
  if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
    one_in_state = !one_in_state;
    intakeRev_state = false;
    intakeFwd_state = false;
  }


  if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
    intakeFwd_state = !intakeFwd_state; //toggle state to roll or not to roll
    intakeRev_state = false;
    one_in_state = false;

  }
  else if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
      intakeRev_state = !intakeRev_state;
      intakeFwd_state = false;
      one_in_state = false;
  }

  if(one_in_state){
    if (line.get_value() <=2500){
      mtr_rollR.move_velocity(0);
      mtr_rollL.move_velocity(0);
      one_in_state = false;
    }
    else{
      mtr_rollR.move_velocity(130);
      mtr_rollL.move_velocity(130);
    }
  }

  else if(intakeFwd_state) {
    //roll forward
    mtr_rollR.move(INTAKE_PWR);
    mtr_rollL.move(INTAKE_PWR);
  }
  else if(intakeRev_state) {
    mtr_rollR.move(-INTAKE_PWR+20);
    mtr_rollL.move(-INTAKE_PWR+20);
  }
  else {
    mtr_rollR.move_velocity(0);
    mtr_rollL.move_velocity(0);
  }

}

double tilt_offset = 0;
void drive_tray() {
  if(bump.get_value()) tilt_offset = mtr_tilt.get_position() - TILT_MAX;
  else if (limit.get_value()) tilt_offset = mtr_tilt.get_position();

  if(control.get_digital(E_CONTROLLER_DIGITAL_UP)){
    //mtr_tilt.move(TILT_PWR);

    //mtr_tilt.move(((double)0.025 * ((double)TILT_MAX - mtr_tilt.get_position()+tilt_offset)));
    //mtr_tilt.move(TILT_PWR * cos((mtr_tilt.get_position()-tilt_offset) * PI * 2.0 / TILT_MAX ));

    if(mtr_tilt.get_position() - tilt_offset < 1000) mtr_tilt.move(TILT_PWR);
    else if (mtr_tilt.get_position() - tilt_offset < 2000) mtr_tilt.move(TILT_PWR - 40);
    else if (mtr_tilt.get_position() - tilt_offset < 3000) mtr_tilt.move(TILT_PWR - 60);
    else if (mtr_tilt.get_position() - tilt_offset < 3500) mtr_tilt.move(TILT_PWR - 80);
    else if (mtr_tilt.get_position() - tilt_offset < 3750) mtr_tilt.move(TILT_PWR - 100);
    else mtr_tilt.move(TILT_PWR - 110);

    mtr_rollR.move(-4);
    mtr_rollL.move(-4);

  }
  else if(control.get_digital(E_CONTROLLER_DIGITAL_DOWN)){
    mtr_tilt.move(-TILT_PWR);
    mtr_rollR.move(-20);
    mtr_rollL.move(-20);
  }
  else{
    mtr_tilt.move_velocity(0);
  }
}
