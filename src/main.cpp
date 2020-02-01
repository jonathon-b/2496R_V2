#include "custom/config.hpp"
#include "custom/lib_drive.hpp"
#include "custom/lib_auton.hpp"
void initialize() {
	enc_l.reset();
	enc_r.reset();
	delay(500);
	pot_offset = pot.get_value();
}


void disabled() {

}

void competition_initialize() {
	while(imu.is_calibrating()) {}
}

void autonomous() {
	while(imu.is_calibrating()) {}
	blue_auton();
}

bool lift_auto = false;
void opcontrol() {




	mtr_rollR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	mtr_rollL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	mtr_lift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	while (true) {


		drive_chassis();

		if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) lift_auto = !lift_auto;
		lift_auto? pid_lift() : drive_lift();

		drive_intake();
		drive_tray();

		delay(15);
	}
}
