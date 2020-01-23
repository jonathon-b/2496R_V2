#include "custom/config.hpp"
#include "custom/lib_drive.hpp"
void initialize() {
}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {
}


void print_control(void *param) {
	control.clear();
	delay(100);
	while(true) {
			/*
			control.print(0,1,"pos: %.2f", mtr_tilt.get_position() - tilt_offset);

			delay(1000);
			control.clear();
			delay(50);
			*/
			printf("tilt: %.2f\n", mtr_tilt.get_position());
			delay(20);
	}

}

bool lift_auto = false;
void opcontrol() {
	Task task1 (print_control, (void*) "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "CONTROL_TASK" );
	delay(1000);

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
