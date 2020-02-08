#include "custom/config.hpp"
#include "custom/lib_drive.hpp"
#include "custom/lib_auton.hpp"
void initialize() {
	enc_l.reset();
	enc_r.reset();
	delay(500);
	pot_offset = pot.get_value();
}

//test comment 123
void disabled() {

}

void competition_initialize() {
	while(imu.is_calibrating()) {}
}

void autonomous() {
	while(imu.is_calibrating()) {}
	//blue_auton();
	/*
	pwr_intake(0);
	mtr_chasBL.move_velocity(127);
	mtr_chasFL.move_velocity(127);
	mtr_chasBR.move_velocity(127);
	mtr_chasFR.move_velocity(127);
	delay(750);
	pwr_intake(-100);
	mtr_chasBL.move_velocity(-127);
	mtr_chasFL.move_velocity(-127);
	mtr_chasBR.move_velocity(-127);
	mtr_chasFR.move_velocity(-127);
	delay(750);
	pwr_intake(0);
	mtr_chasBL.move_velocity(0);
	mtr_chasFL.move_velocity(0);
	mtr_chasBR.move_velocity(0);
	mtr_chasFR.move_velocity(0);
	*/

	//smooth velocity

	/*
	double enc = 0;
	double prev_enc = 0;
	//PIV velo(0,0,0,5);
	double filter_velo = 0;
	double filter_filter_velo = 0;
	while(true) {
		prev_enc = enc;
		enc = (enc_l.get_value() + enc_r.get_value())/2;
		filter_velo = (1-0.03)*filter_velo + 0.03 * (enc - prev_enc) / DT * 1000;
		printf("velocity: %.2f\n", filter_velo);
		volt_chas(2000,2000);
		delay(15);
	}

	*/

	//piv test/tune

	forward(2000,500,750, false);


	/*
	double gyro = 0;
	double prev_gyro = 0;

	double enc_avg = 0;
	double prev_enc_avg = 0;

	double encoder_angle = 0;
	Odometry state(0,0,0);
	//odometry test/tune
	while(true) {
		prev_gyro = gyro;
		prev_enc_avg = enc_avg;

		gyro = imu.get_yaw();
		encoder_angle = (enc_l.get_value() - enc_r.get_value()) / WHEELBASE * RAD_TO_DEG;

		//weighted average
		printf("gyro angle: %.2f			encoder angle: %.2f			average: %.2f			enc_l %d			enc_r: %d\n", gyro, encoder_angle, (gyro+encoder_angle)/2,enc_l.get_value(), enc_r.get_value());

		delay(15);
	}*/

	/*
	Odometry odo(0,0,0);
	while(true) {
		odo.calculate_state();
		printf("angle: %.2f			x: %.2f			y: %.2f\n", odo.state[THETA], odo.state[X], odo.state[Y]);
		delay(15);
	}
	*/

	/*
	while(true) {
		printf("left_velo: %.2f			right_velo: %.2f\n", mtr_chasBL.get_actual_velocity() * 6, mtr_chasBR.get_actual_velocity() * 6);
		delay(15);
	}
	*/

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

		delay(10);
	}
}
