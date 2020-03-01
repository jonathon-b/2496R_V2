#include "custom/config.hpp"
#include "custom/lib_drive.hpp"
#include "custom/lib_auton.hpp"
void initialize() {
	mtr_rollR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	mtr_rollL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	mtr_lift.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	enc_l.reset();
	enc_r.reset();
	delay(500);
	pot_offset = pot.get_value();
}

//test comment 123
void disabled() {
	while(imu.is_calibrating()) {}
}

void competition_initialize() {
}

void auton_lift() {
	while(true) {
		pid_lift();
		delay(DT);
	}
}

bool op_is_on = false;
	bool stack_me = false;
void auto_index(){
  while (limit_index.get_value() == false && op_is_on == false){
    pwr_intake(127);
  }
	stack_me = true;
  pwr_intake(0);
}

void delay_index(){
	delay(3000);
  while (limit_index.get_value() == false && op_is_on == false){
    pwr_intake(127);
  }
	stack_me = true;
  pwr_intake(0);
}

bool lift_auto = false;

void autonomous() {
	//official 6 blue
	/*
	while(imu.is_calibrating()) {}
	chassis_brake(E_MOTOR_BRAKE_HOLD);
	pwr_intake(127);
	forward(1750,650,1600,false);
	delay(100);
	pid_turn(30);
	delay(100);
	Task indexer_on(auto_index);
	forward(475, 700, 1500,false);
	delay(100);
	pid_turn(-30);
	delay(100);
	forward_stack(960,1300,1500,true);
	stack_me = limit_index.get_value();
	//pid_fwd(-920);
	turn(136, 170, 200, true);
	//pid_turn(-135);
	op_is_on = true;
	mtr_rollL.move_velocity(0);
	mtr_rollR.move_velocity(0);
	forward(1250,1400,1600,false);
	//chassis_stop();
	stack_me = true;
	//volt_chas(3000,3000);
	if(stack_me) stack();
	volt_chas(0,0);
	chassis_brake(E_MOTOR_BRAKE_COAST);
	delay(200);
 	pwr_intake(0);
	*/


	//official 6 red
	/*while(imu.is_calibrating()) {}
	chassis_brake(E_MOTOR_BRAKE_HOLD);
	pwr_intake(127);
	forward(1750,650,1600,false);
	delay(100);
	pid_turn(-30);
	delay(100);
	Task indexer_on(auto_index);
	forward(475, 700, 1500,false);
	delay(100);
	pid_turn(30);
	delay(100);
	forward_stack(960,1300,1500,true);
	stack_me = limit_index.get_value();
	//pid_fwd(-920);
	turn(136, 170, 200, false);
	//pid_turn(-135);
	op_is_on = true;
	mtr_rollL.move_velocity(0);
	mtr_rollR.move_velocity(0);
	forward(1250,1400,1600,false);
	//chassis_stop();
	stack_me = true;
	//volt_chas(3000,3000);
	if(stack_me) stack();
	volt_chas(0,0);
	chassis_brake(E_MOTOR_BRAKE_COAST);
	delay(200);
 	pwr_intake(0);
	*/

	//protected 3 cube auton BLUE
	/*
	while(imu.is_calibrating()) {}
  chassis_brake(E_MOTOR_BRAKE_HOLD);
  pwr_intake(127);
  forward(800,800,1000,false);
  delay(30);
  turn(80, 180, 250, false);
  Task indexer_on(auto_index);
  forward(950,800,800,false);
  //turn(45, 180,250, false);
  pid_turn(40);
  delay(100);
  pwr_intake(0);
  forward(600,500,1200,false);
  aggro_stack();
  volt_chas(0,0);
  chassis_brake(E_MOTOR_BRAKE_COAST);
	*/


	//protected 3 cube auton RED
	/*
	while(imu.is_calibrating()) {}
  chassis_brake(E_MOTOR_BRAKE_HOLD);
  pwr_intake(127);
  forward(800,800,1000,false);
  delay(30);
  turn(80, 180, 250, true);
  Task indexer_on(auto_index);
  forward(950,800,800,false);
  //turn(45, 180,250, false);
  pid_turn(-40);
  delay(100);
  pwr_intake(0);
  forward(600,500,1200,false);
  aggro_stack();
  volt_chas(0,0);
  chassis_brake(E_MOTOR_BRAKE_COAST);
*/


	/*push 1 cube auton
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



	//smooth velocity test

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

	//piv test/tune test

	//forward(2000,500,750, false);
	//arc_turns(90, 1000, 500, 750, false);


	/* odometry test/tune test
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

	/* //odometry final test
	Odometry odo(0,0,0);
	while(true) {
		odo.calculate_state();
		printf("angle: %.2f			x: %.2f			y: %.2f\n", odo.state[THETA], odo.state[X], odo.state[Y]);
		delay(15);
	}
	*/

	/* //final velocity test
	while(true) {
		printf("left_velo: %.2f			right_velo: %.2f\n", mtr_chasBL.get_actual_velocity() * 6, mtr_chasBR.get_actual_velocity() * 6);
		delay(15);
	}
	*/

	/* //purepursuit test
	Motion_Profile prof(500,750);
	std::vector<double> start = {0,0,90 * DEG_TO_RAD};
	std::vector<double> end = {100,100,45 * DEG_TO_RAD};
	Spline a(start,end,0.1);
	printf("ax: %.5f		bx: %.5f		cx: %.5f		dx: %.5f\n", a.ax, a.bx, a.cx, a.dx);
	printf("ay: %.5f		by: %.5f		cy: %.5f		dy: %.5f\n", a.ay, a.by, a.cy, a.dy);
	for(int i = 0; i < a.path.size(); i++) {
		printf("index: %d		x: %.2f		y: %.2f\n", i, a.path[i][X], a.path[i][Y]);
	}
	*/

	//injection of points test
	/*
	Motion_Profile prof(1000, 2000);
	std::vector<double> start = {0,0,90 * DEG_TO_RAD};
	std::vector<double> end = {1000,1000,0 * DEG_TO_RAD};
	Spline a(start,end,0.01, 2);
	a.path = prof.inject_trapezoid(a.path);
	printf("ax: %.5f		bx: %.5f		cx: %.5f		dx: %.5f\n", a.ax, a.bx, a.cx, a.dx);
	printf("ay: %.5f		by: %.5f		cy: %.5f		dy: %.5f\n", a.ay, a.by, a.cy, a.dy);
	for(int i = 0; i < a.path.size(); i++) {
		printf("index: %d		x: %.2f		y: %.2f		v: %.2f			a: %.2f\n", i, a.path[i][X], a.path[i][Y], a.path[i][V], a.path[i][A]);
	}
	*/

	/* //changing odo to just imu test
	Odometry odo(0,0,90);
	while(true) {
		odo.calculate_state();
		printf("x: %.2f		y: %.2f		theta: %.2f\n", odo.state[X], odo.state[Y], odo.state[THETA]);
		delay(15);
	}
	*/


	//std::vector<double> start = {0,0,90 * DEG_TO_RAD};
	//std::vector<double> end = {942, 1807, 90 * DEG_TO_RAD};

	//path_follow(path.path, 300, 450, 100, 90);

	/* testing odometry with angle offset
	Odometry odo(0,0,90);
	while(true) {
		odo.calculate_state();
		printf("X: %.2f		y: %.2f		theta: %.2f\n\n", odo.state[X], odo.state[Y], odo.state[THETA]);
		delay(15);
	}
	*/

	//apparently this one works
	//Spline a({0,0,90 * DEG_TO_RAD}, {-556,2462, 90 * DEG_TO_RAD}, 0.2, 6);
	//pure_pursuiter(a.path, 200, 100, 90);

	//Spline b({0,0,90 * DEG_TO_RAD}, {1000, 2000, -90 * DEG_TO_RAD}, 0.05, 2)
	//Spline b({0,0,90 * DEG_TO_RAD}, {2000,2000,45 * DEG_TO_RAD}, 0.1, 2.5);
	//pure_pursuiter(b.path, 400, 120 ,90);
	//pure_pursuiter(b.path, 750, 500, 200, 90 , 90, 200, 100, true);
	//std::vector<std::vector<double>> path = {{0,0}, {0,300}, {-100,500}, {-200,800}};
	//pwr_intake(127);
	/*
	pwr_lift(127);
	delay(230);
	pwr_lift(0);
	pwr_lift(-100);
	delay(220);
	pwr_lift(0);
	*/

	//forward(1600, 800, 900, false);
	//Spline path1({0,0,90 * DEG_TO_RAD}, {900, 1420, 90 * DEG_TO_RAD}, 0.25, 0.1);
	/*Path path;
	path.push_back({0,0,0,0});
	path.push_back({100,400,0,0});
	path.push_back({300,600,0,0});
	path.push_back({500,800,0,0});
	path.push_back({700,800,0,0});
	path.push_back({800,1000,0,0});
	path.push_back({1000,1300,0,0});
	path.push_back({1300,1600,0,0});*/
	//pure_pursuiter(path1.path, 900, 900, 300, 90, 90, 400, 700, true);
	//forward(1600,1000,1200,true);
	//purepursuiter(path1.path, 900 ,900, 300, 90 ,90 , 400 ,700, true);
	//turn(90,100,200,true);


}

void opcontrol() {
	//control.print(1,1,std::to_string(-false));

 //protected auton (empty)
/*

 */



	while(imu.is_calibrating()) {}
	chassis_brake(E_MOTOR_BRAKE_HOLD);
	pwr_intake(70);
	Task lift_start(lift_task);
	forward(1800,700,1500,false);
	Spline path1({0,0,90*DEG_TO_RAD}, {1000, 1455, 45 * DEG_TO_RAD}, 0.25, 0.015);
	pure_pursuiter(path1.path, 1400, 1300, 300 ,90 , 90, 400, 300, true);
	pwr_intake(127);
	Task intake(auto_index);
	forward(1940,500,1500,false);
	//pwr_intake(30);
	forward(1600,1500,1800,true);
	pwr_intake(0);
	turn(125, 160, 280, true);
	volt_chas(12000,12000);
	delay(650);
	volt_chas(0,0);
	stack();
	volt_chas(0,0);
	chassis_brake(E_MOTOR_BRAKE_COAST);


	op_is_on = true;
	while (true) {
		chassis_brake(E_MOTOR_BRAKE_COAST);
		drive_chassis();

		if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) lift_auto = !lift_auto;
		lift_auto? pid_lift() : drive_lift();
		drive_intake();
		drive_tray();

		delay(10);
	}





}
