#include "custom/lib_path.hpp"
#include "custom/lib_auton.hpp"
#include "custom/config.hpp"

bool stack_me = false;
void auto_index(){
  while (limit_index.get_value() == false && op_is_on == false){
    pwr_intake(127);
  }
	stack_me = true;
  pwr_intake(0);
}

void skill_index(){
	delay(7000);
  while (limit_index.get_value() == false && op_is_on == false){
    pwr_intake(127);
  }
	stack_me = true;
  pwr_intake(0);
}


void blue_protected() {
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
}

void red_protected() {
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
}

void blue_unprotected() {
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
}

void red_unprotected() {
  //official 6 red
	while(imu.is_calibrating()) {}
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
}

void skills() {
  while(imu.is_calibrating()) {}
	chassis_brake(E_MOTOR_BRAKE_HOLD);
	pwr_intake(70);
	Task lift_start(lift_task);
	forward(1800,700,1500,false);
	Spline path1({0,0,90*DEG_TO_RAD}, {850, 1400, 45 * DEG_TO_RAD}, 0.25, 0.015);
	pure_pursuiter(path1.path, 1400, 1300, 300 ,90 , 90, 400, 300, true);
	pwr_intake(127);
	//Task intake(auto_index);
	forward(1800,400,1500,false);
	mtr_rollR.move_velocity(0);
	mtr_rollL.move_velocity(0);
	forward(1475,1000,1400,true);
	pwr_intake(0);
	turn(125, 160, 280, true);
	forward(500,400,1000,false);
	skill_stack();
	volt_chas(0,0);
  
	chassis_brake(E_MOTOR_BRAKE_COAST);
}

void one_cube() {
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
}
