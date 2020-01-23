#include "custom/config.hpp"

#define POT_TO_DEGREES 0

Controller control(E_CONTROLLER_MASTER);


//motor(port, gear, reverse, measurement)
Motor mtr_chasBL(4, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_chasFL(20, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_COUNTS);
Motor mtr_chasBR(6, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_COUNTS);
Motor mtr_chasFR(7, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_lift(3, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_tilt(10, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_rollL(5, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_rollR(11, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_COUNTS);

//encoder(port,adjacent_port, reverse)
ADIEncoder enc_l(4,5,false);
ADIEncoder enc_r(6,7,false);

//ADI(port)
ADIAnalogIn pot(1);
ADIDigitalIn limit(3);
ADIDigitalIn bump(2);
ADIAnalogIn line(8);
