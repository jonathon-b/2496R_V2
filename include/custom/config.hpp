#ifndef CONFIG_H
#define CONFIG_H
#include "main.h"
using namespace pros;

extern Controller control;
extern Motor mtr_chasBL;
extern Motor mtr_chasFL;
extern Motor mtr_chasBR;
extern Motor mtr_chasFR;
extern Motor mtr_lift;
extern Motor mtr_tilt;
extern Motor mtr_rollL;
extern Motor mtr_rollR;
extern ADIEncoder enc_l;
extern ADIEncoder enc_r;
extern ADIAnalogIn pot;
extern ADIDigitalIn limit;
extern ADIDigitalIn bump;
extern ADILineSensor line;
/*
  CONSTANTS
*/
#define X 0
#define Y 1
#define THETA 2
#define V 2
#define A 3

#define L 0
#define R 1

#define PI 3.141592653
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI

#define WHEELBASE 0

#define DT 15
#define MS_TO_S 1/1000

#define KP_CHAS_POS 0
#define KI_CHAS_POS 0
#define KD_CHAS_POS 0

#define KP_CHAS_V 0

#define KVFF_CHAS 0
#define KAFF_CHAS 0
#define KVFF_INT_CHAS 0





#endif
