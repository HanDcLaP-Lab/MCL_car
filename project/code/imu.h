#ifndef _IMU_H
#define _IMU_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>
#define IMU_DT 0.001
#ifndef PI
#define PI 3.1415926535f
#endif
extern float car_angle;
void IMU_Update_Loop(void);
void IMU_Init();
#endif