/*
 * corrector.h
 *
 *  Created on: 12 juin 2020
 *      Author: Guillaume
 */

#ifndef INC_CORRECTOR_H_
#define INC_CORRECTOR_H_

#include "pilote.h"
#include "odometry.h"

typedef struct{
	double pid_output_WheelMotor1_rpm;
	double pid_output_WheelMotor2_rpm;
	double pid_output_WheelMotor3_rpm;
	double pid_output_WheelMotor4_rpm;

}CORRECTOR_pid_out;
/**
 *
 * \param wheel
 * \param wheel_speed
 */
void CORRECTOR_pid(ODOMETRY_speed_wheel wheel, PILOTE_target_speed wheel_speed);

#endif /* INC_CORRECTOR_H_ */
