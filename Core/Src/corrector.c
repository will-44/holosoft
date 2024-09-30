/*
 * corrector.c
 *
 *  Created on: 12 juin 2020
 *      Author: Guillaume
 */


#include "main.h"
#include "corrector.h"
#include "motors.h"
#include "debug.h"


// **** Variable pour le PID en vitesse (RPM) sur la roue
static double pid_error_WheelMotor1_rpm, pid_error_WheelMotor2_rpm, pid_error_WheelMotor3_rpm, pid_error_WheelMotor4_rpm;
static double pid_lasterror_WheelMotor1_rpm, pid_lasterror_WheelMotor2_rpm, pid_lasterror_WheelMotor3_rpm, pid_lasterror_WheelMotor4_rpm;
//static double pid_input_WheelMotor1_rpm, pid_input_WheelMotor2_rpm, pid_input_WheelMotor3_rpm, pid_input_WheelMotor4_rpm;
static double pid_integral_WheelMotor1_rpm, pid_integral_WheelMotor2_rpm, pid_integral_WheelMotor3_rpm, pid_integral_WheelMotor4_rpm;
static double pid_derivate_WheelMotor1_rpm, pid_derivate_WheelMotor2_rpm, pid_derivate_WheelMotor3_rpm, pid_derivate_WheelMotor4_rpm;

CORRECTOR_pid_out pid_out;


void CORRECTOR_pid(ODOMETRY_speed_wheel wheel, PILOTE_target_speed wheel_speed){

	if(PID_ON) {
		pid_error_WheelMotor1_rpm = wheel_speed.target_WheelMotor1_rpm - wheel.speed_WheelMotor1_rpm;
		pid_error_WheelMotor2_rpm = wheel_speed.target_WheelMotor2_rpm - wheel.speed_WheelMotor2_rpm;
		pid_error_WheelMotor3_rpm = wheel_speed.target_WheelMotor3_rpm - wheel.speed_WheelMotor3_rpm;
		pid_error_WheelMotor4_rpm = wheel_speed.target_WheelMotor4_rpm - wheel.speed_WheelMotor4_rpm;

		pid_integral_WheelMotor1_rpm = pid_integral_WheelMotor1_rpm + (pid_error_WheelMotor1_rpm );
		pid_integral_WheelMotor2_rpm = pid_integral_WheelMotor2_rpm + (pid_error_WheelMotor2_rpm );
		pid_integral_WheelMotor3_rpm = pid_integral_WheelMotor3_rpm + (pid_error_WheelMotor3_rpm );
		pid_integral_WheelMotor4_rpm = pid_integral_WheelMotor4_rpm + (pid_error_WheelMotor4_rpm);

		pid_derivate_WheelMotor1_rpm = (pid_error_WheelMotor1_rpm - pid_lasterror_WheelMotor1_rpm) ;
		pid_derivate_WheelMotor2_rpm = (pid_error_WheelMotor2_rpm - pid_lasterror_WheelMotor2_rpm) ;
		pid_derivate_WheelMotor3_rpm = (pid_error_WheelMotor3_rpm - pid_lasterror_WheelMotor3_rpm) ;
		pid_derivate_WheelMotor4_rpm = (pid_error_WheelMotor4_rpm - pid_lasterror_WheelMotor4_rpm) ;

		pid_out.pid_output_WheelMotor1_rpm = (pid_KP_WheelMotor1_rpm * pid_error_WheelMotor1_rpm) + (pid_KI_WheelMotor1_rpm * pid_integral_WheelMotor1_rpm) + (pid_KD_WheelMotor1_rpm * pid_derivate_WheelMotor1_rpm);
		pid_out.pid_output_WheelMotor2_rpm = (pid_KP_WheelMotor2_rpm * pid_error_WheelMotor2_rpm) + (pid_KI_WheelMotor2_rpm * pid_integral_WheelMotor2_rpm) + (pid_KD_WheelMotor2_rpm * pid_derivate_WheelMotor2_rpm);
		pid_out.pid_output_WheelMotor3_rpm = (pid_KP_WheelMotor3_rpm * pid_error_WheelMotor3_rpm) + (pid_KI_WheelMotor3_rpm * pid_integral_WheelMotor3_rpm) + (pid_KD_WheelMotor3_rpm * pid_derivate_WheelMotor3_rpm);
		pid_out.pid_output_WheelMotor4_rpm = (pid_KP_WheelMotor4_rpm * pid_error_WheelMotor4_rpm) + (pid_KI_WheelMotor4_rpm * pid_integral_WheelMotor4_rpm) + (pid_KD_WheelMotor4_rpm * pid_derivate_WheelMotor4_rpm);



		pid_lasterror_WheelMotor1_rpm = pid_error_WheelMotor1_rpm;
		pid_lasterror_WheelMotor2_rpm = pid_error_WheelMotor2_rpm;
		pid_lasterror_WheelMotor3_rpm = pid_error_WheelMotor3_rpm;
		pid_lasterror_WheelMotor4_rpm = pid_error_WheelMotor4_rpm;
	}
	else {
		pid_out.pid_output_WheelMotor1_rpm =  wheel_speed.target_WheelMotor1_rpm;
		pid_out.pid_output_WheelMotor2_rpm =  wheel_speed.target_WheelMotor2_rpm;
		pid_out.pid_output_WheelMotor3_rpm =  wheel_speed.target_WheelMotor3_rpm;
		pid_out.pid_output_WheelMotor4_rpm =  wheel_speed.target_WheelMotor4_rpm;
	}

	DEBUG_update_corrector(wheel_speed.target_WheelMotor1_rpm, pid_out.pid_output_WheelMotor1_rpm, pid_out.pid_output_WheelMotor2_rpm, pid_out.pid_output_WheelMotor3_rpm, pid_error_WheelMotor1_rpm , pid_error_WheelMotor2_rpm, pid_error_WheelMotor3_rpm, pid_error_WheelMotor4_rpm, wheel.speed_WheelMotor1_rpm, wheel.speed_WheelMotor2_rpm, wheel.speed_WheelMotor3_rpm, wheel.speed_WheelMotor4_rpm);

	MOTORS_update(pid_out);

}
