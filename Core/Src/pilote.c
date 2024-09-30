/*
 * pilote.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Guillaume
 */


#include "main.h"
#include "pilote.h"
#include "odometry.h"
#include "corrector.h"
#include "debug.h"





double target_WheelMotor1_rpm, target_WheelMotor2_rpm, target_WheelMotor3_rpm, target_WheelMotor4_rpm;

static uint32_t time = 0;
static uint32_t time_tret = 0;
double pid_inputMaxP, pid_inputMaxN;

double cmd_x, cmd_y, cmd_z;

ODOMETRY_speed_wheel wheel;
PILOTE_target_speed wheel_speed;
CORRECTOR_pid_out pid_out;
static int16_t i = 0;

/**
 *
 * @param x en cm/s
 * @param y en cm/s
 * @param z en centirad/s
 */
void PILOTE_recieve_cmd(double x, double y, double z){

	if((((double)((int8_t)x)/100) >= 0.32) ){
		x = 0.32;
	}else if (  (((double)((int8_t)x)/100) <= -0.32)){
		x = -0.32;
	}else{
		x = (double)((int8_t)x)/100;
	}

	if ((((double)((int8_t)y)/100) >= 0.32)){
		y = 0.32;
	}else if (  (((double)((int8_t)y)/100) <= -0.32)){
		y = -0.32;
	}else{
		y = ((double)((int8_t)y)/100);
	}

	if((((double)((int8_t)z)/100) >= 0.32)){
		z = 0.32;
	}else if(  (((double)((int8_t)z)/100) <= -0.32)){
		z = -0.32;
	}else{
		z = ((double)((int8_t)z)/100);
	}



	__disable_irq();
	cmd_x = x; // en mètre/s
	cmd_y = y; // en mètre/s
	cmd_z = z; // en rad/s
	__enable_irq();
	//	DEBUG_update_cmd(x, y, z);
}

void PILOTE_mouv(){
	time = HAL_GetTick();
	//	HAL_GPIO_TogglePin(OUT_LE3_GPIO_Port, OUT_LE3_Pin); // pin de débug pour verifier la période et vérifier qu'on ne dépasse pas l'it
	//PILOTE_target_speed wheel_speed = {0};

	wheel = ODOMETRY_it_10ms();
	//	if(i >= 250){
	//		cmd_x = 0.5; // en mètre/s
	//		cmd_y = 0; // en mètre/s
	//		cmd_z = 0; // en rad/s
	//		wheel_speed.target_WheelMotor1_rpm = (double)4; //en rad/s
	//		wheel_speed.target_WheelMotor2_rpm = (double)4; //en rad/s
	//		wheel_speed.target_WheelMotor3_rpm = (double)4; //en rad/s
	//		wheel_speed.target_WheelMotor4_rpm = (double)4; //en rad/s
	//	}else{
	//		i++;
	//	}
	wheel_speed.target_WheelMotor1_rpm = (cmd_x + cmd_y - cmd_z*(L1+L2))/(wheelRadius); //rad/s
	wheel_speed.target_WheelMotor2_rpm = (cmd_x - cmd_y - cmd_z*(L1+L2))/(wheelRadius);
	wheel_speed.target_WheelMotor3_rpm = (cmd_x + cmd_y + cmd_z*(L1+L2))/(wheelRadius);
	wheel_speed.target_WheelMotor4_rpm = (cmd_x - cmd_y + cmd_z*(L1+L2))/(wheelRadius);


	CORRECTOR_pid(wheel, wheel_speed);

	//	HAL_GPIO_TogglePin(OUT_LE3_GPIO_Port, OUT_LE3_Pin);
	time_tret = (HAL_GetTick() - time);
	//	DEBUG_update_cmd(0, time, time_tret);
}
