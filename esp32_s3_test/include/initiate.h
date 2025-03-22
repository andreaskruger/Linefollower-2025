/**
* Description:
* 
* Note:
* 
* Usage:
* 
**/


#ifndef INITIATE_H
#define INITIATE_H
#include "sensors.h"
#include "controllers.h"
#include "robot_states.h"
#include "defines.h"
#include "filters.h"

/**
 * @brief Initialize all structs to predefined values or standard values.
 * 
 * @param sensorData: Pointer to the sensor data struct.
 * @param encoderData: Pointer to the encoder data struct.
 * @param pwm_pid: Pointer to the PWM PID struct.
 * @param right_pwm_pid: Pointer to the right PWM PID struct.
 * @param left_pwm_pid: Pointer to the left PWM PID struct.
 * @param speed_pid: Pointer to the speed PID struct.
 * @param std_pid_values: Pointer to the standard PID values struct.
 * @param positionData: Pointer to the position data struct.
 * @param robotStates: Pointer to the robot states struct.
 */
void initiate_structs(struct sensorData_t* sensorData, struct encoderData_t* encoderData, struct PID_t* pwm_pid, struct PID_t* right_pwm_pid, struct PID_t* left_pwm_pid, struct PID_t* speed_pid, struct STD_PID_t* std_pid_values, struct positionData_t* positionData, struct robotStates_t* robotStates, struct lowPassFilter_t* lowPassFilter_front, struct lowPassFilter_t* lowPassFilter_back);

#endif