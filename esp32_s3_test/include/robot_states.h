/**
* Description:
* 
* Note:
* 
* Usage:
* 
**/

#ifndef ROBOTSTATES_H
#define ROBOTSTATES_H
#include <Arduino.h>

/**
 * @brief Struct to store position data and velocity data.
 * @param x_pos: X position (float).
 * @param y_pos: Y position (float).
 * @param theta: Theta (float).
 * @param omega: Omega (float).
 * @param V: Velocity (float).
 * @param d: Distance between wheels (float).
 * @param Vl: Left velocity (float).
 * @param Vr: Right velocity (float).
 * @param dt: Time step (float).
 */
struct positionData_t{
    float x_pos;
    float y_pos;
    float theta;
    float omega;
    float V;
    float d;
    float Vl;
    float Vr;
    float dt;
};

/**
 * @brief Struct to store robot states and control signals.
 * @param x_pos: X position (float).
 * @param y_pos: Y position (float).
 * @param velocity: Velocity (float).
 * @param lineSensor_value_front: Value of front line sensor (int32_t).
 * @param lineSensor_value_back: Value of back line sensor (int32_t).
 * @param baseSpeed: Base speed (int32_t).
 * @param left_controlSignal: Left control signal (int32_t).
 * @param right_controlSignal: Right control signal (int32_t).
 * @param message: Message (String).
 */
struct robotStates_t{
    float x_pos;
    float y_pos;
    float velocity;
    int32_t lineSensor_value_front;
    int32_t lineSensor_value_back;
    int32_t baseSpeed;
    int32_t left_controlSignal;
    int32_t right_controlSignal;
    String message;
};

/**
 * @brief Calculate the xy-position of the robot.
 * 
 * @param positionData: Pointer to the position data struct.
 */
void calculate_Position(struct positionData_t* positionData);

/**
 * @brief Calculate the velocity of the robot.
 * 
 * @param positionData: Pointer to the position data struct.
 * @param sensorData: Pointer to the sensor data struct.
 */
void calculate_velocity(struct positionData_t* positionData, struct sensorData_t* sensorData);

/**
 * @brief Update the robot states.
 * 
 * @param sensorData: Pointer to the sensor data struct.
 * @param positionData: Pointer to the position data struct.
 * @param robotStates: Pointer to the robot states struct.
 * @param front_sensor_value: Value of front line sensor (float).
 * @param back_sensor_value: Value of back line sensor (float).
 * @param controlSignal_left: Left control signal (int32_t).
 * @param controlSignal_right: Right control signal (int32_t).
 */
void update_robotStates(struct sensorData_t* sensorData, struct positionData_t* position_states, struct robotStates_t* robot_states, float front_sensor_value, float back_sensor_value, int32_t controlSignal_left, int32_t controlSignal_right);

/**
 * @brief Set the motor commands.
 * 
 * @param stopCommand: Stop command, 1 for go and 0 to froce stop. (int32_t).
 * @param motor_index: Motor index, 1 for left and 2 for right. (int32_t).
 * @param motor_pwm: Motor pwm (int32_t).
 */
void set_motor_commands(int stopCommand, int motor_index, int motor_pwm);

/**
 * @brief Command the motors to stop.
 * 
 */
void stop_motor_commands(void);

/**
 * @brief Reverse the robot when the line is lost.
 * 
 */
void line_lost_reverse(void);

/**
 * @brief Setup the motor pins, predefined in defines.h.
 * @see defines.h
 */
void motor_pins_setup();
#endif

