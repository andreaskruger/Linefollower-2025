/**
* Description:
* 
* Note:
* 
* Usage:
* 
**/

#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include <Arduino.h>

/**
 * @brief PID controller struct.
 * 
 * @param Kp: Proportional gain (float).
 * @param Ki: Integral gain (float).
 * @param Kd: Derivative gain (float).
 * @param setpoint: Setpoint (int32_t).
 * @param error: Error (int32_t).
 * @param error_prev: Previous error (int32_t).
 * @param error_sum: Sum of errors (int32_t).
 * @param output: Control signal (int32_t).
 */
struct PID_t{
    float Kp;
    float Ki;
    float Kd;
    int32_t setpoint;
    int32_t error;
    int32_t error_prev;
    int32_t error_sum;
    int32_t output;
};

/**
 * @brief Standard PID parameters struct.
 * 
 * @param speed_Kp: Speed proportional gain (float).
 * @param speed_Ki: Speed integral gain (float).
 * @param speed_Kd: Speed derivative gain (float).
 * @param pwm_Kp: PWM proportional gain (float).
 * @param pwm_Ki: PWM integral gain (float).
 * @param pwm_Kd: PWM derivative gain (float).
 */
struct STD_PID_t{
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    float pwm_Kp;
    float pwm_Ki;
    float pwm_Kd;
};

/**
 * @brief Aggressive PID parameters struct.
 * 
 * @param speed_Kp: Speed proportional gain (float).
 * @param speed_Ki: Speed integral gain (float).
 * @param speed_Kd: Speed derivative gain (float).
 * @param pwm_Kp: PWM proportional gain (float).
 * @param pwm_Ki: PWM integral gain (float).
 * @param pwm_Kd: PWM derivative gain (float).
 */
struct agressive_PID_t{
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    float pwm_Kp;
    float pwm_Ki;
    float pwm_Kd;
};

/**
 * @brief NOT USED.
 * 
 * @param 
 * @param 
 * @return 
 */
struct LQR_t{
    float Q;
    float R;
};

/**
 * @brief Calculate the PID control signal.
 * 
 * @param pid: Pointer to the PID struct.
 * @param input: Input to the PID controller (float).
 */
void calculate_PID(struct PID_t* pid, float input);


/**
 * @brief Update the PID parameters.
 * 
 * @param pid: Pointer to the PID struct.
 * @param new_Kp: New proportional gain (float).
 * @param new_Ki: New integral gain (float).
 * @param new_Kd: New derivative gain (float).
 */
void update_PID_parameters(struct PID_t* pid, float new_Kp, float new_Ki, float new_Kd);

/**
 * @brief Update the PID setpoint.
 * 
 * @param pid: Pointer to the PID struct.
 * @param new_setpoint: New setpoint (float).
 */
void update_PID_setpoint(struct PID_t* pid, float new_setpoint);

/**
 * @brief Feedforward PID controller.
 * 
 * @param pid: Pointer to the PID struct.
 * @param input: Input to the PID controller (float).
 */
void feedforward_PID(struct PID_t* pid, float input);

#endif

