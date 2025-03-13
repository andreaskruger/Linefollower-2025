
#include "initiate.h"
#include "defines.h"


void initiate_structs(struct sensorData_t* sensorData, struct encoderData_t* encoderData, struct PID_t* pwm_pid, struct PID_t* right_pwm_pid, struct PID_t* left_pwm_pid, struct PID_t* speed_pid, struct STD_PID_t* std_pid_values, struct positionData_t* positionData, struct robotStates_t* robotStates){
    sensorData->lineSensor_value_front = 0.0;
    sensorData->lineSensor_value_back = 0.0;
    sensorData->leftEncoderTick = 0;
    sensorData->rightEncoderTick = 0;
    sensorData->prev_leftEncoderTick = 0;
    sensorData->prev_rightEncoderTick = 0;

    encoderData->ENC_1_A = ENCODER_1_A;
    encoderData->ENC_1_B = ENCODER_1_B;
    encoderData->ENC_2_A = ENCODER_2_A;
    encoderData->ENC_2_B = ENCODER_2_B;
    encoderData->ENC_1_tick = 0;
    encoderData->ENC_2_tick = 0;
    
    pwm_pid->output = 0.0;
    pwm_pid->error = 0;
    pwm_pid->error_prev = 0;
    pwm_pid->error_sum = 0;
    pwm_pid->Kp = std_pid_values->pwm_Kp;
    pwm_pid->Ki = std_pid_values->pwm_Ki;
    pwm_pid->Kd = std_pid_values->pwm_Kd;

    right_pwm_pid->output = 0.0;
    right_pwm_pid->error = 0;
    right_pwm_pid->error_prev = 0;
    right_pwm_pid->error_sum = 0;
    right_pwm_pid->Kp = std_pid_values->pwm_Kp;
    right_pwm_pid->Ki = std_pid_values->pwm_Ki;
    right_pwm_pid->Kd = std_pid_values->pwm_Kd;

    left_pwm_pid->output = 0.0;
    left_pwm_pid->error = 0;
    left_pwm_pid->error_prev = 0;
    left_pwm_pid->error_sum = 0;
    left_pwm_pid->Kp = std_pid_values->pwm_Kp;
    left_pwm_pid->Ki = std_pid_values->pwm_Ki;
    left_pwm_pid->Kd = std_pid_values->pwm_Kd;

    speed_pid->output = 0.0;
    speed_pid->error = 0;
    speed_pid->error_prev = 0;
    speed_pid->error_sum = 0;
    speed_pid->Kp = std_pid_values->speed_Kp;
    speed_pid->Ki = std_pid_values->speed_Ki;
    speed_pid->Kd = std_pid_values->speed_Kd;

    positionData->x_pos = 0.0;
    positionData->y_pos = 0.0;
    positionData->theta = 0.0;
    positionData->omega = 0.0;
    positionData->V = 0.0;
    positionData->d = WHEEL_DIAMETER;
    positionData->Vl = 0.0;
    positionData->Vr = 0.0;
    positionData->dt = 1;

    robotStates->x_pos = 0.0;
    robotStates->y_pos = 0.0;
    robotStates->velocity = 0.0;
    robotStates->baseSpeed = BASE_SPEED;
    robotStates->lineSensor_value_front = 0.0;
    robotStates->lineSensor_value_back = 0.0;
    robotStates->left_controlSignal = 0.0;
    robotStates->right_controlSignal = 0.0;
    robotStates->message = "";
}

