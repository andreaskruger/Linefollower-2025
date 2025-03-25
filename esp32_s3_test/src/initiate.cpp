
#include "initiate.h"
#include "defines.h"


void initiate_structs(struct sensorData_t* sensorData, struct encoderData_t* encoderData, struct PID_t* pwm_pid, struct PID_t* right_pwm_pid, struct PID_t* left_pwm_pid, struct PID_t* speed_pid, struct STD_PID_t* std_pid_values, struct positionData_t* positionData, struct robotStates_t* robotStates, struct lowPassFilter_t* lowPassFilter_front, struct lowPassFilter_t* lowPassFilter_back){
    USBSerial.printf("Initiating sensor data struct...\n");
    sensorData->prev_leftEncoderTick = 0;
    sensorData->prev_rightEncoderTick = 0;
    sensorData->leftEncoderTick = 0;
    sensorData->rightEncoderTick = 0;
    sensorData->lineSensor_value_front = 0.0;
    sensorData->lineSensor_value_back = 0.0;
    sensorData->start_adddress_calibration_front = FRONT_SENSOR_EPROM_ADR;
    sensorData->start_adddress_calibration_back = BACK_SENSOR_EPROM_ADR;
    sensorData->front_sensor_size = NR_FRONT_PIXELS;
    sensorData->back_sensor_size = NR_BACK_PIXELS;
    sensorData->dt = DT;
    sensorData->total_value = 0;

    USBSerial.printf("Initiating encoder data struct...\n");
    encoderData->ENC_1_A = ENCODER_1_A;
    encoderData->ENC_1_B = ENCODER_1_B;
    encoderData->ENC_2_A = ENCODER_2_A;
    encoderData->ENC_2_B = ENCODER_2_B;
    encoderData->ENC_1_tick = 0;
    encoderData->ENC_2_tick = 0;
    encoderData->tick_rate_of_change = 0.0;

    // Only used in the "Simple controller"
    USBSerial.printf("Initiating PWM-PID struct...\n");
    pwm_pid->name = "PWM-PID";
    pwm_pid->setpoint = STD_SETPOINT;
    pwm_pid->output = 0.0;
    pwm_pid->error = 0;
    pwm_pid->error_prev = 0;
    pwm_pid->error_sum = 0;
    pwm_pid->Kp = std_pid_values->pwm_Kp;
    pwm_pid->Ki = std_pid_values->pwm_Ki;
    pwm_pid->Kd = std_pid_values->pwm_Kd;

    // Only used in the "Advanced controller"
    USBSerial.printf("Initiating right PWM-PID struct...\n");
    right_pwm_pid->name = "Right PWM-PID";
    right_pwm_pid->setpoint = STD_SETPOINT;
    right_pwm_pid->output = 0.0;
    right_pwm_pid->error = 0;
    right_pwm_pid->error_prev = 0;
    right_pwm_pid->error_sum = 0;
    right_pwm_pid->Kp = std_pid_values->pwm_Kp;
    right_pwm_pid->Ki = std_pid_values->pwm_Ki;
    right_pwm_pid->Kd = std_pid_values->pwm_Kd;

    // Only used in the "Advanced controller"
    USBSerial.printf("Initiating left PWM-PID struct...\n");
    left_pwm_pid->name = "Left PWM-PID";
    left_pwm_pid->setpoint = STD_SETPOINT;
    left_pwm_pid->output = 0.0;
    left_pwm_pid->error = 0;
    left_pwm_pid->error_prev = 0;
    left_pwm_pid->error_sum = 0;
    left_pwm_pid->Kp = std_pid_values->pwm_Kp;
    left_pwm_pid->Ki = std_pid_values->pwm_Ki;
    left_pwm_pid->Kd = std_pid_values->pwm_Kd;

    // Only used in the "Advanced controller"
    USBSerial.printf("Initiating speed-PID struct...\n");
    speed_pid->name = "Speed-PID";
    speed_pid->setpoint = BASE_SPEED;
    speed_pid->output = 0.0;
    speed_pid->error = 0;
    speed_pid->error_prev = 0;
    speed_pid->error_sum = 0;
    speed_pid->Kp = std_pid_values->speed_Kp;
    speed_pid->Ki = std_pid_values->speed_Ki;
    speed_pid->Kd = std_pid_values->speed_Kd;

    USBSerial.printf("Initiating position data struct...\n");
    positionData->x_pos = 0.0;
    positionData->y_pos = 0.0;
    positionData->theta = 0.0;
    positionData->omega_l = 0.0;
    positionData->omega_r = 0.0;
    positionData->omega_robot = 0.0;
    positionData->V = 0.0;
    positionData->d = WHEEL_DIAMETER;
    positionData->Vl = 0.0;
    positionData->Vr = 0.0;
    positionData->dt = DT;

    USBSerial.printf("Initiating robot states struct...\n");
    robotStates->x_pos = 0.0;
    robotStates->y_pos = 0.0;
    robotStates->velocity = 0.0;
    robotStates->baseSpeed = BASE_SPEED;
    robotStates->lineSensor_value_front = 0.0;
    robotStates->lineSensor_value_back = 0.0;
    robotStates->left_controlSignal = 0.0;
    robotStates->right_controlSignal = 0.0;
    robotStates->message = "";

    USBSerial.printf("Initiating low pass filter struct...\n");
    
    lowPassFilter_front->alpha = LP_ALPHA;
    lowPassFilter_front->output = 0.0;
    lowPassFilter_back->alpha = LP_ALPHA;
    lowPassFilter_back->output = 0.0;
}
