/*


*/
#include "robot_states.h"
#include "defines.h"
#include "sensors.h"


void calculate_Position(struct positionData_t* positionData){
    positionData->omega = (positionData->Vr - positionData->Vl)/(2*positionData->d);
    positionData->V = (positionData->Vr + positionData->Vl)/2;
    positionData->x_pos += (positionData->V*cos(positionData->omega))*positionData->dt;
    positionData->y_pos += (positionData->V*sin(positionData->omega))*positionData->dt;
    positionData->theta += positionData->omega*positionData->dt;
}

void calculate_velocity(struct positionData_t* positionData, struct sensorData_t* sensorData){
    positionData->Vl = (WHEEL_CIRCUMFERENCE_SEGMENT*(sensorData->leftEncoderTick - sensorData->prev_leftEncoderTick)); // Add DT
    positionData->Vr = (WHEEL_CIRCUMFERENCE_SEGMENT*(sensorData->rightEncoderTick - sensorData->prev_rightEncoderTick)); // Add DT
    sensorData->prev_leftEncoderTick = sensorData->leftEncoderTick;
    sensorData->prev_rightEncoderTick = sensorData->rightEncoderTick;
}

void update_robotStates(struct sensorData_t* sensorData, positionData_t* position_states, struct robotStates_t* robot_states, float front_sensor_value, float back_sensor_value, int32_t controlSignal_left, int32_t controlSignal_right){
    calculate_velocity(position_states, sensorData);
    calculate_Position(position_states);
    robot_states->lineSensor_value_front = front_sensor_value;
    robot_states->lineSensor_value_back = back_sensor_value;
    robot_states->left_controlSignal = robot_states->baseSpeed - controlSignal_left; // NOTE: Check if this is correct, no idea if it is + or -
    robot_states->right_controlSignal = robot_states->baseSpeed + controlSignal_right; // NOTE: Check if this is correct, no idea if it is + or -
    robot_states->velocity = position_states->V;
    robot_states->x_pos = position_states->x_pos;
    robot_states->y_pos = position_states->y_pos;
}

void set_motor_commands(int stopCommand, int motor_index, int motor_pwm){
    if (stopCommand == STOP){
        analogWrite(MOTL_1, 0);
        analogWrite(MOTL_2, 0);
        analogWrite(MOTR_1, 0);
        analogWrite(MOTR_2, 0);
    }
    else{ // NOTE: Check if this is correct, done very quickly
        if (motor_index == LEFT_MOTOR){
            if (motor_pwm > 0){
                analogWrite(MOTL_1, motor_pwm);
                analogWrite(MOTL_2, 0);
            }
            else{
                analogWrite(MOTL_1, 0);
                analogWrite(MOTL_2, -motor_pwm);
            }
        }
        else if (motor_index == RIGHT_MOTOR){
            if (motor_pwm > 0){
                analogWrite(MOTR_1, 0);
                analogWrite(MOTR_2, motor_pwm);
            }
            else{
                analogWrite(MOTR_1, -motor_pwm);
                analogWrite(MOTR_2, 0);
            }
        }
    }
}

void stop_motor_commands(void){
    set_motor_commands(STOP, LEFT_MOTOR, 0);
    set_motor_commands(STOP, RIGHT_MOTOR, 0);
}

void line_lost_reverse(void){
    set_motor_commands(RUN, LEFT_MOTOR, 0);
    set_motor_commands(RUN, RIGHT_MOTOR, 0);
}

void motor_pins_setup(){
    USBSerial.printf("Setting up motor pins...\n");
    pinMode(MOTL_1, OUTPUT);
    pinMode(MOTL_2, OUTPUT);
    pinMode(MOTR_1, OUTPUT);
    pinMode(MOTR_2, OUTPUT);
}