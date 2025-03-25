/*


*/
#include "robot_states.h"
#include "defines.h"
#include "macros.h"
#include "sensors.h"

/*Used to force stop the motors with button 1*/
force_stop_commands_t RUN_STATE = RUNNING;

/*Used to start the running command when no wifi is available by pressing button 2*/
no_wifi_commands_t NO_WIFI_COMMAND = STOP_RUNNING;

void calculate_Position(struct positionData_t* positionData){
    positionData->V = (positionData->Vl + positionData->Vr)/2.0f;
    positionData->omega_robot = (positionData->Vr - positionData->Vl)/ROBOT_WIDTH;
    positionData->theta = positionData->omega_robot*positionData->dt;
    positionData->x_pos += positionData->V*cos(positionData->theta)*positionData->dt;
    positionData->y_pos += positionData->V*sin(positionData->theta)*positionData->dt;
    positionData->theta += positionData->omega_robot*positionData->dt;

    #if DEBUG_MODE == 1
        USBSerial.printf("X position: %f\n", positionData->x_pos);
        USBSerial.printf("Y position: %f\n", positionData->y_pos);
    #endif
}

void calculate_velocity(struct positionData_t* positionData, struct sensorData_t* sensorData){
    positionData->omega_l = (PI*((float)(sensorData->leftEncoderTick - sensorData->prev_leftEncoderTick)*sensorData->dt))/MOTOR_POLES;
    positionData->omega_r = (PI*((float)(sensorData->rightEncoderTick - sensorData->prev_rightEncoderTick)*sensorData->dt))/MOTOR_POLES;
    positionData->Vl = positionData->omega_l*WHEEL_DIAMETER;
    positionData->Vr = positionData->omega_r*WHEEL_DIAMETER;
    sensorData->prev_leftEncoderTick = sensorData->leftEncoderTick;
    sensorData->prev_rightEncoderTick = sensorData->rightEncoderTick;

    #if DEBUG_MODE == 1
        USBSerial.printf("Right wheel velocity: %f\n", positionData->Vr);
        USBSerial.printf("Left wheel velocity: %f\n", positionData->Vl);
    #endif
}

void update_robotStates(struct sensorData_t* sensorData, positionData_t* position_states, struct robotStates_t* robot_states, int32_t controlSignal_left, int32_t controlSignal_right){
    int32_t base_speed = robot_states->baseSpeed;
    calculate_velocity(position_states, sensorData);
    calculate_Position(position_states);
    robot_states->lineSensor_value_front = sensorData->lineSensor_value_front;
    robot_states->lineSensor_value_back = sensorData->lineSensor_value_back;
    robot_states->left_controlSignal = base_speed - controlSignal_left;    // NOTE: Check this, something strange with base_speed, it its not working try "BASE_SPEED" instead of "base_speed"
    robot_states->right_controlSignal = base_speed + controlSignal_right;   // NOTE: Check this, something strange with base_speed, it its not working try "BASE_SPEED" instead of "base_speed"
    robot_states->velocity = position_states->V;
    robot_states->x_pos = position_states->x_pos;
    robot_states->y_pos = position_states->y_pos;
}

void set_motor_commands(int stopCommand, int motor_index, int motor_pwm){
    motor_pwm = CLAMP(motor_pwm, -2048, 2048);
    #if DEBUG_MODE == 1
        USBSerial.printf("Setting motor: %d, to PWM: %d. The stop command is set to: %d\n", motor_index, motor_pwm, stopCommand);
    #endif
    if (stopCommand == STOP or RUN_STATE == STOPPING){
        analogWrite(MOTL_1, 0);
        analogWrite(MOTL_2, 0);
        analogWrite(MOTR_1, 0);
        analogWrite(MOTR_2, 0);
    }
    else{ // NOTE: Check if this is correct, done very quickly
        if (motor_index == LEFT_MOTOR){
            if (motor_pwm > 0){
                analogWrite(MOTL_1, 0);
                analogWrite(MOTL_2, motor_pwm);
            }
            else{
                analogWrite(MOTL_1, -motor_pwm);
                analogWrite(MOTL_2, 0);
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

void force_stop_motor_commands(void){
    if (RUN_STATE == RUNNING){
        RUN_STATE = STOPPING;
        set_motor_commands(STOP, LEFT_MOTOR, 0);
        set_motor_commands(STOP, RIGHT_MOTOR, 0);
    } else{
        RUN_STATE = RUNNING;
    }
}

void start_running(){
    if (NO_WIFI_COMMAND == START_RUNNING){
        NO_WIFI_COMMAND = STOP_RUNNING;
    }
    else{
        delay(1000);
        NO_WIFI_COMMAND = START_RUNNING;
    }
}

int32_t get_no_wifi_command(void){
    int32_t return_command;
    if (NO_WIFI_COMMAND == START_RUNNING){
        return_command = 1;
    }
    else{
        return_command = 0;
    }
    return return_command;
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

void update_base_speed(struct robotStates_t* robotStates, int32_t new_baseSpeed){
    USBSerial.printf("Setting new base speed. Base speed: %d\n", new_baseSpeed);
    robotStates->baseSpeed = new_baseSpeed;
}