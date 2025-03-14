/*


*/
#include "controllers.h"
#include "defines.h"

void calculate_PID(struct PID_t* pid, float input){
    static float prev_err = 0;
    float err = (float)pid->setpoint - (float)input;
    float P = pid->Kp * err;
    float D = pid->Kd * (err - prev_err);
    int32_t out = (int32_t)(P + D);
    prev_err = err;

    pid->error = err;
    pid->error_prev = prev_err;
    pid->output = out;
}

void update_PID_setpoint(struct PID_t* pid, float new_setpoint){
    USBSerial.printf("Setting new PID setpoint. Setpoint: %d\n", new_setpoint);
    pid->setpoint = new_setpoint;
}

void update_PID_parameters(struct PID_t* pid, float new_Kp, float new_Ki, float new_Kd){
    USBSerial.printf("Setting new PID parameters -> Kp: %f, Ki: %f, Kd: %f\n", new_Kp, new_Ki, new_Kd);
    pid->Kp = new_Kp;
    pid->Ki = new_Ki;
    pid->Kd = new_Kd;
}

void feedforward_PID(struct PID_t* pid, float input){
    
}
