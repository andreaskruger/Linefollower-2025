/*


*/
#include "controllers.h"
#include "defines.h"
#include "filters.h"

void calculate_PID(struct PID_t* pid, struct lowPassFilter_t* filter, int32_t input){
    float err = (float)(pid->setpoint - input);
    pid->error += err;
    
    float P = pid->Kp * err;
    float D = pid->Kd * (err - (float)pid->error_prev);
    applyLPFilter(filter, D);
    float I = pid->Ki * (pid->error);

    int32_t out = (int32_t)(P + filter->output);
    pid->error_prev = err;
    pid->output = out;
}

void update_PID_setpoint(struct PID_t* pid, float new_setpoint){
    USBSerial.printf("Setting new PID setpoint. Setpoint: %d\n", new_setpoint);
    pid->setpoint = new_setpoint;
}

void update_PID_parameters(struct PID_t* pid, float new_Kp, float new_Ki, float new_Kd){
    USBSerial.printf("Setting new PID parameters for %s -> Kp: %f, Ki: %f, Kd: %f\n", pid->name.c_str(), new_Kp, new_Ki, new_Kd);
    pid->Kp = new_Kp;
    pid->Ki = new_Ki;
    pid->Kd = new_Kd;
}

void feedforward_PID(struct PID_t* pid, float input){
    
}