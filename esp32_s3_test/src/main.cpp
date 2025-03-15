#include <Arduino.h>
#include <Wire.h>
#include "WiFi.h"
#include "esp_timer.h"
#include "isr_handler.h"
#include "communication.h"
#include "sensors.h"
#include "controllers.h"
#include "robot_states.h"
#include "defines.h"
#include "initiate.h"

// AP settigns and credentials
const char *AP_SSID = "MyESP32AP";
const char *AP_PASSWORD = "12345678";

// The TCP server will listen on port 3333 (arbitrary choice)
WiFiServer server(3333);
WiFiClient client;
hw_timer_t *My_timer = NULL;

// State for PID controller scheduler
typedef enum{
    LINE_REGULAR,
    LINE_LOST,
    LINE_STRAIGHT,
    LINE_CURVE,
    LINE_SHARP_CURVE
} line_state_t;
line_state_t line_state = LINE_REGULAR;

// Main FSM
typedef enum{
    STATE_INIT,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_FINISHED,
    STATE_CALIBRATING,
    STATE_TESTING_PWM,
    STATE_STOP,
    STATE_CLIENT_DISCONNECTED,
    STATE_LINE_LOST_REVERSE
} system_state_t;
system_state_t current_state = STATE_INIT;

// DIP switch configurations
typedef enum{
    UNDECLARED,
    DIP_00,
    DIP_01,
    DIP_10,
    DIP_11
} dip_switch_config_t;
dip_switch_config_t dip_configuration = UNDECLARED;

// Create structs
struct sensorData_t sensorData;
struct encoderData_t encoders_struct;
struct PID_t pwm_pid; // Remove this and use left/right pwm pids
struct PID_t left_pwm_pid; 
struct PID_t right_pwm_pid;
struct PID_t speed_pid;
struct positionData_t positionData;
struct robotStates_t robotStates;
struct messageFields_t messageParts;
struct lowPassFilter_t LP_front_sensor_filter;
struct STD_PID_t std_pid_values = {STD_SPEED_KP,
                                   STD_SPEED_KI, 
                                   STD_SPEED_KD, 
                                   STD_PWM_KP, 
                                   STD_PWM_KI, 
                                   STD_PWM_KD};

// Create a pointer to the encoder struct to make it a shared resource
struct encoderData_t* encodePtr = &encoders_struct;

// count the nr of timer inputs to create a slower PID
int32_t timer_counter = 0;

void startRunning(){
    USBSerial.printf("Start command received!\n");
    current_state = STATE_FINISHED;
    sendState(client, (int32_t)current_state);
}

void stopRunning(){
    USBSerial.printf("Stop command received!\n");
    current_state = STATE_STOP;
}

void client_disconnected(){
    current_state = STATE_CLIENT_DISCONNECTED;
}

void IRAM_ATTR onTimer(){
    if (current_state == STATE_FINISHED){
        current_state = STATE_RUNNING;
        timer_counter +=1;
    }
    else if (current_state == STATE_IDLE){
        //Do nothing
    }
    else if (current_state == STATE_INIT){
        //Do nothing
    }
    else if (current_state == STATE_CALIBRATING){
        // Do nothing
    }
    else if (current_state == STATE_TESTING_PWM){
        // Do nothing
    }
    else if (current_state == STATE_CLIENT_DISCONNECTED){
        current_state = STATE_IDLE;
    }
    else{
        Serial.printf("The running function is not yet completed, go to error state\n");
        current_state = STATE_ERROR;
    }
}

void read_dipSwitchConfig(){
    USBSerial.printf("Reading switches for configuration...\n");
    int32_t dip_switch_1 = digitalRead(DIP1);
    int32_t dip_switch_2 = digitalRead(DIP2);
    if (dip_switch_1 == HIGH && dip_switch_2 == HIGH){
        dip_configuration = DIP_11;
    }else if (dip_switch_1 == HIGH && dip_switch_2 == LOW){
        dip_configuration = DIP_10;
    }else if (dip_switch_1 == LOW && dip_switch_2 == HIGH){
        dip_configuration = DIP_01;
    }else if (dip_switch_1 == LOW && dip_switch_2 == LOW){
        dip_configuration = DIP_00;
    } else{
        dip_configuration = UNDECLARED;
    }
    USBSerial.printf("Switch configuration: (%d, %d)\n", dip_switch_1, dip_switch_2);
}

void run_idle(){
    int32_t returnCode = receiveMessage(client, &messageParts);
    // Remake this into a switch statement
    if (returnCode == 8){
        calibrate_front_line_sensor(&sensorData);
        calibrate_back_line_sensor(&sensorData);
    }
    else if (returnCode == 7){
        set_motor_commands(RUN, LEFT_MOTOR, messageParts.floatPart_1);
        set_motor_commands(RUN, RIGHT_MOTOR, messageParts.floatPart_1);
    }
    else if (returnCode == 6){
        sendState(client, (int32_t)current_state);
    }
    else if (returnCode == 5){
        sendPIDparams(client, &pwm_pid, &speed_pid);
    }
    else if(returnCode == 4){
        update_PID_parameters(&speed_pid, messageParts.floatPart_1, messageParts.floatPart_2, messageParts.floatPart_3);
    }
    else if(returnCode == 3){
        update_PID_parameters(&pwm_pid, messageParts.floatPart_1, messageParts.floatPart_2, messageParts.floatPart_3);
    }
    else if (returnCode == 2){
        stopRunning();
    }
    else if (returnCode == 1){
        startRunning();
    }
    else if(returnCode == -1){
        current_state = STATE_IDLE;
    }
}

void run_simple(){
    int32_t returnCode = receiveMessage(client, &messageParts);
    if (returnCode == 2){
        stopRunning();
    }
    else{
        lineSensor_value_front(&sensorData);
        lineSensor_value_back(&sensorData);
        update_encoder(&sensorData, &encoders_struct);
        if (timer_counter <= 9){
            timer_counter = 0;
        }
        calculate_PID(&pwm_pid, sensorData.lineSensor_value_front);

        update_robotStates(&sensorData, &positionData, &robotStates, pwm_pid.output, pwm_pid.output);
        set_motor_commands(RUN, LEFT_MOTOR, robotStates.left_controlSignal);
        set_motor_commands(RUN, RIGHT_MOTOR, robotStates.right_controlSignal);
        sendMessage(&robotStates, client);
        if (robotStates.lineSensor_value_front == ALL_BLACK_VALUE){// Make this a range and more robust so not just a "unlucky" sensor reading is a false finish line.
            stop_motor_commands();
            send_finishLine_found(client);
            current_state = STATE_IDLE;
        }else{
            current_state = STATE_FINISHED;
        }
    }
}

void run_advanced(){
    int32_t returnCode = receiveMessage(client, &messageParts);
    if (returnCode == 2){
        stopRunning();
    }
    else{
        lineSensor_value_front(&sensorData);
        lineSensor_value_back(&sensorData);
        update_encoder(&sensorData, &encoders_struct);
        if (timer_counter <= 9){
            calculate_PID(&speed_pid, sensorData.lineSensor_value_front);       // This one should be 10x slower than the pwm PIDs
            timer_counter = 0;
        }
        calculate_PID(&left_pwm_pid, speed_pid.output);
        calculate_PID(&right_pwm_pid, speed_pid.output);

        update_robotStates(&sensorData, &positionData, &robotStates, left_pwm_pid.output, right_pwm_pid.output);
        set_motor_commands(RUN, LEFT_MOTOR, robotStates.left_controlSignal);
        set_motor_commands(RUN, RIGHT_MOTOR, robotStates.right_controlSignal);
        sendMessage(&robotStates, client);
        if (robotStates.lineSensor_value_front == ALL_BLACK_VALUE){// Make this a range and more robust so not just a "unlucky" sensor reading is a false finish line.
            stop_motor_commands();
            send_finishLine_found(client);
            current_state = STATE_IDLE;
        }else{
            current_state = STATE_FINISHED;
        }
    }
}

void setupAccessPoint(){
    // Configure ESP32 as AP
    USBSerial.printf("Setting up access point\n");
    WiFi.mode(WIFI_AP);
    // Start AP
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    // Start the TCP server
    USBSerial.printf("Starting TCP server...\n");
    server.begin();

    // For debugging
    USBSerial.printf("Access Point SSID: ");
    USBSerial.print(AP_SSID);
    USBSerial.printf("... Started!\n");
    USBSerial.printf("IP address: ");
    USBSerial.println(WiFi.softAPIP());
}

void setup() {
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    USBSerial.begin(115200);
    delay(4000);
    USBSerial.printf("Starting setup...\n");

    buttons_setup();
    motor_pins_setup();
    read_dipSwitchConfig();

    if (dip_configuration == DIP_11 or DIP_10){
        setupAccessPoint();
        USBSerial.printf("Waiting for client to connect... ");
        while (true){
            if (!client || !client.connected()) {
                WiFiClient newClient = server.available();
                if (newClient) {
                    client = newClient;
                    USBSerial.printf("New client connected!\n");
                    break;
                }
            }
            delay(500);
        }
    }else{
        USBSerial.printf("Access point not enabled, going to idle mode with no WiFi connection...\n");
    }

    // Setup encoders
    USBSerial.printf("Setting up resolution and frequency for encoders...\n");
    analogWriteResolution(ENCODER_RESOLUTION);
    analogWriteFrequency(ENCODER_FREQUENCY);
    initiate_structs(&sensorData, &encoders_struct, &pwm_pid, &left_pwm_pid, &right_pwm_pid, &speed_pid, &std_pid_values, &positionData, &robotStates, &LP_front_sensor_filter);
    setupEncoders(&encoders_struct);

    // Setup timer for interupt to run the main fsm
    USBSerial.printf("Setting up timers...\n");
    My_timer = timerBegin(0, TIMER_PRESCALER, true);
    timerAttachInterrupt(My_timer, &onTimer, true);
    timerAlarmWrite(My_timer, TIMER_FREQUENCY, true);
    timerAlarmEnable(My_timer);
    USBSerial.printf("Setup complete...\n");
    USBSerial.printf("Going to idle mode and waiting for instructions...\n");
    current_state = STATE_IDLE;
}

void loop() {
    switch (current_state){
    case STATE_INIT:
        current_state = STATE_IDLE;
        break;
    case STATE_IDLE:
        run_idle();
        break;
    case STATE_RUNNING:
        if (dip_configuration == DIP_11){
            run_simple();
        }else if(dip_configuration == DIP_10){
            run_advanced();
        }
        break;
    case STATE_FINISHED:
        break;
    case STATE_STOP:
        stop_motor_commands();
        current_state = STATE_IDLE;
        sendState(client, (int32_t)current_state);
        break;
    default:
        break;
    }
}