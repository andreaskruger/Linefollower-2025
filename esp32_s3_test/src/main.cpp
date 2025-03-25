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
// const char *AP_SSID = "SpeedyBoiiNetwork";
const char *AP_SSID = "SpeedyBoii";
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
    DIP_00, // Nothing yet
    DIP_01, // Nothing yet
    DIP_10, // Run advanced control loop
    DIP_11  // Run simple control loop
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
struct lowPassFilter_t LP_back_sensor_filter;
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
    //sendState(client, (int32_t)current_state);
}

void stopRunning(){
    USBSerial.printf("Stop command received!\n");
    current_state = STATE_STOP;
    //sendState(client, (int32_t)current_state);
}

void client_disconnected(){
    USBSerial.printf("Disconnecting from client...\n");
    client.stop();
    current_state = STATE_CLIENT_DISCONNECTED;
}

void IRAM_ATTR onTimer(){
    if (current_state == STATE_FINISHED){
        current_state = STATE_RUNNING;
        timer_counter += 1;
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
        // Do nothing
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
    #if WIFI_MODE == 1
        int32_t returnCode = receiveMessage(client, &messageParts);
        // Remake this into a switch statement
        if (returnCode == 13){
            update_base_speed(&robotStates, (int32_t)messageParts.floatPart_1);
        }
        else if (returnCode == 12){
            update_PID_parameters(&right_pwm_pid, messageParts.floatPart_1, messageParts.floatPart_2, messageParts.floatPart_3);
        }
        else if (returnCode == 11){
            update_PID_parameters(&left_pwm_pid, messageParts.floatPart_1, messageParts.floatPart_2, messageParts.floatPart_3);
        }
        else if (returnCode == 10){
            client_disconnected();
        }
        else if (returnCode == 8){
            calibrate_front_line_sensor(&sensorData);
            calibrate_back_line_sensor(&sensorData);
        }
        else if (returnCode == 7){
            set_motor_commands(RUN, LEFT_MOTOR, messageParts.floatPart_1);
            set_motor_commands(RUN, RIGHT_MOTOR, messageParts.floatPart_1);
        }
        else if (returnCode == 6){
            //sendState(client, (int32_t)current_state);
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
        else if(returnCode == 0){
            // Do nothing
        }
        else if(returnCode == -1){
            current_state = STATE_IDLE;
        }
    #endif
    #if WIFI_MODE == 0
        int32_t command = get_no_wifi_command();
        if (command == 1){
            startRunning();
        }
        else{
            current_state = STATE_STOP;
        }
    #endif
}

void run_simple(){
    #if WIFI_MODE == 1
        int32_t returnCode = receiveMessage(client, &messageParts);
    #else
        int32_t returnCode = 0;
    #endif
    if (returnCode == 2){
        stopRunning();
    }else if (returnCode == 10){
        current_state = STATE_CLIENT_DISCONNECTED;
    }else{
        //lineSensor_value_front(&sensorData, LEFT_ADR, 0, 10);
        //lineSensor_value_back(&sensorData);
        //update_encoder(&sensorData, &encoders_struct);
        if (timer_counter <= 9){
            timer_counter = 0;
            sendMessage(&robotStates, client);
        }
        calculate_PID(&pwm_pid, &LP_front_sensor_filter, sensorData.lineSensor_value_front);

        update_robotStates(&sensorData, &positionData, &robotStates, timer_counter, timer_counter*2);
        set_motor_commands(RUN, LEFT_MOTOR, robotStates.left_controlSignal);
        set_motor_commands(RUN, RIGHT_MOTOR, robotStates.right_controlSignal);
        if (sensorData.total_value > ALL_BLACK_VALUE){
            current_state = STATE_STOP;
            #if WIFI_MODE == 1
                //send_finishLine_found(client);
            #endif
        }else{
            current_state = STATE_FINISHED;
        }
    }
}

void run_advanced(){
    int32_t returnCode = receiveMessage(client, &messageParts);;
    if (returnCode == 2){
        stopRunning();
    }else{
        lineSensor_value_front(&sensorData, LEFT_ADR, 0, 10);
        lineSensor_value_back(&sensorData);
        update_encoder(&sensorData, &encoders_struct);
        if (timer_counter >= 9){
            timer_counter = 0;
            calculate_PID(&speed_pid, &LP_front_sensor_filter, sensorData.lineSensor_value_front);
            update_PID_setpoint(&left_pwm_pid, speed_pid.output);
            update_PID_setpoint(&right_pwm_pid, speed_pid.output);
            sendMessage(&robotStates, client);
        }

        /*
        NOTE: Vl/Vr is the calculated velocity of the left wheel, can be changed to ticks/s or something else.
        Encoder data stored in sensorData struct. Vl/Vr calculated in calculate_velocity() in robot_states.cpp
        Examples: 
            encoderChange = (sensorData.leftEncoderTick - sensorData.prev_leftEncoderTick)/DT;
            Vl = ((PI*((float)(sensorData->leftEncoderTick - sensorData->prev_leftEncoderTick)*sensorData->dt))/MOTOR_POLES)*WHEEL_DIAMETER;

        */
        calculate_PID(&left_pwm_pid, &LP_front_sensor_filter, positionData.Vl);
        calculate_PID(&right_pwm_pid, &LP_front_sensor_filter, positionData.Vr);
        update_robotStates(&sensorData, &positionData, &robotStates, left_pwm_pid.output, right_pwm_pid.output);
        set_motor_commands(RUN, LEFT_MOTOR, robotStates.left_controlSignal);
        set_motor_commands(RUN, RIGHT_MOTOR, robotStates.right_controlSignal);
        if (sensorData.total_value > ALL_BLACK_VALUE){
            current_state = STATE_STOP;
            #if WIFI_MODE == 1
                //send_finishLine_found(client);
            #endif
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
    init_i2c_frontSensor();
    USBSerial.begin(BAUD_RATE);
    USBSerial.printf("Starting setup...\n");
    USBSerial.printf("Loop delta time: %f\n", DT);

    buttons_setup();
    motor_pins_setup();
    read_dipSwitchConfig();

    #if WIFI_MODE == 1
        USBSerial.printf("WiFi mode is ON, setting up WiFi configs.\n");
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
    #endif

    // Setup encoders
    USBSerial.printf("Setting up resolution and frequency for encoders...\n");
    analogWriteResolution(ENCODER_RESOLUTION);
    analogWriteFrequency(ENCODER_FREQUENCY);
    initiate_structs(&sensorData, &encoders_struct, &pwm_pid, &left_pwm_pid, &right_pwm_pid, &speed_pid, &std_pid_values, &positionData, &robotStates, &LP_front_sensor_filter, &LP_back_sensor_filter);
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
        run_simple();
        // if (dip_configuration == DIP_11){
        //     run_simple();
        // }else if(dip_configuration == DIP_10){
        //     run_advanced();
        // }
        // break;
    case STATE_FINISHED:
        break;
    case STATE_STOP:
        stop_motor_commands();
        current_state = STATE_IDLE;
        break;
    case STATE_CLIENT_DISCONNECTED:
        #if WIFI_MODE == 1
            stop_motor_commands();
            if (!client || !client.connected()) {
                WiFiClient newClient = server.available();
                if (newClient) {
                    client = newClient;
                    USBSerial.printf("New client connected!\n");
                    current_state = STATE_IDLE;
                    break;
                }
            }
        #endif
        break;
    default:
        break;
    }
}