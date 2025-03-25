/**

**/
#include "isr_handler.h"
#include "defines.h"
#include "sensors.h"
#include "robot_states.h"

// Extern pointer to the encoder struct, used as shared instance of the encoder struct
extern encoderData_t* encodePtr;

//Debounce
volatile unsigned long button_1_lastPressTime = 0;
volatile unsigned long button_2_lastPressTime = 0;
volatile unsigned long button_3_lastPressTime = 0;

/*
Force stop motorrs, press once to stop and press once again to start again.
*/
void IRAM_ATTR button1_isr(){
  unsigned long currentTime = millis();
  if (currentTime - button_1_lastPressTime > DEBOUNCE_DELAY) {
    button_1_lastPressTime = currentTime;
    USBSerial.printf("Button 1 pressed! Stopping the motors and going into idle mode.\n");
    force_stop_motor_commands();
  }
}

/*
If the code is running in WIFI_MODE OFF(0) button 2 is used to start/stop the main loop, press once to start(1 second delay) and press again to stop.
*/
void IRAM_ATTR button2_isr(){
  unsigned long currentTime = millis();
  if (currentTime - button_2_lastPressTime > DEBOUNCE_DELAY) {
    button_2_lastPressTime = currentTime;
    USBSerial.printf("Button 2 pressed!\n");
    #if WIFI_MODE == 0
      start_running();
    #endif
  }
}
/*
Not yet in use
*/
void IRAM_ATTR button3_isr(){
  unsigned long currentTime = millis();
  if (currentTime - button_3_lastPressTime > DEBOUNCE_DELAY) {
    button_3_lastPressTime = currentTime;
    USBSerial.printf("Button 3 pressed!.\n");
  }
}   

void IRAM_ATTR dip_isr(){
  int dipRead1 = digitalRead(DIP1);
  int dipRead2 = digitalRead(DIP2);
  USBSerial.printf("Dip switch pressed, configuration: (%d, %d)\n", dipRead1, dipRead2);
}

void IRAM_ATTR encoder1_isr(){
  if (digitalRead(encodePtr->ENC_1_A) == digitalRead(encodePtr->ENC_1_B)){
    encodePtr->ENC_1_tick++; // Forward
  }
  else{
    encodePtr->ENC_1_tick--; // Reverse
  }
}

void IRAM_ATTR encoder2_isr(){
  if (digitalRead(encodePtr->ENC_2_A) == digitalRead(encodePtr->ENC_2_B)){
    encodePtr->ENC_2_tick++; // Forward
  }
  else{
    encodePtr->ENC_2_tick--; // Reverse
  }
}

void setupEncoders(struct encoderData_t* encoders_struct){
  delay(3000);
  USBSerial.printf("Setting up encoder pins, Encoder 1 A: %d, Encoder 1 B: %d.\n", encoders_struct->ENC_1_A, encoders_struct->ENC_1_B);
  pinMode(encoders_struct->ENC_1_A, INPUT);
  pinMode(encoders_struct->ENC_1_B, INPUT);
  
  USBSerial.printf("Setting up encoder pins, Encoder 2 A: %d, Encoder 2 B: %d.\n", encoders_struct->ENC_1_A, encoders_struct->ENC_1_B);
  pinMode(encoders_struct->ENC_2_A, INPUT);
  pinMode(encoders_struct->ENC_2_B, INPUT);

  USBSerial.printf("Attaching ISR.\n");
  attachInterrupt(encoders_struct->ENC_1_A, encoder1_isr, CHANGE);
  attachInterrupt(encoders_struct->ENC_2_A, encoder2_isr, CHANGE);

  USBSerial.printf("Encoder setup done.\n");
}

void buttons_setup(){
    USBSerial.printf("Setting up buttons...\n");
    pinMode(BT1, INPUT);
    pinMode(BT2, INPUT);
    pinMode(BT3, INPUT);
    pinMode(DIP1, INPUT);
    pinMode(DIP2, INPUT);

    attachInterrupt(BT1, button1_isr, RISING);
    attachInterrupt(BT2, button2_isr, RISING);
    attachInterrupt(BT3, button3_isr, RISING);
    attachInterrupt(DIP1, dip_isr, CHANGE);
    attachInterrupt(DIP2, dip_isr, CHANGE);
}