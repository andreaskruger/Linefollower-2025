/**
* Description:
* 
* Note:
* 
* Usage:
* 
**/
#include "isr_handler.h"
#include "defines.h"
#include "sensors.h"

// Extern pointer to the encoder struct, used as shared instance of the encoder struct
extern encoderData_t* encodePtr;

void IRAM_ATTR button1_isr(){
  USBSerial.printf("Button 1 pressed\n");
}

void IRAM_ATTR button2_isr(){
  USBSerial.printf("Button 2 pressed\n");
}

void IRAM_ATTR button3_isr(){
  USBSerial.printf("Button 3 pressed\n");
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