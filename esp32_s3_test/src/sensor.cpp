/*


*/
#include <Wire.h>
#include "sensors.h"
#include "defines.h"

uint8_t buf[20] = {0x00};
uint16_t sens_vals[10] = {0};

void lineSensor_value_front(struct sensorData_t* sensorData){
  uint32_t num = 0;
  uint32_t denom = 0;
  Wire.beginTransmission(LEFT_ADR);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(LEFT_ADR, 10, 1);

  for (size_t i = 0; i < 10; i++){
    buf[i] = Wire.read();
  }

  Wire.beginTransmission(RIGHT_ADR);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(RIGHT_ADR, 10, 1);

  for (size_t i = 0; i < 10; i++){
    buf[i + 10] = Wire.read();
  }

  for (uint32_t i = 0; i < 10; i++){
    uint16_t val = (uint16_t)(buf[2 * i]) | (((uint16_t)(buf[2 * i + 1]) << 8) & 0xFF00);
    num += val * i * 1000;
    denom += val;
  }
  sensorData->lineSensor_value_front = (uint32_t)(num / denom);
  #if DEBUG_MODE == 1
    USBSerial.printf("Line sensor value front: %d\n", sensorData->lineSensor_value_front);
  #endif
}

void lineSensor_value_back(struct sensorData_t* sensorData){
  uint16_t sens_1 = analogRead(backSensor_1);
  uint16_t sens_2 = analogRead(backSensor_2);
  uint16_t sens_3 = analogRead(backSensor_3);
  uint16_t sens_4 = analogRead(backSensor_4);
  uint16_t sens_5 = analogRead(backSensor_5);
  int32_t result = (int32_t)((-sens_1*3 + -sens_2*2 + sens_3 + sens_4*2 + sens_5*3) / 5); // NOTE: Correct this, only for testing
  sensorData->lineSensor_value_back = result;

  #if DEBUG_MODE == 1
    USBSerial.printf("Line sensor value back: %d\n", sensorData->lineSensor_value_back);
  #endif
}

void update_encoder(struct sensorData_t* sensorData, struct encoderData_t* encoderData){
  sensorData->prev_leftEncoderTick = sensorData->leftEncoderTick;
  sensorData->prev_rightEncoderTick = sensorData->rightEncoderTick;
  sensorData->leftEncoderTick = encoderData->ENC_1_tick;
  sensorData->rightEncoderTick = encoderData->ENC_2_tick;
}

void calibrate_front_line_sensor(struct sensorData_t* sensorData){

}

void calibrate_back_line_sensor(struct sensorData_t* sensorData){
  
}
