/*


*/
#include <Wire.h>
#include <EEPROM.h>
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

  for (size_t i = 0; i < sensorData->front_sensor_size; i++){
    buf[i] = Wire.read();
  }

  Wire.beginTransmission(RIGHT_ADR);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(RIGHT_ADR, 10, 1);

  for (size_t i = 0; i < sensorData->front_sensor_size; i++){
    buf[i + 10] = Wire.read();
  }

  for (size_t i = 0; i < sensorData->front_sensor_size; i++){
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
  int32_t back_buffer[5] = {0x00};
  int32_t num = 1;
  int32_t denom = 1;
  back_buffer[0] = analogRead(backSensor_1);
  back_buffer[1] = analogRead(backSensor_2);
  back_buffer[2] = analogRead(backSensor_3);
  back_buffer[3] = analogRead(backSensor_4);
  back_buffer[4] = analogRead(backSensor_5);
  
  for (size_t i = 0; i < sensorData->back_sensor_size; i++){
    num += back_buffer[i] * i;
    denom += back_buffer[i];
  } 
  sensorData->lineSensor_value_back = (num/denom);
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

void init_sensor_calibration_eeprom(uint32_t size){
  EEPROM.begin(size);
}

void store_sensor_calibration_eeprom(struct sensorData_t* sensorData){        // NOTE: Add calibration for back sensors(sensor struct should change and be moer general and not have both back and front in one srtuct)
  EEPROM.write(sensorData->start_adddress_calibration_front, 1);              // Write 1 to the first address to indicate that the calibration data is stored
  for (size_t i = 1; i <= sensorData->front_sensor_size; i++){
    EEPROM.write(sensorData->start_adddress_calibration_front + i, sensorData->front_calibration_data[i]);
  }
  EEPROM.commit();
}

void read_sensor_calibration_eeprom(struct sensorData_t* sensorData){
  if (EEPROM.read(sensorData->start_adddress_calibration_front) == 1){
    for (size_t i = 1; i <= sensorData->front_sensor_size; i++){
      sensorData->front_calibration_data[i] = EEPROM.read(sensorData->start_adddress_calibration_front + i);
    }
  }else{
    USBSerial.printf("No calibration data found\n");
  }
}