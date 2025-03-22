/*


*/
//#include <Wire.h>
#include <EEPROM.h>
#include "sensors.h"
#include "defines.h"
#include "driver/i2c.h"

uint8_t buf[20] = {0x00};
uint16_t sens_vals[10] = {0};

void init_i2c_frontSensor(){
  int i2c_master_port = 0;
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_SDA;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = I2C_SCL;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = I2C_READ_FREQ;

  delay(1000);
  i2c_param_config(0, &conf);
  i2c_driver_install(0, conf.mode, 0, 0, 0);
  delay(1000);
}

void lineSensor_value_front(struct sensorData_t* sensorData, uint8_t device_addr, uint8_t reg, size_t len, int32_t offset){
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((device_addr + 1) << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((device_addr + 1) << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, buf + 10, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  //USBSerial.printf("Values:");
  uint32_t num = 0;
  uint32_t denom = 0;
  for (size_t i = 0; i < 10; i++){
    uint16_t val = (uint16_t)(buf[2 * i]) | (((uint16_t)(buf[2 * i + 1]) << 8) & 0xFF00);
    num += val * i * 1000;
    denom += val; // Save as tot for finish line.
    //USBSerial.printf(" %d ", val);
  }
  //USBSerial.printf("\n");

  sensorData->total_value = (uint32_t)(denom);
  sensorData->lineSensor_value_front = (uint32_t)(num / denom);
  //USBSerial.printf("Line sensor value front: %d. Denom: %d\n", sensorData->lineSensor_value_front, denom);
  #if DEBUG_MODE == 1
    USBSerial.printf("Line sensor value front: %d\n", sensorData->lineSensor_value_front);
  #endif
}

/*
void lineSensor_value_front(struct sensorData_t* sensorData){   
  uint32_t num = 0;
  uint32_t denom = 0;
  Wire.beginTransmission(RIGHT_ADR);
  //Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(RIGHT_ADR, 10, 1);

  for (size_t i = 0; i < sensorData->front_sensor_size; i++){
    buf[i] = Wire.read();
  }

  Wire.beginTransmission(LEFT_ADR);
  //Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(LEFT_ADR, 10, 1);

  for (size_t i = 0; i < sensorData->front_sensor_size; i++){
    buf[i + 10] = Wire.read();
  }

  USBSerial.printf("Values: ");
  for (size_t i = 0; i < sensorData->front_sensor_size; i++){
    uint16_t val = (uint16_t)(buf[2 * i]) | (((uint16_t)(buf[2 * i + 1]) << 8) & 0xFF00);
    USBSerial.printf(" %d", val);
    num += val * i * 1000;
    denom += val;
  }
  USBSerial.printf("\n");
  sensorData->lineSensor_value_front = (uint32_t)(num / denom);
  USBSerial.printf("Line sensor value front: %d\n", sensorData->lineSensor_value_front); 
  #if DEBUG_MODE == 1
    USBSerial.printf("Line sensor value front: %d\n", sensorData->lineSensor_value_front);
  #endif
}*/

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
    num += back_buffer[i] * i * 1000;
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