/**
* Description: Header file for sensors.cpp
* Contains functions to read sensors and copy information stored in structs. All sensors are stored in structs.
* 
* Note:
* The struct encoderData_t stores the nr of ticks of the encoders which are updated in the isr functions encoder1_isr() and encoder2_isr().
* To share the instance of the struct encoderData_t with the isr functions, a ptr is constructed in the main.cpp file and used by the isr functions as an extern ptr.
* Usage:
* Initialize the structs in the main.cpp file.
* Call the functions to read the sensors.
* Calibrate the sensors if needed, store in EEPROM. 
* Read the calibration data from EEPROM.
**/

#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>

/**
 * @brief Struct to store encoder data.
 * @see encoder1_isr()
 * @see encoder2_isr()
 * @param ENC_1_A: Pin of encoder 1 channel A (int32_t).
 * @param ENC_1_B: Pin of encoder 1 channel B (int32_t).
 * @param ENC_2_A: Pin of encoder 2 channel A (int32_t).
 * @param ENC_2_B: Pin of encoder 2 channel B (int32_t).
 * @param ENC_1_tick: Tick count of encoder 1 (volatile int32_t).
 * @param ENC_2_tick: Tick count of encoder 2 (volatile int32_t).
 */
struct encoderData_t{
    int32_t ENC_1_A;
    int32_t ENC_1_B;
    int32_t ENC_2_A;
    int32_t ENC_2_B;
    volatile int32_t ENC_1_tick;
    volatile int32_t ENC_2_tick;
};

/**
 * @brief Struct to store sensor data.
 * @param prev_leftEncoderTick: Previous tick count of left encoder (int32_t).
 * @param prev_rightEncoderTick: Previous tick count of right encoder (int32_t).
 * @param leftEncoderTick: Tick count of left encoder (int32_t).
 * @param rightEncoderTick: Tick count of right encoder (int32_t).
 * @param lineSensor_value_front: Value of front line sensor (int32_t).
 * @param lineSensor_value_back: Value of back line sensor (int32_t).
 * @param start_adddress_calibration: Start address of the calibration data in the eeprom (uint16_t).
 * @param sensor_size: Size of the sensor data in the eeprom (uint16_t).
 * @param calibration_data: Array to store the calibration data (int32_t[10]).
 * @param dt: Time step (float).
 */
struct sensorData_t{ // Change thi struct to be more general and have 1 defined struct and declare structs for each sensor intead
    int32_t prev_leftEncoderTick;
    int32_t prev_rightEncoderTick;
    int32_t leftEncoderTick;
    int32_t rightEncoderTick;
    int32_t lineSensor_value_front;
    int32_t lineSensor_value_back;
    uint16_t start_adddress_calibration_front;
    uint16_t start_adddress_calibration_back;
    uint16_t front_sensor_size;
    uint16_t back_sensor_size;
    int32_t front_calibration_data[11];
    int32_t back_calibration_data[6];
    float dt;
};

/**
 * @brief Read the value of the front line sensor.
 * @param sensorData: Pointer to the sensor data struct.
 * @see sensorData_t
 */
void lineSensor_value_front(struct sensorData_t* sensorData);

/**
 * @brief Read the value of the back line sensor.
 * @param sensorData: Pointer to the sensor data struct.
 * @see sensorData_t
 */
void lineSensor_value_back(struct sensorData_t* sensorData);

/**
 * @brief Update the encoder data in the sensorData struct.
 * @param sensorData: Pointer to the sensor data struct.
 * @param encoderData: Pointer to the encoder data struct.
 * @see sensorData_t
 * @see encoderData_t
 */
void update_encoder(struct sensorData_t* sensorData, struct encoderData_t* encoderData);

/**
 * @brief Calibrate the front line sensor.
 * @param sensorData: Pointer to the sensor data struct.
 * @see sensorData_t
 */
void calibrate_front_line_sensor(struct sensorData_t* sensorData);

/**
 * @brief Calibrate the back line sensor.
 * @param sensorData: Pointer to the sensor data struct.
 * @see sensorData_t
 */
void calibrate_back_line_sensor(struct sensorData_t* sensorData);


/**
 * @brief Initialize the sensor calibration in the eeprom.
 * @param size: Size of the sensor data in the eeprom (uint32_t).
 */
void init_sensor_calibration_eeprom(uint32_t size);

/**
 * @brief Store the sensor calibration in the eeprom.
 * @param sensorData: Pointer to the sensor data struct.
 * @see sensorData_t
 */
void store_sensor_calibration_eeprom(struct sensorData_t* sensorData);

/**
 * @brief Read the sensor calibration from the eeprom.
 * @param sensorData: Pointer to the sensor data struct.
 * @see sensorData_t
 */
void read_sensor_calibration_eeprom(struct sensorData_t* sensorData);
#endif

