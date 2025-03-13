/**
* Description:
* 
* Note:
* 
* Usage:
* 
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
 */
struct sensorData_t{
    int32_t prev_leftEncoderTick;
    int32_t prev_rightEncoderTick;
    int32_t leftEncoderTick;
    int32_t rightEncoderTick;
    int32_t lineSensor_value_front;
    int32_t lineSensor_value_back;
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

#endif

