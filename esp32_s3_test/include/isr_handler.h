/**
* Description: Handles the attachment and setup of interupt routines for buttons, sensors and timers.
* 
* Note:
* 
* Usage:
* 
**/

#ifndef ISR_HANDLER_H
#define ISR_HANDLER_H
#include <Arduino.h>

/**
 * @brief Interupt service routine for button 1.
 * 
 */
void IRAM_ATTR button1_isr();

/**
 * @brief Interupt service routine for button 2.
 * 
 */
void IRAM_ATTR button2_isr();

/**
 * @brief Interupt service routine for button 3.
 * 
 */
void IRAM_ATTR button3_isr();

/**
 * @brief Interupt service routine for the dip switch.
 * 
 */
void IRAM_ATTR dip_isr();

/**
 * @brief Interupt service routine for encoder 1. Increments or decrements the encoder count of the first encoder,
 * @note A shared resource is used to access the encoder struct.
 * @see encoderData_t
 * 
 */
void IRAM_ATTR encoder1_isr();

/**
 * @brief Interupt service routine for encoder 2. Increments or decrements the encoder count of the second encoder,
 * @note A shared resource is used to access the encoder struct.
 * @see encoderData_t
 * 
 */
void IRAM_ATTR encoder2_isr();

/**
 * @brief Configure and setup the encoders, sets pinmodes and attaches the interupts.
 * 
 * @param encoders_struct: Pointer to the encoder struct.
 * @see encoder1_isr()
 * @see encoder2_isr()
 */
void setupEncoders(struct encoderData_t* encoders_struct);

/**
 * @brief Configure and setup the buttons, sets pinmodes and attaches the interupts.
 * Button1, Button2, Button3, DipSwitch are all attached to an isr.
 * @see button1_isr()
 * @see button2_isr()
 * @see button3_isr()
 * @see dip_isr()
 */
void buttons_setup();

#endif

