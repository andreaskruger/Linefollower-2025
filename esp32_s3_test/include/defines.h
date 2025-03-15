/**
* Description: Contains defines for the project.
* 
* Note:
* 
* Usage:
* Import the file in all .h files that need to use the defines(Often all .h files).
**/

#ifndef DEFINES_H
#define DEFINES_H

/*Global Defines*/
//Debugging
#define DEBUG_MODE                  (1)  // 0: OFF, 1: ON

// Clamping
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(x, lower, upper) (MIN((upper), MAX((x), (lower))))

// Robot constants
#define ROBOT_LENGTH                  (0.3) // In meters
#define ROBOT_WIDTH                   (0.2) // In meters
#define FRONT_SENSOR_NR               (10)  // Number of IR sensors in front of the robot
#define BACK_SENSOR_NR                (5)   // Number of IR sensors in back of the robot

// Motor
#define BASE_SPEED                  (100)
#define MOTOR_POLES (24)                                                // Number of poles is 12, isr on "change" -> "24" poles 
#define WHEEL_DIAMETER (0.3)                                            // in dm
#define WHEEL_CIRCUMFERENCE (2.0*PI*WHEEL_DIAMETER)                     // in dm
#define WHEEL_CIRCUMFERENCE_SEGMENT (WHEEL_CIRCUMFERENCE/MOTOR_POLES)   // in dm
#define MOTR_1                      (17)
#define MOTR_2                      (18)
#define MOTL_1                      (40)
#define MOTL_2                      (41)
#define RUN                         (1)
#define STOP                        (0)
#define LEFT_MOTOR                  (1)
#define RIGHT_MOTOR                 (2)
#define FORWARD                     (1)
#define BACKWARD                    (0)

// Encoder
#define ENCODER_RESOLUTION          (11)
#define ENCODER_FREQUENCY           (16000)
#define ENCODER_1_A                 (38)
#define ENCODER_1_B                 (39)
#define ENCODER_2_A                 (10)
#define ENCODER_2_B                 (8)

// Timer interupt, 1000Hz
#define TIMER_PRESCALER             (80)
#if DEBUG_MODE == 1  
    #define TIMER_FREQUENCY         (1000000) // Resulting in 1 second timer
#else
    #define TIMER_FREQUENCY         (1000) // Resulting in 1ms timer
#endif
#define MC_CLOCK_SPEED              (80000000)
#define DT                          (1/((MC_CLOCK_SPEED/TIMER_PRESCALER)/TIMER_FREQUENCY))

// Buttons
#define BT1                         (14)
#define BT2                         (21)
#define BT3                         (42)
#define DIP1                        (1)
#define DIP2                        (2)

// Sensors
#define backSensor_1                (4)
#define backSensor_2                (5)
#define backSensor_3                (6)
#define backSensor_4                (7)
#define backSensor_5                (9)
#define I2C_SDA                     (48)
#define I2C_SCL                     (47)
#define LEFT_ADR                    (0x09)
#define RIGHT_ADR                   (0x0A)
#define FRONT_SENSOR_EPROM_ADR      (0x00)
#define BACK_SENSOR_EPROM_ADR       (0x0C)
#define ALL_BLACK_VALUE             (1000) // Figure out a value and a range to signal the end line is found

// PID
#define STD_SPEED_KP                (1.0)
#define STD_SPEED_KI                (0.0)
#define STD_SPEED_KD                (1.0)
#define STD_PWM_KP                  (1.0)
#define STD_PWM_KI                  (0.0)
#define STD_PWM_KD                  (1.0)
#define AGR_SPEED_KP                (0.0)
#define AGR_SPEED_KI                (0.0)
#define AGR_SPEED_KD                (0.0)
#define AGR_PWM_KP                  (0.0)
#define AGR_PWM_KI                  (0.0)
#define AGR_PWM_KD                  (0.0)

// Filters
#define LP_ALPHA                    (0.01)

#endif