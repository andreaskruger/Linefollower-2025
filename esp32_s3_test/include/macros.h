#ifndef MACROS_H
#define MACROS_H

#include <Arduino.h>
#include "defines.h"

/*Clamping*/
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(x, lower, upper) (MIN((upper), MAX((x), (lower))))


/*Bitwise ops*/
#define BIT(x) (1 << (x))                                               // Create a bit mask (e.g., BIT(3) = 0b00001000)
#define SET_BIT(REG, BIT) ((REG) |= (1 << (BIT)))                       // Set a bit in a register
#define CLR_BIT(REG, BIT) ((REG) &= ~(1 << (BIT)))                      // Clear a bit in a register
#define TOGGLE_BIT(REG, BIT) ((REG) ^= (1 << (BIT)))                    // Toggle a bit
#define CHECK_BIT(REG, BIT) (((REG) >> (BIT)) & 1)                      // Check if a bit is set (returns 0 or 1)


/*Byte ops*/
#define LOW_BYTE(x) ((uint8_t)((x) & 0xFF))                             // Get the low byte of a 16-bit value
#define HIGH_BYTE(x) ((uint8_t)(((x) >> 8) & 0xFF))                     // Get the high byte of a 16-bit value
#define COMBINE_BYTES(high, low) (((high) << 8) | (low))                // Combine two bytes into a 16-bit value
#define DELAY_US(x) for (volatile int i = 0; i < (x * 10); i++) _NOP()  // Approximate microsecond delay


/*Debugging*/
#if DEBUG_MODE == 1                                                     // If debug mode is on(define.h), print to USBSerial
    #define DEBUG_PRINT(x) printf("%s\n", x)
#else
    #define DEBUG_PRINT(x)
#endif


/*Size checking*/
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))                      // Get size of an array
#define FIELD_SIZE(type, field) (sizeof(((type *)0)->field))            // Get size of a struct field











#endif


