#ifndef MACROS_H
#define MACROS_H

#include <Arduino.h>
#include "defines.h"

/*Clamping*/
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(x, lower, upper) (MIN((upper), MAX((x), (lower))))


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

