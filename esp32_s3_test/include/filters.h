/**
* Description: Contains the structs and functions to apply filters.
* 
* Note:
* 
* Usage:
* 
**/

#ifndef FILTERS_H
#define FILTERS_H
#include <Arduino.h>

/**
 * @brief Struct to store low pass filter parameters.
 * 
 * @param alpha: Filter constant (float).
 * @param output: Filtered output (float).
 */
struct lowPassFilter_t{
    float alpha;
    float output;
};

/**
 * @brief Struct to store low pass filter parameters.
 * 
 * @param alpha: Filter constant (float).
 * @param output: Filtered output (float).
 */
void applyLPFilter(struct lowPassFilter_t* filter, float input);

#endif