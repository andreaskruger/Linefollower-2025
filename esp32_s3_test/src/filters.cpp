
#include "filters.h"

void applyLPFilter(struct lowPassFilter_t* filter, float input){
    filter->output = filter->alpha * input + (1 - filter->alpha) * filter->output;
}