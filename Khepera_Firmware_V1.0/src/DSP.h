#pragma once

#include <Arduino.h>

// Slightly silly way of handling median and average filters
// To optimize for speed, the average filter must be 
#define filterSize 4 // Size of the filter. Must be 2^numbAvgBits for average filter or 5 for Median. Anymore can hurt the system
#define numbAvgBits 2 // Number of bits in the average filter equal to log2(filterSize)
struct filterBuffer {
    uint16_t arrayValue[filterSize];
};
// filter instantiations
filterBuffer perpendicularFilter; 
filterBuffer parallelFilter;

// Calculate median for 5. Hardcoded because a larger size could lead to speed complications
void find_med_five(struct filterBuffer med, uint16_t *med_val)
{
    
    uint16_t min_val;
    // Loop through the first half of the values and sort them
    for (uint16_t j = 0; j < 3; j++)
    {
        uint16_t min_valInd = j;
        // Get the current value
        min_val = (med.arrayValue)[j];
        for (uint16_t i = j + 1; i < 5; i++) // Compare to the rest
        {
            // If there is a smaller one then find its index and reset index count
            if (min_val > (med.arrayValue)[i])
            {
                min_valInd = i;
                min_val = (med.arrayValue)[i];
            }
        }
        // If the index changed. ie it is not in the right place, swap positions
        if (min_val != (med.arrayValue)[j])
        {
            // swap positions
            (med.arrayValue)[min_valInd] = (med.arrayValue)[j];
            (med.arrayValue)[j] = min_val;
        }
    }
    // The last acquired minimum check must be the median (third value)
    *med_val = min_val;
}
// Add the value to the median filter
void addFilter(uint16_t value, struct filterBuffer *medFilter) {
    static uint16_t arrayPos = 0;
    arrayPos = (arrayPos + 1) % 5;
    medFilter->arrayValue[arrayPos] = value;
}
// Run through the median filter
uint16_t getMedFilter(struct filterBuffer medFilter) {
    uint16_t medianValue;
    find_med_five(medFilter, &medianValue);
    return medianValue;
}
// Run through the average filter
uint16_t getAvgFilter(struct filterBuffer avgFilter) {
    uint32_t sum = 0;
    for(int i = 0; i < filterSize; i++) sum += avgFilter.arrayValue[i];
    return sum >> numbAvgBits;

}


