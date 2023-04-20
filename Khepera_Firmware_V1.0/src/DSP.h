#pragma once

#include <Arduino.h>

struct MedFilter {
    uint16_t arrayValue[5];
};
MedFilter perpendicularFilter; 
MedFilter parallelFilter;
// Calculate median for 5. Hardcoded because a larger size could lead to speed complications
void find_med_five(struct MedFilter med, uint16_t *med_val)
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
void addMedFilter(uint16_t value, struct MedFilter *medFilter) {
    static uint16_t arrayPos = 0;
    arrayPos = (arrayPos + 1) % 5;
    medFilter->arrayValue[arrayPos] = value;
}
// Run through the median filter
uint16_t getMedFilter(struct MedFilter medFilter) {
    uint16_t medianValue;
    find_med_five(medFilter, &medianValue);
    return medianValue;
}
