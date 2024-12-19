#ifndef DTW_DISTANCE_H
#define DTW_DISTANCE_H 

#include "macro.hpp"

float p_norm(float a[3], float b[3], uint32_t a_b_dimLEN, float p);
float dtw_distance_only(
    float a[MAX_ARRY_2D_SIZE][3], uint32_t a_dataPointLEN, uint32_t a_dimLEN,
    float b[MAX_ARRY_2D_SIZE][3], uint32_t b_dataPointLEN, uint32_t b_dimLEN, float p
);

#endif