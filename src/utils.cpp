#include <cmath>

#include "utils.hpp"
#include "macro.hpp"

void calculate_mean(
    const float arr[MAX_ARRY_2D_SIZE][3],
    float *mean_x,
    float *mean_y,
    float *mean_z
    ) {
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;

    for (int i = 0; i < MAX_ARRY_2D_SIZE; i++) {
        sum_x += arr[i][0];
        sum_y += arr[i][1];
        sum_z += arr[i][2];
    }

    *mean_x = sum_x/MAX_ARRY_2D_SIZE;
    *mean_y = sum_y/MAX_ARRY_2D_SIZE;
    *mean_z = sum_z/MAX_ARRY_2D_SIZE;
}


void calculate_std(
    const float arr[MAX_ARRY_2D_SIZE][3],
    float *std_x,
    float *std_y,
    float *std_z,
    float mean_x,
    float mean_y,
    float mean_z
) {
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;

    for (int i = 0; i < MAX_ARRY_2D_SIZE; i++) {
        sum_x += std::pow(mean_x - arr[i][0], 2);
        sum_y += std::pow(mean_y - arr[i][1], 2);
        sum_z += std::pow(mean_z - arr[i][2], 2);

    }

    *std_x = std::sqrt(sum_x/MAX_ARRY_2D_SIZE);
    *std_y = std::sqrt(sum_y/MAX_ARRY_2D_SIZE);
    *std_z = std::sqrt(sum_z/MAX_ARRY_2D_SIZE);
}


void standard_scaler(float arr[MAX_ARRY_2D_SIZE][3]) {
    float mean_x = 0;
    float mean_y = 0;
    float mean_z = 0;

    float std_x = 0;
    float std_y = 0;
    float std_z = 0;

    calculate_mean(arr, &mean_x, &mean_y, &mean_z);
    calculate_std(arr, &std_x, &std_y, &std_z, mean_x, mean_y, mean_z);

    for (int i=0; i < MAX_ARRY_2D_SIZE; i++) {
        arr[i][0] = (arr[i][0] - mean_x)/std_x;
        arr[i][1] = (arr[i][1] - mean_y)/std_y;
        arr[i][2] = (arr[i][2] - mean_z)/std_z;
    }

}