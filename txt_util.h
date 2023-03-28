//
// Created by Chu-Hsuan Lin on 2022/3/16.
//
#include <cstdio>
#include <vector>
#include <iostream>

#ifndef CALIBRATION_AND_AUGMENTED_REALITY_TXT_UTIL_H
#define CALIBRATION_AND_AUGMENTED_REALITY_TXT_UTIL_H

int readInstrinsic(char *filename, std::vector<float> &camera_matrix, std::vector<float> &dist_coef);

#endif //CALIBRATION_AND_AUGMENTED_REALITY_TXT_UTIL_H
