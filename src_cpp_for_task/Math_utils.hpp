#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <vector>
#include <cstdint>

double deg2rad (double degrees);

template <int dotPos>
double fix2double (std::vector<unsigned char> &v) {
    int32_t vi = 0;
    for (int i = 0; i < 4; ++i) {
        vi += ((int32_t)v[i]) << (8*i);
    }

    return (double)vi / (double)(1<<dotPos);
}

template <int dotPos>
std::vector<unsigned char> double2fix (double v) {
    std::vector<unsigned char> ret;
    int32_t vd = (int32_t)round(v * (1<<dotPos));
    for (int i = 0; i < 4; ++i) {
        ret.push_back((vd >> (8*i)) & 0xFF);
    }

    return ret;
}

#endif