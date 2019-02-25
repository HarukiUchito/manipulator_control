#include "Math_utils.hpp"

#include <cmath>

double deg2rad (double degrees) {
    return degrees * 4.0 * atan (1.0) / 180.0;
}