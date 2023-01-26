#pragma once

#include <eigen3/Eigen/Eigen>
#include <math.h>

inline float customMapping(float val, float valRangeMin, float valRangeMax, float outRangeMin,
                           float outRangeMax) {
  return (val - valRangeMin) * (outRangeMax - outRangeMin) / (valRangeMax - valRangeMin) + outRangeMin;
}

inline float vectorDistance(Vector3f v) { return sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z()); }