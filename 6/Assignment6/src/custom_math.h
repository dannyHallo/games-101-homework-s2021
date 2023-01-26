#pragma once

inline float findMax(float a, float b, float c) {
  if (a > b && a > c)
    return a;
  if (b > c)
    return b;
  return c;
}

inline float findMin(float a, float b, float c) {
  if (a < b && a < c)
    return a;
  if (b < c)
    return b;
  return c;
}
