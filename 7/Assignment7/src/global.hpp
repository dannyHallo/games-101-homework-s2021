#pragma once

#include "Vector.hpp"
#include <cmath>
#include <iostream>
#include <random>

const float M_PI      = 3.141592653589793f;
const float EPSILON   = 1e-6f;
const float kInfinity = std::numeric_limits<float>::max();
const Vector3f red    = {1, 0, 0};
const Vector3f green  = {0, 1, 0};
const Vector3f blue   = {0, 0, 1};

inline float clamp(const float &lo, const float &hi, const float &v) { return std::max(lo, std::min(hi, v)); }

inline bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1) {
  float discr = b * b - 4 * a * c;
  if (discr < 0)
    return false;
  else if (discr == 0)
    x0 = x1 = -0.5f * b / a;
  else {
    float q = (b > 0) ? -0.5f * (b + sqrt(discr)) : -0.5f * (b - sqrt(discr));
    x0      = q / a;
    x1      = c / q;
  }
  if (x0 > x1)
    std::swap(x0, x1);
  return true;
}

inline int getRandomInt(const int minIncl, const int maxExcl) {
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<> dist(minIncl, maxExcl - 1);

  return dist(rng);
}

inline float getRandomFloat(const float minIncl = 0, const float maxExcl = 1) {
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_real_distribution<float> dist(minIncl, maxExcl);

  return dist(rng);
}

inline void UpdateProgress(float progress) {
  int barWidth = 70;

  std::cout << "[";
  int pos = static_cast<int>(barWidth * progress);
  for (int i = 0; i < barWidth; ++i) {
    if (i < pos)
      std::cout << "=";
    else if (i == pos)
      std::cout << ">";
    else
      std::cout << " ";
  }
  std::cout << "] " << int(progress * 100.0) << " %\r";
  std::cout.flush();
};

inline Vector3f randomPointInSphere() {
  float x, y, z, d;
  do {
    x = getRandomFloat() * 2 - 1;
    y = getRandomFloat() * 2 - 1;
    z = getRandomFloat() * 2 - 1;
    d = x * x + y * y + z * z;
  } while (d > 1);

  return Vector3f{x, y, z};
}

inline Vector3f randomPointOnSphere() {
  Vector3f p = randomPointInSphere();
  p /= p.norm();
  return p;
}

inline Vector3f randomPointOnSemisphere(const Vector3f &n) {
  Vector3f v{};
  do {
    v = randomPointOnSphere();
  } while (dotProduct(v, n) < 0);
  return v;
}

inline void pause() { system("pause"); }