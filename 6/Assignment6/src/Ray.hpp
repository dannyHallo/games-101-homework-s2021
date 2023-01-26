#pragma once

#include "Vector.hpp"

struct Ray {
  // Destination = origin + t*direction
  Vector3f origin;
  Vector3f direction, direction_inv;
  float t; // transportation time,
  float t_min, t_max;

  Ray(const Vector3f &ori, const Vector3f &dir, const float _t = 0.0) : origin(ori), direction(dir), t(_t) {
    direction_inv = Vector3f(1.f / direction.x, 1.f / direction.y, 1.f / direction.z);
    t_min         = 0.0;
    t_max         = std::numeric_limits<float>::max();
  }

  inline void print() const {
    std::cout << "origin: " << origin.toString() << "    "
              << "dir: " << direction.toString() << std::endl;
  }

  Vector3f operator()(float t) const { return origin + direction * t; }

  friend std::ostream &operator<<(std::ostream &os, const Ray &r) {
    os << "[origin:=" << r.origin << ", direction=" << r.direction << ", time=" << r.t << "]\n";
    return os;
  }
};
