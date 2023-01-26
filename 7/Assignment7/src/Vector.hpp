#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

class Vector3f {
public:
  float x, y, z;
  Vector3f() : x(0), y(0), z(0) {}
  Vector3f(float xx) : x(xx), y(xx), z(xx) {}
  Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

  Vector3f operator*(const float &r) const { return Vector3f(x * r, y * r, z * r); }
  Vector3f operator/(const float &r) const { return Vector3f(x / r, y / r, z / r); }

  Vector3f operator*(const Vector3f &v) const { return Vector3f(x * v.x, y * v.y, z * v.z); }
  Vector3f operator-(const Vector3f &v) const { return Vector3f(x - v.x, y - v.y, z - v.z); }
  Vector3f operator+(const Vector3f &v) const { return Vector3f(x + v.x, y + v.y, z + v.z); }
  Vector3f operator-() const { return Vector3f(-x, -y, -z); }
  Vector3f &operator+=(const Vector3f &v) {
    x += v.x, y += v.y, z += v.z;
    return *this;
  }
  Vector3f &operator/=(const float &r) {
    x /= r, y /= r, z /= r;
    return *this;
  }
  friend Vector3f operator*(const float &r, const Vector3f &v) { return Vector3f(v.x * r, v.y * r, v.z * r); }
  friend std::ostream &operator<<(std::ostream &os, const Vector3f &v) {
    return os << v.x << ", " << v.y << ", " << v.z;
  }
  double operator[](int index) const { return (&x)[index]; };
  double &operator[](int index);

  float normSqr() const { return x * x + y * y + z * z; }
  float norm() const { return std::sqrt(normSqr()); }

  Vector3f normalized() const {
    float n = std::sqrt(x * x + y * y + z * z);
    assert(n && "Zero vector cannot be normalized!");
    return Vector3f(x / n, y / n, z / n);
  }

  std::string toString() const {
    std::ostringstream out;
    out.precision(6);
    out << std::fixed << x << ", " << y << ", " << z;
    return out.str();
  }

  static Vector3f Min(const Vector3f &p1, const Vector3f &p2) {
    return Vector3f(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
  }

  static Vector3f Max(const Vector3f &p1, const Vector3f &p2) {
    return Vector3f(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
  }
};

class Vector2f {
public:
  Vector2f() : x(0), y(0) {}
  Vector2f(float xx) : x(xx), y(xx) {}
  Vector2f(float xx, float yy) : x(xx), y(yy) {}
  Vector2f operator*(const float &r) const { return Vector2f(x * r, y * r); }
  Vector2f operator+(const Vector2f &v) const { return Vector2f(x + v.x, y + v.y); }
  float x, y;

  inline std::string toString() {
    std::ostringstream out;
    out.precision(2);
    out << std::fixed << x << ", " << y;
    return out.str();
  }
};

inline Vector3f lerp(const Vector3f &a, const Vector3f &b, const float &t) { return a * (1 - t) + b * t; }

inline Vector3f normalize(const Vector3f &v) { return v.normalized(); }

inline float dotProduct(const Vector3f &a, const Vector3f &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vector3f crossProduct(const Vector3f &a, const Vector3f &b) {
  return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}