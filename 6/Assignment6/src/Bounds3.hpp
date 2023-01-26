//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include "custom_math.h"
#include <array>
#include <limits>

class Bounds3 {
public:
  Vector3f pMin, pMax; // two points to specify the bounding box
  Bounds3() {
    float minNum = std::numeric_limits<float>::lowest();
    float maxNum = std::numeric_limits<float>::max();
    pMax         = Vector3f(minNum, minNum, minNum);
    pMin         = Vector3f(maxNum, maxNum, maxNum);
  }
  Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
  Bounds3(const Vector3f p1, const Vector3f p2) {
    pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
    pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
  }

  inline void print() const { std::cout << "min: " << pMin.toString() << "    max: " << pMax.toString() << std::endl; }

  inline Vector3f Diagonal() const { return pMax - pMin; }

  inline int maxExtent() const {
    Vector3f d = Diagonal();

    // x is the largest
    if (d.x > d.y && d.x > d.z)
      return 0;
    // y is the largest
    else if (d.y > d.z)
      return 1;
    // z is the largest
    else
      return 2;
  }

  inline double SurfaceArea() const {
    Vector3f d = Diagonal();
    return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
  }

  /// @return the center point of the given bound
  inline Vector3f Centroid() const { return 0.5 * pMin + 0.5 * pMax; }

  /// @param b another bound to make union with
  /// @return the union bound of two bounds
  inline Bounds3 Intersect(const Bounds3 &b) const {
    return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y), fmax(pMin.z, b.pMin.z)),
                   Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y), fmin(pMax.z, b.pMax.z)));
  }

  /// @param p the point to calculate the offset
  /// @return the offset of a point, relative to the bound, points in the bound have offest in each axis ranged from 0-1
  inline Vector3f Offset(const Vector3f &p) const {
    Vector3f o = p - pMin;
    if (pMax.x > pMin.x)
      o.x /= pMax.x - pMin.x;
    if (pMax.y > pMin.y)
      o.y /= pMax.y - pMin.y;
    if (pMax.z > pMin.z)
      o.z /= pMax.z - pMin.z;
    return o;
  }

  inline bool Overlaps(const Bounds3 &b1, const Bounds3 &b2) const {
    bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
    bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
    bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
    return (x && y && z);
  }

  inline bool Inside(const Vector3f &p, const Bounds3 &b) const {
    return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y && p.z >= b.pMin.z &&
            p.z <= b.pMax.z);
  }

  inline const Vector3f &operator[](int i) const { return (i == 0) ? pMin : pMax; }

  /// @brief test if ray bound intersects
  bool IntersectP(const Ray &ray) const {
    const Vector3f &o      = ray.origin;
    const Vector3f &dir    = ray.direction;
    const Vector3f &dirInv = ray.direction_inv;

    std::array<bool, 3> dirNotNeg{dir.x >= 0, dir.y >= 0, dir.z >= 0};

    float xNear = dirNotNeg[0] ? pMin.x : pMax.x;
    float xFar  = dirNotNeg[0] ? pMax.x : pMin.x;
    float yNear = dirNotNeg[1] ? pMin.y : pMax.y;
    float yFar  = dirNotNeg[1] ? pMax.y : pMin.y;
    float zNear = dirNotNeg[2] ? pMin.z : pMax.z;
    float zFar  = dirNotNeg[2] ? pMax.z : pMin.z;

    float tMinX = (xNear - o.x) * dirInv.x;
    float tMaxX = (xFar - o.x) * dirInv.x;
    float tMinY = (yNear - o.y) * dirInv.y;
    float tMaxY = (yFar - o.y) * dirInv.y;
    float tMinZ = (zNear - o.z) * dirInv.z;
    float tMaxZ = (zFar - o.z) * dirInv.z;

    float tEnter = findMax(tMinX, tMinY, tMinZ);
    float tExit  = findMin(tMaxX, tMaxY, tMaxZ);

    return tEnter <= tExit && tExit >= 0;
  }
};

inline Bounds3 Union(const Bounds3 &b1, const Bounds3 &b2) {
  Bounds3 ret;
  ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
  ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
  return ret;
}

inline Bounds3 Union(const Bounds3 &b, const Vector3f &p) {
  Bounds3 ret;
  ret.pMin = Vector3f::Min(b.pMin, p);
  ret.pMax = Vector3f::Max(b.pMax, p);
  return ret;
}

#endif // RAYTRACING_BOUNDS3_H
