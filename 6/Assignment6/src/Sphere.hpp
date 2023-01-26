#pragma once

#include "Bounds3.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include "Vector.hpp"

class Sphere : public Object {
public:
  Vector3f center;
  float radius, radius2;

public:
  Sphere(const Vector3f &c, const float &r) : center(c), radius(r), radius2(r * r) { m = new Material(); }

  inline bool intersect(const Ray &ray, float &tnear, Vector3f &normal, Vector2f &uv, Object **obj,
                        Material **mat) override {
    // analytic solution
    Vector3f L = ray.origin - center;
    float a    = dotProduct(ray.direction, ray.direction);
    float b    = 2 * dotProduct(ray.direction, L);
    float c    = dotProduct(L, L) - radius2;
    float t0, t1;
    if (!solveQuadratic(a, b, c, t0, t1))
      return false;
    if (t0 < 0)
      t0 = t1;
    if (t0 < 0)
      return false;

    tnear  = t0;
    normal = normalize((ray.origin + t0 * ray.direction) - center);
    uv     = Vector2f{};
    *obj   = this;
    *mat   = m;

    return true;
  }

  inline Vector3f evalDiffuseColor(const Vector2f &st) const override { return m->getColor(); }

  inline Bounds3 getBounds() const override {
    return Bounds3(Vector3f(center.x - radius, center.y - radius, center.z - radius),
                   Vector3f(center.x + radius, center.y + radius, center.z + radius));
  }
};
