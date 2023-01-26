#pragma once

#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include "global.hpp"

class Object {
public:
  Material *m;

public:
  Object() {}
  virtual ~Object() {}
  virtual bool intersect(const Ray &ray, float &tnear, Vector3f &normal, Vector2f &uv, Object **obj,
                         Material **mat) = 0;
  //   virtual Intersection getIntersection(Ray _ray)                    = 0;
  virtual Vector3f evalDiffuseColor(const Vector2f &) const              = 0;
  virtual Bounds3 getBounds() const                                      = 0;
  virtual float getArea() const                                          = 0;
  virtual void Sample(Vector3f &pos, Vector3f &normal, float &pdf) const = 0;

  virtual bool hasEmit() const { return m->hasEmission(); }
};
