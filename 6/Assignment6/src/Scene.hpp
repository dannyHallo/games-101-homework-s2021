#pragma once

#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Light.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include <vector>

class Scene {
public:
  uint32_t width           = 400;
  uint32_t height          = 300;
  float fov                = 60;
  int maxDepth             = 5;
  Vector3f ambientLight    = Vector3f{1, 1, 1};
  Vector3f backgroundColor = Vector3f(0.235f, 0.674f, 0.843f);

private:
  std::vector<Object *> objects;
  std::vector<std::unique_ptr<Light>> lights;
  BVHAccel *bvh;

public:
  // a null constructor can be DEFINED by a pair of empty curly braces, however, a single semicolon only declares it!
  Scene() {}
  Scene(int w, int h) : width(w), height(h) {}

  // inline prefix useful when DEFINING the function, not when DECLARING it.
  inline void Add(Object *object) { objects.push_back(object); }
  inline void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

  // the return val should never be omitted
  [[nodiscard]] const std::vector<Object *> &get_objects() const { return objects; }
  [[nodiscard]] const std::vector<std::unique_ptr<Light>> &get_lights() const { return lights; }

  void buildBVH();
  // Intersection intersect(const Ray &ray) const;
  Vector3f castRay(const Ray &ray, int depth) const;
  // bool trace(const Ray &ray, const std::vector<Object *> &objects, float &tNear, uint32_t &index, Object
  // **hitObject);
  std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint, const Vector3f &N,
                                                 const Vector3f &shadowPointOrig, const std::vector<Object *> &objects,
                                                 uint32_t &index, const Vector3f &dir, float specularExponent);

  // Compute reflection direction
  Vector3f reflect(const Vector3f &I, const Vector3f &N) const { return I - 2 * dotProduct(I, N) * N; }

  // Compute refraction direction using Snell's law
  // We need to handle with care the two possible situations:
  //    - When the ray is inside the object
  //    - When the ray is outside.
  // If the ray is outside, you need to make cosi positive cosi = -N.I
  // If the ray is inside, you need to invert the refractive indices and negate the normal N
  // TODO: avoid inline
  inline Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const {
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    Vector3f n = N;
    if (cosi < 0) {
      cosi = -cosi;
    } else {
      std::swap(etai, etat);
      n = -N;
    }
    float eta = etai / etat;
    float k   = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
  }

  // Compute Fresnel equation
  // \param I is the incident view direction
  // \param N is the normal at the intersection point
  // \param ior is the material refractive index
  // \param[out] kr is the amount of light reflected
  inline void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const {
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0) {
      std::swap(etai, etat);
    }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) {
      kr = 1;
    } else {
      float cost = sqrtf(std::max(0.f, 1 - sint * sint));
      cosi       = fabsf(cosi);
      float Rs   = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
      float Rp   = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
      kr         = (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
  }
};
