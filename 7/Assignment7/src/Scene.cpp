#include "Scene.hpp"
#include "global.hpp"

void Scene::buildBVH() { bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE); }

// void Scene::sampleLight(Intersection &pos, float &pdf) const {
//   float emit_area_sum = 0;
//   for (uint32_t k = 0; k < objects.size(); ++k) {
//     if (objects[k]->hasEmit()) {
//       emit_area_sum += objects[k]->getArea();
//     }
//   }
//   float p       = getRandomFloat() * emit_area_sum;
//   emit_area_sum = 0;
//   for (uint32_t k = 0; k < objects.size(); ++k) {
//     if (objects[k]->hasEmit()) {
//       emit_area_sum += objects[k]->getArea();
//       if (p <= emit_area_sum) {
//         objects[k]->Sample(pos, pdf);
//         break;
//       }
//     }
//   }
// }

// TODO: Implement Path Tracing Algorithm
Vector3f Scene::castRay(const Ray &ray, const bool &primaryRay) const {
  Vector3f hitColor{};

  Intersection intersection = bvh->Intersect(ray);

  if (!intersection.happened) {
    if (primaryRay)
      return backgroundColor;
    return hitColor;
  }

  // return intersection.m->Kd;

  Vector3f hitPoint = intersection.coords;
  Vector3f n        = intersection.normal;
  Material &m       = *intersection.m;

  if (m.hasEmission()) {

    return m.m_emission;
  }

  // make sure the origin of the secondRay is always near the viewer
  Vector3f secondaryRayOrig{(dotProduct(ray.direction, n) < 0) ? hitPoint + n * EPSILON : hitPoint - n * EPSILON};

  // direct light calculation
  Vector3f directColor{};
  for (const auto &l : lights) {
    Vector3f samplePointOnLight, normalOnLight;
    float pdfLight;

    l->Sample(samplePointOnLight, normalOnLight, pdfLight);

    Vector3f o2l  = samplePointOnLight - secondaryRayOrig;
    float distSqr = o2l.normSqr();
    float dist    = std::sqrt(distSqr);

    // if the light path is not occluded..
    Vector3f secondaryRayDir = o2l / dist;
    Ray secondaryRay{secondaryRayOrig, secondaryRayDir};
    Intersection lightIntersection = bvh->Intersect(secondaryRay);
    if (lightIntersection.happened) {
      if (lightIntersection.m->hasEmission()) {
        directColor = m.Kd * l->m->m_emission * dotProduct(secondaryRayDir, n) *
                      dotProduct(-secondaryRayDir, normalOnLight) / distSqr / pdfLight;
      }
    } 
  }
  hitColor += directColor;

  // indirct light calculation
  Vector3f indirectColor{};
  if (!primaryRay && getRandomFloat() > russianRoulette)
    return hitColor;
  Vector3f secondaryRayDir = randomPointOnSemisphere(n);
  Ray shadowRay{secondaryRayOrig, secondaryRayDir};

  indirectColor = m.Kd * castRay(shadowRay, false) * dotProduct(secondaryRayDir, n) * 2 * M_PI / russianRoulette;
  hitColor += indirectColor;

  return hitColor;
}