#include "Scene.hpp"

const float EPSILON = 0.00001f;

void Scene::buildBVH() { bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE); }

// Implementation of the Whitted-syle light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refracton direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refractin depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is duffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
Vector3f Scene::castRay(const Ray &ray, int depth) const {
  if (depth > maxDepth)
    return Vector3f(0.0, 0.0, 0.0);

  // std::cout << "--> primary ray: " << std::flush;
  // ray.print();

  Intersection intersection = bvh->Intersect(ray);

  Vector3f hitColor{};

  if (!intersection.happened) {
    Vector3f hitColor = backgroundColor;
    return hitColor;
  }

  Material *m       = intersection.m;
  Object *hitObject = intersection.obj;
  Vector3f hitPoint = intersection.coords;
  Vector2f uv       = intersection.uv;
  Vector3f N        = intersection.normal;
  Vector2f st       = intersection.uv;

  switch (m->getType()) {
  case REFLECTION_AND_REFRACTION: {
    Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
    Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));
    Vector3f reflectionRayOrig =
        (dotProduct(reflectionDirection, N) < 0) ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;
    Vector3f refractionRayOrig =
        (dotProduct(refractionDirection, N) < 0) ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;
    Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
    Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
    float kr;
    fresnel(ray.direction, N, m->ior, kr);
    hitColor = reflectionColor * kr + refractionColor * (1 - kr);
    break;
  }
  case REFLECTION: {
    float kr;
    fresnel(ray.direction, N, m->ior, kr);
    Vector3f reflectionDirection = reflect(ray.direction, N);
    Vector3f reflectionRayOrig =
        (dotProduct(reflectionDirection, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON;
    hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1) * kr;
    break;
  }
  default: {
    // We use the Phong illumination model in the default case. The phong model
    // is composed of a diffuse and a specular reflection component.

    Vector3f lightAmt{}, specularColor{};
    float seenByLight = 0;
    Vector3f shadowPointOrig{(dotProduct(ray.direction, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON};

    // Loop over all lights in the scene and sum their contribution up
    // We also apply the lambert cosine law

    for (const auto &light : lights) {
      auto l        = light.get();
      auto area_ptr = dynamic_cast<AreaLight *>(l);
      if (area_ptr) {
        // Do nothing for this assignment
      } else {
        Vector3f lightDir = l->position - hitPoint;
        // square of the distance between hitPoint and the light
        float lightDistance2    = dotProduct(lightDir, lightDir);
        lightDir                = normalize(lightDir);
        float LdotN             = std::max(0.f, dotProduct(lightDir, N));
        Object *shadowHitObject = nullptr;
        float tNearShadow       = kInfinity;

        Ray shadowRay{shadowPointOrig, lightDir};

        // std::cout << "--> shadow ray: " << std::flush;
        // shadowRay.print();
        // the light path hit something
        bool inShadow = bvh->Intersect(shadowRay).happened;

        seenByLight                  = inShadow ? 0 : 1;
        lightAmt                     = l->intensity * LdotN;
        Vector3f reflectionDirection = reflect(-lightDir, N);
        specularColor +=
            powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)), m->specularExponent) * l->intensity;
        hitColor += seenByLight * m->m_color * (lightAmt * hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks) +
                    ambientLight * m->Ka;
      }
    }
    break;
  }
  }
  return hitColor;
}