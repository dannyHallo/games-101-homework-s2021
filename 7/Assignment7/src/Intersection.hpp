#pragma once

#include "Material.hpp"
#include "Vector.hpp"

class Object;

struct Intersection {
  bool happened;
  Vector3f coords;
  Vector3f tcoords; // ?
  Vector3f normal;
  Vector3f emit;
  float distance;
  Object *obj;
  Material *m;
  Vector2f uv;

  Intersection()
      : happened{false}, coords{}, tcoords{}, normal{}, emit{}, distance{std::numeric_limits<float>::max()}, obj{0},
        m{0}, uv{} {}
};
