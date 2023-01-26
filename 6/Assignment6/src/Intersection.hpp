#pragma once

#include "Material.hpp"
#include "Vector.hpp"

class Object;

struct Intersection {
  bool happened;
  Vector3f coords;
  Vector3f normal;
  double distance;
  Object *obj;
  Material *m;
  Vector2f uv;

  Intersection()
      : happened{false}, coords{}, normal{}, distance{std::numeric_limits<double>::max()}, obj{0}, m{0}, uv{} {}

  inline void print() {
    if (!happened) {
      std::cout << "NOT happened";
      return;
    }

    std::cout << "coords: " << coords.toString() << " normal: " << normal.toString() << " distance: " << distance
              << " obj: " << obj << " mat: " << m << " uv: " << uv.toString() << std::endl;
  }
};
