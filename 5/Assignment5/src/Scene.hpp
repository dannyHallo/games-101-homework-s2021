#pragma once

#include "Light.hpp"
#include "Object.hpp"
#include "Vector.hpp"
#include <memory>
#include <vector>

class Scene {
private:
  std::vector<std::unique_ptr<Object>> objects;
  std::vector<std::unique_ptr<Light>> lights;

public:
  // setting up options
  int width                = 400;
  int height               = 300;
  double fov               = 60;
  Vector3f backgroundColor = Vector3f(0.23f, 0.67f, 0.84f);
  int maxDepth             = 5;
  float epsilon            = 1e-5f;

  Scene() {}
  Scene(int w, int h) : width(w), height(h) {}

  // These class functions will automatically be inline functions when defined here
  void Add(std::unique_ptr<Object> object);
  void Add(std::unique_ptr<Light> light);

  [[nodiscard]] const std::vector<std::unique_ptr<Object>> &get_objects() const { return objects; }
  [[nodiscard]] const std::vector<std::unique_ptr<Light>> &get_lights() const { return lights; }
};

// inline prefix useful when DEFINING the function, not when DECLARING it.
inline void Scene::Add(std::unique_ptr<Object> object) { objects.push_back(std::move(object)); }
inline void Scene::Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }
