#pragma once

#include "Scene.hpp"

struct hit_payload {
  float tNear;
  uint32_t index;
  Vector2f uv;
  Object *hit_obj;
};

class Renderer {
public:
  void Renderer::Render(const Scene &scene, const std::string &projDir);
};
