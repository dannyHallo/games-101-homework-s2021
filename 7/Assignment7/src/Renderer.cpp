#pragma once

#include "Renderer.hpp"
#include "Scene.hpp"
#include "global.hpp"
#include <fstream>

const float degToRad = 0.01745329252f;
inline float deg2rad(const float &deg) { return deg * degToRad; }

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene, const std::string &projDir) {
  std::vector<Vector3f> framebuffer(scene.width * scene.height);

  float scale            = tan(deg2rad(scene.fov * 0.5f));
  float imageAspectRatio = scene.width / static_cast<float>(scene.height);
  Vector3f eye_pos{};
  // Vector3f eye_pos(278, 273, -800);
  int m = 0;

  int spp = 40; // sample per pixel

  // THIS TIME, CAM HEADS TOWARDS +Z AXIS
  for (int j = scene.height - 1; j >= 0; --j) {
    for (int i = scene.width - 1; i >= 0; --i) {
      // uint32_t i = scene.width / 2, j = scene.height / 2;

      float x     = (static_cast<float>(i) / scene.width - 0.5f) * 2.f * scale * imageAspectRatio;
      float y     = (static_cast<float>(j) / scene.height - 0.5f) * 2.f * scale;
      float nextX = (static_cast<float>(i + 1) / scene.width - 0.5f) * 2.f * scale * imageAspectRatio;
      float nextY = (static_cast<float>(j + 1) / scene.height - 0.5f) * 2.f * scale;
      y           = ((y + 0) == 0) ? 0 : y; // y might be negative 0, this conversion is recommended

      Vector3f dir = normalize(Vector3f(getRandomFloat(x, nextX), getRandomFloat(y,nextY), 1));
      Ray ray{eye_pos, dir};

      for (int k = 0; k < spp; k++)
        framebuffer[m] += scene.castRay(ray, true) / static_cast<float>(spp);

      m++;
    }
    UpdateProgress((scene.height - j) / static_cast<float>(scene.height));
  }
  UpdateProgress(1.f);

  // save result to ppm file
  FILE *fp;
  fopen_s(&fp, (projDir + "preview.ppm").c_str(), "wb");
  (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
  for (int i = 0; i < scene.height * scene.width; ++i) {
    static unsigned char color[3];
    color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
    color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
    color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
    fwrite(color, 1, 3, fp);
  }
  fclose(fp);
}
