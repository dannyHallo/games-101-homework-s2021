#include "Renderer.hpp"
#include "Ray.hpp"
#include "Scene.hpp"
#include <fstream>

constexpr float degToRad = 0.01745329252f;
inline float deg2rad(const float &deg) { return deg * degToRad; }

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene, const std::string &projDir) {
  std::vector<Vector3f> framebuffer(scene.width * scene.height);

  float scale            = tan(deg2rad(scene.fov * 0.5f));
  float imageAspectRatio = scene.width / (float)scene.height;
  Vector3f eye_pos{};
  int m = 0;
  for (uint32_t j = 0; j < scene.height; ++j) {
    for (uint32_t i = 0; i < scene.width; ++i) {
      // uint32_t i = scene.width / 2, j = scene.height / 2;

      float x = (static_cast<float>(i + 0.5f) / scene.width - 0.5f) * 2.f * scale * imageAspectRatio;
      float y = -(static_cast<float>(j + 0.5f) / scene.height - 0.5f) * 2.f * scale;
      y       = ((y + 0) == 0) ? 0 : y; // y might be negative 0, this conversion is recommended

      Vector3f dir = normalize(Vector3f(x, y, -1));
      Ray ray{eye_pos, dir};

      framebuffer[m++] = scene.castRay(ray, 0);
    }
    UpdateProgress(j / (float)scene.height);
  }

  UpdateProgress(1.f);

  // save framebuffer to file
  FILE *fp;
  fopen_s(&fp, (projDir + "preview.ppm").c_str(), "wb");
  (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
  for (uint32_t i = 0; i < scene.height * scene.width; ++i) {
    static unsigned char color[3];
    color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
    color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
    color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
    fwrite(color, 1, 3, fp);
  }
  fclose(fp);
}
