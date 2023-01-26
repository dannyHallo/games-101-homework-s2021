#include "Light.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"

using namespace std;

const string projectDir{"E:/Computer Graphics/GAMES101_Homework_S2021/5/Assignment5/"};

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main() {
  // Scene scene{1280, 720};
  Scene scene{};

  auto sph1          = make_unique<Sphere>(Vector3f(-1, 0, -12), 2);
  sph1->materialType = DIFFUSE_AND_GLOSSY;
  sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);

  auto sph2          = make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);
  sph2->ior          = 1.5;
  sph2->materialType = REFLECTION_AND_REFRACTION;

  scene.Add(move(sph1));
  scene.Add(move(sph2));

  Vector3f verts[4]     = {{-5, -3, -6}, {5, -3, -6}, {5, -3, -16}, {-5, -3, -16}};
  uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};
  Vector2f st[4]        = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  auto mesh             = make_unique<MeshTriangle>(verts, vertIndex, 2, st);
  mesh->materialType    = DIFFUSE_AND_GLOSSY;

  scene.Add(move(mesh));
  scene.Add(make_unique<Light>(Vector3f(-20, 70, 20), 0.5));
  scene.Add(make_unique<Light>(Vector3f(30, 50, -12), 0.5));

  Renderer r;
  r.Render(scene, projectDir);

  string command{"start /d \"" + projectDir + "\" preview.ppm"};
  system(command.c_str());

  return 0;
}