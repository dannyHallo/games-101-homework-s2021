#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

using namespace std;

const string projectDir{"E:/Computer Graphics/GAMES101_Homework_S2021/7/Assignment7/"};

int main(int argc, char **argv) {
  Scene scene{400, 400};

  Material *red   = new Material(DIFFUSE, {0.63f, 0.065f, 0.05f});
  Material *green = new Material(DIFFUSE, {0.14f, 0.45f, 0.09f});
  Material *white = new Material(DIFFUSE, {0.725f, 0.71f, 0.68f});
  Material *light = new Material(DIFFUSE, {0.65f}, {30});

  // Testing purpose:
  // Sphere *testBall = new Sphere{Vector3f(0, 0, -10), 2};
  // testBall->m = red;
  // MeshTriangle *bunny    = new MeshTriangle(projectDir + "models/bunny/bunny.obj", red, {1, -5, -10}, 50);
  // scene.Add(testBall);
  // scene.Add(bunny);
  // scene.Add(make_unique<Light>(Vector3f(10, 0, -2), 1));

  MeshTriangle *floor     = new MeshTriangle(projectDir + "models/cornellbox/floor.obj", white);
  MeshTriangle *shortbox  = new MeshTriangle(projectDir + "models/cornellbox/shortbox.obj", green);
  MeshTriangle *tallbox   = new MeshTriangle(projectDir + "models/cornellbox/tallbox.obj", red);
  MeshTriangle *left      = new MeshTriangle(projectDir + "models/cornellbox/left.obj", white);
  MeshTriangle *right     = new MeshTriangle(projectDir + "models/cornellbox/right.obj", white);
  MeshTriangle *meshLight = new MeshTriangle(projectDir + "models/cornellbox/light.obj", light);
  Sphere *sphereLight     = new Sphere({0, 0.45f, 1.5f}, 0.05f, light);

  // cout << "mesh light: " << meshLight->getArea() << endl;
  // cout << "sphere light: " << sphereLight->getArea() << endl;

  // Vector3f p, n;
  // float pdf;
  // meshLight->Sample(p, n, pdf);
  // sphereLight->Sample(p, n, pdf);
  // pause();

  scene.Add(floor);
  scene.Add(shortbox);
  scene.Add(tallbox);
  scene.Add(left);
  scene.Add(right);
  scene.Add(meshLight);
  // scene.Add(sphereLight);

  scene.buildBVH();

  Renderer r;

  auto start = std::chrono::system_clock::now();
  r.Render(scene, projectDir);
  auto stop = std::chrono::system_clock::now();

  std::cout << "Render complete: \n";
  std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
  std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
  std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

  string command{"start /d \"" + projectDir + "\" preview.ppm"};
  system(command.c_str());
  // system("pause");

  return 0;
}