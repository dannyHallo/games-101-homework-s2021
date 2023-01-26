#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

using namespace std;

const string projectDir{"E:/Computer Graphics/GAMES101_Homework_S2021/6/Assignment6/"};

int main(int argc, char **argv) {
  Scene scene(1000, 800);

  MeshTriangle *bunny = new MeshTriangle{projectDir + "models/bunny/bunny.obj", {1, -5, -10}, 50};
  MeshTriangle *cube  = new MeshTriangle{projectDir + "models/cube/cube.obj", {0, 0, -10}};
  Sphere *testBall    = new Sphere{Vector3f(0, 0, -10), 2};
  Sphere *testBall2   = new Sphere{Vector3f(4, 0, -6), 2};

  scene.Add(bunny);
  // scene.Add(cube);
  // scene.Add(testBall);
  // scene.Add(testBall2);
  scene.Add(make_unique<Light>(Vector3f(10, 0, -2), 1));

  cout << "\n--------- building scene BVH ---------\n" << endl;
  scene.buildBVH();

  Renderer r;

  auto start = chrono::system_clock::now();
  r.Render(scene, projectDir);
  auto stop = chrono::system_clock::now();

  cout << "Render complete: \n";
  cout << "Time taken: " << chrono::duration_cast<chrono::hours>(stop - start).count() << " hours\n";
  cout << "          : " << chrono::duration_cast<chrono::minutes>(stop - start).count() << " minutes\n";
  cout << "          : " << chrono::duration_cast<chrono::seconds>(stop - start).count() << " seconds\n";

  string command{"start /d \"" + projectDir + "\" preview.ppm"};
  system(command.c_str());
  // system("pause");

  return 0;
}