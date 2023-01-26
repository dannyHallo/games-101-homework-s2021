#include <iostream>
#include <opencv2/opencv.hpp>

#include "OBJ_Loader.h"
#include "Shader.hpp"
#include "Texture.hpp"
#include "Triangle.hpp"
#include "custom_math.hpp"
#include "global.hpp"
#include "rasterizer.hpp"

using namespace std;
using namespace Eigen;

constexpr double MY_PI    = 3.1415926;
constexpr double DegToRad = 0.0174533;

const string ProjectDir{"E:/Computer Graphics/GAMES101_Homework_S2021/3/Assignment3"};

Matrix4f get_view_matrix(Vector3f eye_pos) {
  Matrix4f view = Matrix4f::Identity();

  Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

  view = translate * view;

  return view;
}

/// @brief calculates the model transform matrix, only simple rotation is applied here
/// @param angle rotating angle around y axis
/// @return model transform matrix
Matrix4f get_model_matrix(float angle) {
  float scalingFac = 1;

  Matrix4f rotation{};
  Matrix4f scale{};
  Matrix4f translate{};

  angle = static_cast<float>(DegToRad * angle);

  rotation << cos(angle), 0, sin(angle), 0, 0, 1, 0, 0, -sin(angle), 0, cos(angle), 0, 0, 0, 0, 1;
  scale << scalingFac, 0, 0, 0, 0, scalingFac, 0, 0, 0, 0, scalingFac, 0, 0, 0, 0, 1;
  translate << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  return translate * rotation * scale;
}

Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
  zNear = -zNear;
  zFar  = -zFar;

  float a = 2 * tan(eye_fov / 2.f) * fabs(zNear);
  float b = a * aspect_ratio;

  Matrix4f projection = Matrix4f::Identity();

  Matrix4f perspToOrtho{};
  perspToOrtho << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear * zFar, 0, 0, 1.f, 0;

  Matrix4f translation{};
  translation << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -(zNear + zFar) / 2.f, 0, 0, 0, 1;

  Matrix4f scaling{};
  scaling << 2.f / a, 0, 0, 0, 0, 2.f / b, 0, 0, 0, 0, 2.f / (zNear - zFar), 0, 0, 0, 0, 1.f;

  projection = scaling * translation * perspToOrtho * projection;

  return projection;
}

// not in use at this time
Vector3f vertex_shader(const vertex_shader_payload &payload) { return payload.position; }

// Simple visualization of viewport normal
Vector3f normal_fragment_shader(const fragment_shader_payload &payload) {
  Vector3f col = (payload.normal + Vector3f{1.f, 1.f, 1.f}) / 2.f;
  return {col.x() * 255, col.y() * 255, col.z() * 255};
}

static Vector3f reflect(const Vector3f &vec, const Vector3f &axis) {
  auto costheta = vec.dot(axis);
  return (2 * costheta * axis - vec).normalized();
}

struct light {
  Vector3f position;
  Vector3f intensity;
};

Vector3f phong_fragment_shader(const fragment_shader_payload &payload) {
  Vector3f ka = Vector3f(0.005f, 0.005f, 0.005f);
  Vector3f kd = payload.color;
  Vector3f ks = Vector3f(0.7937f, 0.7937f, 0.7937f);

  auto l1 = light{{-20, 0, 0}, {100, 100, 100}};
  auto l2 = light{{20, 0, 0}, {100, 100, 100}};

  vector<light> lights         = {l1, l2};
  Vector3f amb_light_intensity = {10, 10, 10};
  Vector3f eye_pos             = {0, 0, 10};
  float p                      = 150;

  Vector3f result_color = {0, 0, 0};
  for (auto &light : lights) {
    Vector3f v = eye_pos - payload.view_pos;        // un-normalized
    Vector3f n = payload.normal;                    // normalized
    Vector3f l = light.position - payload.view_pos; // un-normalized
    Vector3f h = (l + v).normalized();              // normalized

    float r = v.norm();

    float dotNL = n.dot(l.normalized());
    float dotNH = n.dot(h);

    Vector3f diffuse{};
    Vector3f specular{};
    Vector3f ambient{};
    for (int i = 0; i < 3; i++) {
      float intensityFalloff = light.intensity[i] / (r * r); // Ranges from 0 - 1

      diffuse[i]  = kd[i] * intensityFalloff * (dotNL > 0 ? dotNL : 0);
      specular[i] = ks[i] * intensityFalloff * (dotNH > 0 ? static_cast<float>(pow(dotNH, p)) : 0);
      ambient[i]  = ka[i] * amb_light_intensity[i];
    }

    result_color += diffuse + specular + ambient;
  }

  return result_color * 255.f;
}

Vector3f texture_fragment_shader(const fragment_shader_payload &payload) {
  Vector3f ka = Vector3f(0.005f, 0.005f, 0.005f);
  Vector3f kd{};
  Vector3f ks = Vector3f(0.7937f, 0.7937f, 0.7937f);

  auto l1 = light{{-20, 20, -20}, {200, 200, 200}};
  auto l2 = light{{20, 20, -20}, {200, 200, 200}};
  auto l3 = light{{-20, 20, 20}, {200, 200, 200}};
  auto l4 = light{{20, 20, 20}, {200, 200, 200}};

  vector<light> lights         = {l1, l2, l3, l4};
  Vector3f amb_light_intensity = {10, 10, 10};
  Vector3f eye_pos             = {0, 0, 10};
  float p                      = 150;

  if (payload.texture) {
    kd = payload.texture->getColor(payload.tex_coords);
  } else {
    kd = payload.color;
  }

  Vector3f result_color = {0, 0, 0};
  for (auto &light : lights) {
    Vector3f v = eye_pos - payload.view_pos;        // un-normalized
    Vector3f n = payload.normal;                    // normalized
    Vector3f l = light.position - payload.view_pos; // un-normalized
    Vector3f h = (l + v).normalized();              // normalized

    float r = v.norm();

    float dotNL = n.dot(l.normalized());
    float dotNH = n.dot(h);

    Vector3f diffuse{};
    Vector3f specular{};
    Vector3f ambient{};
    for (int i = 0; i < 3; i++) {
      float intensityFalloff = light.intensity[i] / (r * r); // Ranges from 0 - 1

      diffuse[i]  = kd[i] * intensityFalloff * (dotNL > 0 ? dotNL : 0);
      specular[i] = ks[i] * intensityFalloff * (dotNH > 0 ? static_cast<float>(pow(dotNH, p)) : 0);
      ambient[i]  = ka[i] * amb_light_intensity[i];
    }

    result_color += diffuse + specular + ambient;
  }

  return result_color * 255.f;
}

Vector3f bump_fragment_shader(const fragment_shader_payload &payload) {

  Vector3f ka = Vector3f(0.005f, 0.005f, 0.005f);
  Vector3f kd = payload.color;
  Vector3f ks = Vector3f(0.7937f, 0.7937f, 0.7937f);

  auto l1 = light{{-20, 0, 0}, {100, 100, 100}};
  auto l2 = light{{20, 0, 0}, {100, 100, 100}};

  vector<light> lights = {l1, l2};

  Vector3f amb_light_intensity{10, 10, 10};
  Vector3f eye_pos{0, 0, 10};

  float p = 150;

  Vector3f color = payload.color;
  Vector3f point = payload.view_pos;
  Vector3f n     = payload.normal;

  float kh = 0.2f, kn = .1f;

  Vector3f t = {n.x() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()), sqrt(n.x() * n.x() + n.z() * n.z()),
                n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z())};
  Vector3f b = n.cross(t);
  Matrix3f TBN{};
  TBN << t, b, n;

  float u = payload.tex_coords.x();
  float v = payload.tex_coords.y();
  float w = static_cast<float>(payload.texture->width);
  float h = static_cast<float>(payload.texture->height);

  float dU =
      kh * kn * (payload.texture->getColor(u + 1 / w, v).norm() - payload.texture->getColor(u, v).norm());
  float dV =
      kh * kn * (payload.texture->getColor(u, v + 1 / h).norm() - payload.texture->getColor(u, v).norm());

  Vector3f ln{-dU, -dV, 1 / w};
  n = (TBN * ln).normalized();

  Vector3f result_color = {0, 0, 0};
  result_color          = n;

  return result_color * 255.f;
}

Vector3f displacement_fragment_shader(const fragment_shader_payload &payload) {

  Vector3f ka = Vector3f(0.005f, 0.005f, 0.005f);
  Vector3f kd = payload.color;
  Vector3f ks = Vector3f(0.7937f, 0.7937f, 0.7937f);

  auto l1 = light{{-20, 0, 0}, {100, 100, 100}};
  auto l2 = light{{20, 0, 0}, {100, 100, 100}};

  vector<light> lights = {l1, l2};

  Vector3f amb_light_intensity{10, 10, 10};
  Vector3f eye_pos{0, 0, 10};

  float p = 150;

  Vector3f color = payload.color;
  Vector3f point = payload.view_pos;
  Vector3f n     = payload.normal;

  float kh = 0.2f, kn = .02f;

  Vector3f t = {n.x() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()), sqrt(n.x() * n.x() + n.z() * n.z()),
                n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z())};
  Vector3f b = n.cross(t);
  Matrix3f TBN{};
  TBN << t, b, n;

  float u = payload.tex_coords.x();
  float v = payload.tex_coords.y();
  float w = static_cast<float>(payload.texture->width);
  float h = static_cast<float>(payload.texture->height);

  float dU =
      kh * kn * (payload.texture->getColor(u + 1 / w, v).norm() - payload.texture->getColor(u, v).norm());
  float dV =
      kh * kn * (payload.texture->getColor(u, v + 1 / h).norm() - payload.texture->getColor(u, v).norm());

  Vector3f ln{-dU, -dV, 1 / w};
  n = (TBN * ln).normalized();

  Vector3f result_color = {0, 0, 0};

  for (auto &light : lights) {
    Vector3f v = eye_pos - payload.view_pos; // un-normalized
    // Vector3f n = payload.normal;                    // normalized
    Vector3f l = light.position - payload.view_pos; // un-normalized
    Vector3f h = (l + v).normalized();              // normalized

    float r = v.norm();

    float dotNL = n.dot(l.normalized());
    float dotNH = n.dot(h);

    Vector3f diffuse{};
    Vector3f specular{};
    Vector3f ambient{};
    for (int i = 0; i < 3; i++) {
      float intensityFalloff = light.intensity[i] / (r * r); // Ranges from 0 - 1

      diffuse[i]  = kd[i] * intensityFalloff * (dotNL > 0 ? dotNL : 0);
      specular[i] = ks[i] * intensityFalloff * (dotNH > 0 ? static_cast<float>(pow(dotNH, p)) : 0);
      ambient[i]  = ka[i] * amb_light_intensity[i];
    }

    result_color += diffuse + specular + ambient;
  }

  return result_color * 255.f;
}

int main(int argc, const char **argv) {
  vector<Triangle *> TriangleList;

  float angle      = 140.f;
  bool commandLine = false;

  const int screenWidth  = 700;
  const int screenHeight = 700;

  string filename = "output.png";
  objl::Loader Loader;

  string objPath = ProjectDir + "/models/cube/cube.obj";
  // string texturePath = ProjectDir + "/models/spot/hmap.jpg";
  string texturePath = ProjectDir + "/models/cube/bump2.png";
  // string texturePath = ProjectDir + "/models/cube/grid.png";

  // function<Vector3f(fragment_shader_payload)> activeShader = bump_fragment_shader;
  function<Vector3f(fragment_shader_payload)> activeShader = displacement_fragment_shader;

  // Load .obj File
  bool loadout = Loader.LoadFile(objPath);

  // Num of separable meshes stored in .obj file
  for (auto mesh : Loader.LoadedMeshes) {
    // Num of tris inside a separable mesh
    for (int i = 0; i < mesh.Vertices.size(); i += 3) {
      Triangle *t = new Triangle();
      for (int j = 0; j < 3; j++) {
        t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y,
                                 mesh.Vertices[i + j].Position.Z, 1.0));
        t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y,
                                 mesh.Vertices[i + j].Normal.Z));
        t->setTexCoord(
            j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
      }
      TriangleList.push_back(t);
    }
  }

  cout << "num of tris: " << TriangleList.size() << endl;

  rst::rasterizer r(screenWidth, screenHeight);

  //   if (argc >= 2) {
  //     commandLine = true;
  //     filename    = string(argv[1]);

  //     if (argc == 3 && string(argv[2]) == "texture") {
  //       cout << "Rasterizing using the texture shader\n";
  //       activeShader = texture_fragment_shader;
  //       texturePath  = ProjectDir + "/models/spot/spot_texture.png";
  //       r.set_texture(Texture(texturePath));
  //     } else if (argc == 3 && string(argv[2]) == "normal") {
  //       cout << "Rasterizing using the normal shader\n";
  //       activeShader = normal_fragment_shader;
  //     } else if (argc == 3 && string(argv[2]) == "phong") {
  //       cout << "Rasterizing using the phong shader\n";
  //       activeShader = phong_fragment_shader;
  //     } else if (argc == 3 && string(argv[2]) == "bump") {
  //       cout << "Rasterizing using the bump shader\n";
  //       activeShader = bump_fragment_shader;
  //     } else if (argc == 3 && string(argv[2]) == "displacement") {
  //       cout << "Rasterizing using the bump shader\n";
  //       activeShader = displacement_fragment_shader;
  //     }
  //   }

  Vector3f eye_pos = {0, 0, 10};

  r.set_vertex_shader(vertex_shader);
  r.set_fragment_shader(activeShader);
  r.set_texture(Texture(texturePath));

  int key         = 0;
  int frame_count = 0;

  //   if (commandLine) {
  //     r.clear(rst::Buffers::Color | rst::Buffers::Depth);
  //     r.set_model(get_model_matrix(angle));
  //     r.set_view(get_view_matrix(eye_pos));
  //     r.set_projection(get_projection_matrix(45.f, 1.f, 0.1f, 50.f));

  //     r.draw(TriangleList);
  //     cv::Mat image(screenWidth, screenHeight, CV_32FC3, r.frame_buffer().data());
  //     image.convertTo(image, CV_8UC3, 1.0f);
  //     cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

  //     cv::imwrite(filename, image);

  //     string str;
  //     cin >> str;

  //     return 0;
  //   }

  cout << "---- rendering loop begin ----" << endl;

  // while loop here
  while (key != 27) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45.f, 1.f, 0.1f, 50.f));

    r.draw(TriangleList);
    cv::Mat image(screenWidth, screenHeight, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    cv::imshow("image", image);
    // cv::imwrite(filename, image);
    std::cout << "frame count: " << frame_count++ << '\n';

    key = cv::waitKey(10);
    if (key == 'a') {
      angle -= 5;
    } else if (key == 'd') {
      angle += 5;
    }
  }

  return 0;
}
