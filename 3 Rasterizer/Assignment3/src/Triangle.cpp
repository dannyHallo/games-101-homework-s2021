//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>
#include <array>
#include <eigen3/Eigen/Eigen>

Triangle::Triangle() {
  v[0] << 0, 0, 0, 1;
  v[1] << 0, 0, 0, 1;
  v[2] << 0, 0, 0, 1;

  color[0] << 0.0, 0.0, 0.0;
  color[1] << 0.0, 0.0, 0.0;
  color[2] << 0.0, 0.0, 0.0;

  tex_coords[0] << 0.0, 0.0;
  tex_coords[1] << 0.0, 0.0;
  tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int ind, Vector4f ver) { v[ind] = ver; }
void Triangle::setNormal(int ind, Vector3f n) { normal[ind] = n; }
void Triangle::setColor(int ind, float r, float g, float b) {
  if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) || (b > 255.)) {
    fprintf(stderr, "ERROR! Invalid color values");
    fflush(stderr);
    exit(-1);
  }

  color[ind] = Vector3f{r, g, b};
  return;
}

void Triangle::setTexCoord(int ind, Vector2f uv) { tex_coords[ind] = uv; }

std::array<Vector4f, 3> Triangle::toVector4() const {
  std::array<Vector4f, 3> res;
  std::transform(v, v + 3, res.begin(), [](Vector4f vec) {
    return Vector4f{vec.x(), vec.y(), vec.z(), 1.f};
  });
  return res;
}

void Triangle::setNormals(const std::array<Vector3f, 3> &normals) {
  normal[0] = normals[0];
  normal[1] = normals[1];
  normal[2] = normals[2];
}

void Triangle::setColors(const std::array<Vector3f, 3> &colors) {
  auto first_color = colors[0];
  setColor(0, colors[0][0], colors[0][1], colors[0][2]);
  setColor(1, colors[1][0], colors[1][1], colors[1][2]);
  setColor(2, colors[2][0], colors[2][1], colors[2][2]);
}