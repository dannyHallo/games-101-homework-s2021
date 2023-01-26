//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

Triangle::Triangle() {
  v[0] << 0, 0, 0;
  v[1] << 0, 0, 0;
  v[2] << 0, 0, 0;

  color[0] << 0.0, 0.0, 0.0;
  color[1] << 0.0, 0.0, 0.0;
  color[2] << 0.0, 0.0, 0.0;

  tex_coords[0] << 0.0, 0.0;
  tex_coords[1] << 0.0, 0.0;
  tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int ind, Eigen::Vector3d ver) { v[ind] = ver; }

void Triangle::setNormal(int ind, Vector3d n) { normal[ind] = n; }

void Triangle::setColor(int ind, double r, double g, double b) {
  if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) || (b > 255.)) {
    throw std::runtime_error("Invalid color values");
  }

  color[ind] = Vector3d(r / 255., g / 255., b / 255.);
  return;
}
void Triangle::setTexCoord(int ind, double s, double t) { tex_coords[ind] = Vector2d(s, t); }

std::array<Vector4d, 3> Triangle::toVector4() const {
  std::array<Vector4d, 3> res;
  std::transform(std::begin(v), std::end(v), res.begin(),
                 [](auto &vec) { return Vector4d(vec.x(), vec.y(), vec.z(), 1.); });
  return res;
}
