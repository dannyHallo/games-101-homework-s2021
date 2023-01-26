//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>

using namespace Eigen;
using namespace rst;
using namespace std;

pos_buf_id rasterizer::load_positions(const std::vector<Vector3d> &positions) {
  auto id = get_next_id();
  pos_buf.emplace(id, positions);

  return {id};
}

ind_buf_id rasterizer::load_indices(const std::vector<Vector3i> &indices) {
  auto id = get_next_id();
  ind_buf.emplace(id, indices);

  return {id};
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rasterizer::draw_line(Vector3d begin, Vector3d end) {
  auto x1 = static_cast<int>(begin.x());
  auto y1 = static_cast<int>(begin.y());
  auto x2 = static_cast<int>(end.x());
  auto y2 = static_cast<int>(end.y());

  Vector3d line_color = {255, 255, 255};

  int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

  dx  = x2 - x1;
  dy  = y2 - y1;
  dx1 = fabs(dx);
  dy1 = fabs(dy);
  px  = 2 * dy1 - dx1;
  py  = 2 * dx1 - dy1;

  if (dy1 <= dx1) {
    if (dx >= 0) {
      x  = x1;
      y  = y1;
      xe = x2;
    } else {
      x  = x2;
      y  = y2;
      xe = x1;
    }
    Vector3d point = Vector3d(x, y, 1.);

    set_pixel(point, line_color);

    for (i = 0; x < xe; i++) {
      x = x + 1;
      if (px < 0) {
        px = px + 2 * dy1;
      } else {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
          y = y + 1;
        } else {
          y = y - 1;
        }
        px = px + 2 * (dy1 - dx1);
      }

      Vector3d point = Vector3d(x, y, 1.);
      set_pixel(point, line_color);
    }
  } else {
    if (dy >= 0) {
      x  = x1;
      y  = y1;
      ye = y2;
    } else {
      x  = x2;
      y  = y2;
      ye = y1;
    }
    Vector3d point = Vector3d(x, y, 1.);
    set_pixel(point, line_color);
    for (i = 0; y < ye; i++) {
      y = y + 1;
      if (py <= 0) {
        py = py + 2 * dx1;
      } else {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
          x = x + 1;
        } else {
          x = x - 1;
        }
        py = py + 2 * (dx1 - dy1);
      }
      //            delay(0);
      Vector3d point = Vector3d(x, y, 1.);
      set_pixel(point, line_color);
    }
  }
}

auto to_vec4(const Vector3d &v3, double w = 1.) { return Vector4d(v3.x(), v3.y(), v3.z(), w); }

void rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type) {
  if (type != Primitive::Triangle) {
    throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
  }
  auto &buf = pos_buf[pos_buffer.pos_id];
  auto &ind = ind_buf[ind_buffer.ind_id];

  double f1 = (100. - 0.1) / 2.;
  double f2 = (100. + 0.1) / 2.;

  Matrix4d mvp = projection * view * model;
  for (auto &i : ind) {
    Triangle t;

    // cout << "original vertex" << endl;
    // for (int vertId = 0; vertId < 3; ++vertId) {
    //   cout << to_vec4(buf[i[vertId]], 1.) << endl << endl;
    // }
    // cout << endl;

    Vector4d v[] = {mvp * to_vec4(buf[i[0]], 1.), mvp * to_vec4(buf[i[1]], 1.),
                    mvp * to_vec4(buf[i[2]], 1.)};

    for (auto &vec : v) {
      vec /= vec.w();
    }

    // cout << "transformed" << endl;
    // for (int vertId = 0; vertId < 3; ++vertId) {
    //   cout << v[vertId] << endl << endl;
    // }
    // cout << endl;

    for (auto &vert : v) {
      vert.x() = 0.5 * width * (vert.x() + 1.0);
      vert.y() = 0.5 * height * (vert.y() + 1.0);
      vert.z() = vert.z() * f1 + f2;
    }

    // cout << "to screen pix" << endl;
    // for (int vertId = 0; vertId < 3; ++vertId) {
    //   cout << v[vertId] << endl << endl;
    // }
    // cout << endl;

    for (int i = 0; i < 3; ++i) {
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
    }

    t.setColor(0, 255.0, 0.0, 0.0);
    t.setColor(1, 0.0, 255.0, 0.0);
    t.setColor(2, 0.0, 0.0, 255.0);

    rasterize_wireframe(t);
  }
}

void rasterizer::rasterize_wireframe(const Triangle &t) {
  draw_line(t.c(), t.a());
  draw_line(t.c(), t.b());
  draw_line(t.b(), t.a());
}

void rasterizer::set_model(const Matrix4d &m) { model = m; }

void rasterizer::set_view(const Matrix4d &v) { view = v; }

void rasterizer::set_projection(const Matrix4d &p) { projection = p; }

void rasterizer::clear(Buffers buff) {
  if ((buff & Buffers::Color) == Buffers::Color) {
    std::fill(frame_buf.begin(), frame_buf.end(), Vector3f{0, 0, 0});
  }
  if ((buff & Buffers::Depth) == Buffers::Depth) {
    std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<double>::infinity());
  }
}

rasterizer::rasterizer(int w, int h) : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

int rasterizer::get_index(int x, int y) { return (height - y) * width + x; }

void rasterizer::set_pixel(const Vector3d &point, const Vector3d &color) {
  // old index: auto ind = point.y() + point.x() * width;
  if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
    return;
  auto ind       = (height - 1 - point.y()) * width + point.x();
  frame_buf[ind] = color.cast<float>();
}