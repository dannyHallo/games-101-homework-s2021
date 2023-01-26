//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include "custom_math.hpp"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace rst;
using namespace Eigen;

pos_buf_id rasterizer::load_positions(const vector<Vector3f> &positions) {
  auto id = get_next_id();
  pos_buf.emplace(id, positions);

  return {id};
}

ind_buf_id rasterizer::load_indices(const vector<Vector3i> &indices) {
  auto id = get_next_id();
  ind_buf.emplace(id, indices);

  return {id};
}

col_buf_id rasterizer::load_colors(const vector<Vector3f> &cols) {
  auto id = get_next_id();
  col_buf.emplace(id, cols);

  return {id};
}

col_buf_id rasterizer::load_normals(const vector<Vector3f> &normals) {
  auto id = get_next_id();
  nor_buf.emplace(id, normals);

  normal_id = id;

  return {id};
}

// Bresenham's line drawing algorithm
void rasterizer::draw_line(Vector3f begin, Vector3f end) {
  int x1 = static_cast<int>(begin.x());
  int y1 = static_cast<int>(begin.y());
  int x2 = static_cast<int>(end.x());
  int y2 = static_cast<int>(end.y());

  Vector3f line_color = {255, 255, 255};

  int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

  dx  = x2 - x1;
  dy  = y2 - y1;
  dx1 = static_cast<int>(fabs(dx));
  dy1 = static_cast<int>(fabs(dy));
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
    Vector2i point = Vector2i(x, y);
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
      //            delay(0);
      Vector2i point = Vector2i(x, y);
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
    Vector2i point = Vector2i(x, y);
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
      Vector2i point = Vector2i(x, y);
      set_pixel(point, line_color);
    }
  }
}

auto to_vec4(const Vector3f &v3, float w = 1.0f) { return Vector4f(v3.x(), v3.y(), v3.z(), w); }

static bool insideTriangle(int x, int y, const Vector4f *_v) {
  Vector3f dot{};
  dot << static_cast<float>(x), static_cast<float>(y), 0;

  Vector3f vert0 = _v[0].head<3>();
  vert0.z()      = 0;
  Vector3f vert1 = _v[1].head<3>();
  vert1.z()      = 0;
  Vector3f vert2 = _v[2].head<3>();
  vert2.z()      = 0;

  Vector3f v0 = vert1 - vert0;
  Vector3f v1 = vert2 - vert1;
  Vector3f v2 = vert0 - vert2;

  Vector3f d0 = dot - vert0;
  Vector3f d1 = dot - vert1;
  Vector3f d2 = dot - vert2;

  float crossZ0 = (d0.cross(v0)).z();
  float crossZ1 = (d1.cross(v1)).z();
  float crossZ2 = (d2.cross(v2)).z();

  if ((crossZ0 > 0 && crossZ1 > 0 && crossZ2 > 0) || (crossZ0 < 0 && crossZ1 < 0 && crossZ2 < 0)) {
    return true;
  }
  return false;
}

static tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f *v) {
  float c1 =
      (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
      (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() -
       v[2].x() * v[1].y());
  float c2 =
      (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
      (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() -
       v[0].x() * v[2].y());
  float c3 =
      (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
      (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() -
       v[1].x() * v[0].y());
  return {c1, c2, c3};
}

// https://blog.csdn.net/caihaihua0572/article/details/117353526
static tuple<float, float, float> computeBarycentric3D(const array<Vector3f, 3> &v, const Vector3f &p) {
  const Vector3f &a = v[0];
  const Vector3f &b = v[1];
  const Vector3f &c = v[2];

  Vector3f n     = (b - a).cross(c - a);
  float nNormPow = n.dot(n);

  Vector3f na = (c - b).cross(p - b);
  Vector3f nb = (a - c).cross(p - c);
  Vector3f nc = (b - a).cross(p - a);

  float c1 = n.dot(na) / nNormPow;
  float c2 = n.dot(nb) / nNormPow;
  float c3 = n.dot(nc) / nNormPow;

  return {c1, c2, c3};
}

void rasterizer::draw(vector<Triangle *> &TriangleList) {

  float f1 = (50 - 0.1f) / 2.f;
  float f2 = (50 + 0.1f) / 2.f;

  Matrix4f mv  = view * model;
  Matrix4f mvp = projection * mv;

  for (const auto &t : TriangleList) {
    Triangle newtri = *t;

    array<Vector4f, 3> mm{(mv * t->v[0]), (mv * t->v[1]), (mv * t->v[2])};
    array<Vector3f, 3> viewspacePos;

    // Drop away w and store the transformed vectors
    transform(mm.begin(), mm.end(), viewspacePos.begin(),
              [](Vector4f &v) -> Vector3f { return v.head<3>(); });

    // Gets screen space position of three points of the current triangle
    Vector4f v[] = {mvp * t->v[0], mvp * t->v[1], mvp * t->v[2]};
    for (auto &vec : v) {
      vec /= vec.w();
    }

    Vector4f n[] = {mv * to_vec4(t->normal[0], 0.0f), mv * to_vec4(t->normal[1], 0.0f),
                    mv * to_vec4(t->normal[2], 0.0f)};

    // viewport transformation
    for (auto &vert : v) {
      vert.x() = 0.5f * width * (vert.x() + 1.f);
      vert.y() = 0.5f * height * (vert.y() + 1.f);
      // vert.z() = vert.z() * f1 + f2;
    }

    for (int i = 0; i < 3; ++i) {
      // screen space coordinates
      newtri.setVertex(i, v[i]);
      // view space normal
      newtri.setNormal(i, n[i].head<3>());
    }

    newtri.setColor(0, .4f, 1, 1);
    newtri.setColor(1, .4f, 1, 1);
    newtri.setColor(2, .4f, 1, 1);

    // also pass view space vertice position
    rasterize_triangle(newtri, viewspacePos);
  }
}

static Vector4f interpolate(float alpha, float beta, float gamma, const Vector4f &vert1,
                            const Vector4f &vert2, const Vector4f &vert3, float weight) {
  return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Vector3f interpolate(float alpha, float beta, float gamma, const Vector3f &vert1,
                            const Vector3f &vert2, const Vector3f &vert3, float weight) {
  return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Vector2f interpolate(float alpha, float beta, float gamma, const Vector2f &vert1,
                            const Vector2f &vert2, const Vector2f &vert3, float weight) {
  return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

// Screen space rasterization
void rasterizer::rasterize_triangle(const Triangle &t, const array<Vector3f, 3> &view_pos) {
  // DONE: Inside your rasterization loop:
  //    * v[i].w() is the vertex view space depth value z.
  //    * Z is interpolated view space depth for the current pixel
  //    * zp is depth between zNear and zFar, used for z-buffer

  auto &v = t.v;

  int minBoundX  = numeric_limits<int>().infinity();
  int minBoundY  = numeric_limits<int>().infinity();
  int maxBoundX  = 0;
  int maxBoundY  = 0;
  float nearestZ = numeric_limits<float>().infinity();

  for (int i = 0; i < 3; i++) {
    minBoundX = v[i].x() < minBoundX ? static_cast<int>(v[i].x()) : minBoundX;
    minBoundY = v[i].y() < minBoundY ? static_cast<int>(v[i].y()) : minBoundY;
    maxBoundX = v[i].x() > maxBoundX ? static_cast<int>(v[i].x()) + 1 : maxBoundX;
    maxBoundY = v[i].y() > maxBoundY ? static_cast<int>(v[i].y()) + 1 : maxBoundY;
    nearestZ  = -v[i].z() < nearestZ ? -v[i].z() : nearestZ;
  }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      if (x < minBoundX || x > maxBoundX || y < minBoundY || y > maxBoundY)
        continue;

      Vector2i pixPoint{x, y};
      int ind = (height - 1 - pixPoint.y()) * width + pixPoint.x();

      if (nearestZ > depth_buf[ind])
        continue;

      if (insideTriangle(x, y, v)) {
        auto [alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(x), static_cast<float>(y), v);
        float Z                   = alpha + beta + gamma;
        float zp                  = interpolate(alpha, beta, gamma, v[0], v[1], v[2], Z).z();
        float interpolatedZ       = -zp;

        if (interpolatedZ < depth_buf[ind]) {
          depth_buf[ind] = interpolatedZ;

          auto &c   = t.color;
          auto &n   = t.normal;
          auto &tex = t.tex_coords;

          Vector4f pixInProjectionSpace{(2.f * static_cast<float>(x)) / width - 1.f,
                                        (2.f * static_cast<float>(y)) / height - 1.f, zp, 1.f};
          Vector4f pixInViewPos = projection.inverse() * pixInProjectionSpace;
          pixInViewPos /= pixInViewPos.w();

          auto [alpha2, beta2, gamma2] =
              computeBarycentric3D(view_pos, static_cast<Vector3f>(pixInViewPos.head<3>()));

          // cout << "bary1:" << endl;
          // cout << alpha << endl;
          // cout << beta << endl;
          // cout << gamma << endl;

          // cout << "bary2:" << endl;
          // cout << alpha2 << endl;
          // cout << beta2 << endl;
          // cout << gamma2 << endl;

          // string aa;
          // cin >> aa;

          // DONE: Interpolate the attributes:
          auto interpolated_color     = interpolate(alpha2, beta2, gamma2, c[0], c[1], c[2], Z);
          auto interpolated_normal    = interpolate(alpha2, beta2, gamma2, n[0], n[1], n[2], Z).normalized();
          auto interpolated_texcoords = interpolate(alpha2, beta2, gamma2, tex[0], tex[1], tex[2], Z);
          auto interpolated_shadingcoords =
              interpolate(alpha2, beta2, gamma2, view_pos[0], view_pos[1], view_pos[2], Z);

          fragment_shader_payload payload(interpolated_color, interpolated_normal, interpolated_texcoords,
                                          texture ? &*texture : nullptr);
          payload.view_pos = interpolated_shadingcoords;

          auto pixel_color = fragment_shader(payload);
          set_pixel(pixPoint, pixel_color);
        }
      }
    }
  }
}

void rasterizer::set_model(const Matrix4f &m) { model = m; }

void rasterizer::set_view(const Matrix4f &v) { view = v; }

void rasterizer::set_projection(const Matrix4f &p) { projection = p; }

void rasterizer::clear(Buffers buff) {
  if ((buff & Buffers::Color) == Buffers::Color) {
    fill(frame_buf.begin(), frame_buf.end(), Vector3f{0, 0, 0});
  }
  if ((buff & Buffers::Depth) == Buffers::Depth) {
    fill(depth_buf.begin(), depth_buf.end(), numeric_limits<float>::infinity());
  }
}

rasterizer::rasterizer(int w, int h) : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);

  texture = nullopt;

  clear(Buffers::Depth | Buffers::Color);
}

int rasterizer::get_index(int x, int y) { return (height - y) * width + x; }

void rasterizer::set_pixel(const Vector2i &point, const Vector3f &color) {
  // old index: auto ind = point.y() + point.x() * width;
  int ind        = (height - 1 - point.y()) * width + point.x();
  frame_buf[ind] = color;
}

void rasterizer::set_vertex_shader(function<Vector3f(vertex_shader_payload)> vert_shader) {
  vertex_shader = vert_shader;
}

void rasterizer::set_fragment_shader(function<Vector3f(fragment_shader_payload)> frag_shader) {
  fragment_shader = frag_shader;
}
