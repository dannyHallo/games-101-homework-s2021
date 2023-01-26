#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <array>
#include <cassert>

inline bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1, const Vector3f &v2, const Vector3f &orig,
                                 const Vector3f &dir, float &tnear, float &u, float &v) {
  Vector3f e1 = v1 - v0;
  Vector3f e2 = v2 - v0;
  Vector3f s  = orig - v0;
  Vector3f s1 = crossProduct(dir, e2);
  Vector3f s2 = crossProduct(s, e1);

  Vector3f sol{dotProduct(s2, e2), dotProduct(s1, s), dotProduct(s2, dir)};
  sol   = sol / dotProduct(s1, e1);
  tnear = sol.x;
  u     = sol.y;
  v     = sol.z;

  if (u >= 0 && v >= 0 && (1 - u - v) >= 0 && tnear > 0)
    return true;
  return false;
}

class Triangle : public Object {
public:
  Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
  Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
  Vector2f t0, t1, t2; // texture coords
  Vector3f n;
  float area;

  Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m) : v0(_v0), v1(_v1), v2(_v2) {
    e1   = v1 - v0;
    e2   = v2 - v0;
    n    = normalize(crossProduct(e1, e2));
    m    = _m;
    area = crossProduct(e1, e2).norm() * 0.5f;
  }

  bool intersect(const Ray &ray, float &tnear, Vector3f &normal, Vector2f &uv, Object **obj, Material **mat) override {
    float t, u, v;
    if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t, u, v)) {
      tnear  = t;
      normal = n;
      uv     = t0 * (1 - u - v) + t1 * u + t2 * v;
      *obj   = this;
      *mat   = m;

      return true;
    }
    return false;
  };

  Vector3f evalDiffuseColor(const Vector2f &) const override { return m->Kd; };

  Bounds3 getBounds() const override { return Union(Bounds3(v0, v1), v2); };

  float getArea() const override { return area; }

  void Sample(Vector3f &pos, Vector3f &normal, float &pdf) const override {
    float x = getRandomFloat(), y = getRandomFloat();
    if (x + y > 1)
      x = 1 - x, y = 1 - y;

    pos    = v0 + x * e1 + y * e2;
    normal = n;
    pdf    = 1.0f / area;

    // std::cout << "  pos: " << pos.toString() << " normal: " << normal.toString() << " pdf: " << pdf << std::endl;
  }

  bool hasEmit() const override { return m->hasEmission(); }

  void print() const {
    std::cout << "  v0: " << v0.toString() << " v1: " << v1.toString() << " v2: " << v2.toString() << std::endl;
  }
};

class MeshTriangle : public Object {
public:
  Bounds3 bounding_box;
  std::vector<Triangle> triangles;
  BVHAccel *bvh;
  float area;

public:
  MeshTriangle(const std::string &filename, Material *mt = new Material(), const Vector3f position = {-0.5f, -0.5f, 1},
               const Vector3f scale = {1 / 552.8f, 1 / 548.8f, 1 / 559.2f}) {
    objl::Loader loader;
    loader.LoadFile(filename);
    area = 0;
    m    = mt;
    assert(loader.LoadedMeshes.size() == 1);
    auto mesh = loader.LoadedMeshes[0];

    Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                                 std::numeric_limits<float>::infinity()};
    Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                                 -std::numeric_limits<float>::infinity()};
    for (int i = 0; i < mesh.Vertices.size(); i += 3) {
      std::array<Vector3f, 3> face_vertices;

      for (int j = 0; j < 3; j++) {
        auto vert = position + Vector3f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y,
                                        mesh.Vertices[i + j].Position.Z) *
                                   scale;
        face_vertices[j] = vert;

        min_vert = Vector3f(std::min(min_vert.x, vert.x), std::min(min_vert.y, vert.y), std::min(min_vert.z, vert.z));
        max_vert = Vector3f(std::max(max_vert.x, vert.x), std::max(max_vert.y, vert.y), std::max(max_vert.z, vert.z));
      }

      triangles.emplace_back(face_vertices[0], face_vertices[1], face_vertices[2], mt);
    }

    bounding_box = Bounds3(min_vert, max_vert);

    std::cout << "Main bounding box: " << std::endl;
    bounding_box.print();

    std::vector<Object *> ptrs;
    for (auto &tri : triangles) {
      ptrs.push_back(&tri);
      area += tri.area;
    }
    bvh = new BVHAccel(ptrs);
  }

  bool intersect(const Ray &ray, float &tnear, Vector3f &normal, Vector2f &uv, Object **obj, Material **mat) override {
    Intersection isect = bvh->Intersect(ray);

    if (!isect.happened)
      return false;

    tnear  = isect.distance;
    normal = isect.normal;
    uv     = isect.uv;
    *obj   = isect.obj;
    *mat   = isect.m;

    return true;
  }

  Vector3f evalDiffuseColor(const Vector2f &st) const override {
    float scale   = 5;
    float pattern = static_cast<float>((fmodf(st.x * scale, 1) > 0.5f) ^ (fmodf(st.y * scale, 1) > 0.5f));
    return lerp(Vector3f(0.815f, 0.235f, 0.031f), Vector3f(0.937f, 0.937f, 0.231f), pattern);
  }

  Bounds3 getBounds() const override { return bounding_box; }

  float getArea() const override { return area; }

  void Sample(Vector3f &pos, Vector3f &normal, float &pdf) const override {
    const Triangle &randomSelectedTri = triangles[getRandomInt(0, triangles.size())];
    randomSelectedTri.Sample(pos, normal, pdf);
  }

  bool hasEmit() const override { return m->hasEmission(); }

  void print() const {
    std::cout << "tri count: " << triangles.size() << std::endl;
    for (const auto &tri : triangles) {
      tri.print();
    }
  }
};