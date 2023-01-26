#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include <array>
#include <cassert>

inline bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1, const Vector3f &v2, const Vector3f &orig,
                                 const Vector3f &dir, float &tnear, float &u, float &v) {
  Vector3f edge1 = v1 - v0;
  Vector3f edge2 = v2 - v0;
  Vector3f pvec  = crossProduct(dir, edge2);
  float det      = dotProduct(edge1, pvec);
  if (det <= 0)
    return false;

  Vector3f tvec = orig - v0;
  u             = dotProduct(tvec, pvec);
  if (u < 0 || u > det)
    return false;

  Vector3f qvec = crossProduct(tvec, edge1);
  v             = dotProduct(dir, qvec);
  if (v < 0 || u + v > det)
    return false;

  float invDet = 1 / det;

  tnear = dotProduct(edge2, qvec) * invDet;
  u *= invDet;
  v *= invDet;

  return true;
}

class Triangle : public Object {
public:
  Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
  Vector3f e1, e2;
  Vector2f t0, t1, t2; // texture coords
  Vector3f n;

  Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = nullptr) : v0(_v0), v1(_v1), v2(_v2) {
    e1 = v1 - v0;
    e2 = v2 - v0;
    n  = normalize(crossProduct(e1, e2));
    m  = _m; // variables of base class cannot be initialized from here, but assignment is allowed
  }

  inline bool intersect(const Ray &ray, float &tnear, Vector3f &normal, Vector2f &uv, Object **obj,
                        Material **mat) override {
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
  }

  inline Bounds3 getBounds() const override { return Union(Bounds3(v0, v1), v2); }

  Vector3f evalDiffuseColor(const Vector2f &) const override { return Vector3f(0.5, 0.5, 0.5); }
};

class MeshTriangle : public Object {
public:
  Bounds3 bounding_box;
  std::vector<Triangle> triangles;
  BVHAccel *bvh;

public:
  MeshTriangle(const std::string &filename, const Vector3f position, const float scale = 1.f) {
    objl::Loader loader;
    loader.LoadFile(filename);

    m = new Material{};

    assert(loader.LoadedMeshes.size() == 1);
    auto mesh = loader.LoadedMeshes[0];

    Vector3f min_vert = Vector3f{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()};
    // lowest() denotes the lowest POSSIBLE number (-max())
    // while min() denotes lowest POSITIVE number for floating point types, and lowest POSSIBLE for other types
    Vector3f max_vert = Vector3f{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                                 std::numeric_limits<float>::lowest()};

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

      triangles.emplace_back(Triangle{face_vertices[0], face_vertices[1], face_vertices[2], m});
    }

    bounding_box = Bounds3(min_vert, max_vert);

    // std::cout << "Main bounding box: " << std::endl;

    // each triangle is seen as a object in a bvh
    std::vector<Object *> ptrs;
    for (auto &tri : triangles) {
      ptrs.push_back(&tri);
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

  Bounds3 getBounds() const override { return bounding_box; }

  Vector3f evalDiffuseColor(const Vector2f &st) const override {
    float scale   = 5;
    float pattern = static_cast<float>((fmodf(st.x * scale, 1) > 0.5f) ^ (fmodf(st.y * scale, 1) > 0.5f));
    return lerp(Vector3f(0.815f, 0.235f, 0.031f), Vector3f(0.937f, 0.937f, 0.231f), pattern);
  }
};
