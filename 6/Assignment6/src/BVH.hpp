#pragma once

#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include <atomic>
#include <ctime>
#include <memory>
#include <vector>

struct BVHBuildNode;
struct BVHPrimitiveInfo;

class BVHAccel {
public:
  enum class SplitMethod { NAIVE, SAH };
  BVHBuildNode *root;

private:
  const int maxPrimsInNode;
  const SplitMethod splitMethod;
  std::vector<Object *> primitives;

public:
  BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
  ~BVHAccel();

  Bounds3 WorldBound() const;
  Intersection Intersect(const Ray &ray) const;
  bool IntersectP(const Ray &ray) const;
  uint32_t countBVHLayer() const;

private:
  BVHBuildNode *recursiveBuild(std::vector<Object *> objects);
  static void getIntersection(BVHBuildNode *node, const Ray &ray, Intersection &isect, uint32_t layerCount = 0);
};

struct BVHBuildNode {
  Bounds3 bounds;
  BVHBuildNode *left;
  BVHBuildNode *right;
  Object *object;
  int splitAxis, firstPrimOffset, nPrimitives;

public:
  // BVHBuildNode Public Methods
  BVHBuildNode() : splitAxis{0}, firstPrimOffset{0}, nPrimitives{0} {
    bounds = Bounds3();
    left   = nullptr;
    right  = nullptr;
    object = nullptr;
  }
};
