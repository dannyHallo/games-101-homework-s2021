//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

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
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
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

  void getSample(BVHBuildNode *node, float p, Intersection &pos, float &pdf);
  void Sample(Intersection &pos, float &pdf);

private:
  uint32_t countBVHLayer() const;
  BVHBuildNode *recursiveBuild(std::vector<Object *> objects);
  static void getIntersection(BVHBuildNode *node, const Ray &ray, Intersection &isect, uint32_t layerCount = 0);
};

struct BVHBuildNode {
  Bounds3 bounds;
  BVHBuildNode *left;
  BVHBuildNode *right;
  Object *object;
  float area;

public:
  int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
  // BVHBuildNode Public Methods
  BVHBuildNode() {
    bounds = Bounds3();
    left   = nullptr;
    right  = nullptr;
    object = nullptr;
  }
};

#endif // RAYTRACING_BVH_H
