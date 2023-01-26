#include "BVH.hpp"
#include <algorithm>
#include <cassert>

/// @brief Initializes BVH structure for the scene objects
/// @param p all objects in the scene
/// @param maxPrimsInNode not in use
/// @param splitMethod not in use
BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode, SplitMethod splitMethod)
    : primitives(p), maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod) {
  assert(!primitives.empty() && "Primitives must not be empty to build a BVH structure!");

  std::cout << "Generating BVH with " << primitives.size() << " objects" << std::endl;

  root = recursiveBuild(primitives);

  std::cout << "Created BVH with " << countBVHLayer() << " layers\n" << std::endl;
}

uint32_t BVHAccel::countBVHLayer() const {
  uint32_t layerCount = 1;
  BVHBuildNode *node  = root;
  // the right node is always allocated first
  while (node->right) {
    node = node->right;
    ++layerCount;
  }

  return layerCount;
}

/// @brief Sort all objects by their position recursivelly
/// @param objects the object list to be sorted
/// @return the root node
BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects) {
  BVHBuildNode *node = new BVHBuildNode();

  // Compute bounds of all primitives in BVH node
  Bounds3 bounds;
  for (int i = 0; i < objects.size(); ++i)
    bounds = Union(bounds, objects[i]->getBounds());

  // one object left: store the object's pointer and its bound, no further work is required (empty child nodes)
  if (objects.size() == 1) {
    // Create leaf _BVHBuildNode_
    node->bounds = objects[0]->getBounds();
    node->object = objects[0];
    node->left   = nullptr;
    node->right  = nullptr;
    return node;
  }

  // 2 object is left: the object with the lowest val in the previous sort is allocated to left
  else if (objects.size() == 2) {
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
      centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
    int dim = centroidBounds.maxExtent();
    switch (dim) {
      // x is the biggest dim -> sort all objects by x val
    case 0:
      std::sort(objects.begin(), objects.end(),
                [](auto f1, auto f2) { return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x; });
      break;
      // y is ..
    case 1:
      std::sort(objects.begin(), objects.end(),
                [](auto f1, auto f2) { return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y; });
      break;
      // z is ..
    case 2:
      std::sort(objects.begin(), objects.end(),
                [](auto f1, auto f2) { return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z; });
      break;
    }

    node->left  = recursiveBuild(std::vector{objects[0]});
    node->right = recursiveBuild(std::vector{objects[1]});

    node->bounds = Union(node->left->bounds, node->right->bounds);
    return node;
  }

  // more than 2 object left: sort them by their longest dim, and split them in half
  else {
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
      centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
    int dim = centroidBounds.maxExtent();
    switch (dim) {
      // x is the biggest dim -> sort all objects by x val
    case 0:
      std::sort(objects.begin(), objects.end(),
                [](auto f1, auto f2) { return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x; });
      break;
      // y is ..
    case 1:
      std::sort(objects.begin(), objects.end(),
                [](auto f1, auto f2) { return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y; });
      break;
      // z is ..
    case 2:
      std::sort(objects.begin(), objects.end(),
                [](auto f1, auto f2) { return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z; });
      break;
    }

    auto beginning = objects.begin();
    auto middling  = objects.begin() + (objects.size() / 2);
    auto ending    = objects.end();

    auto leftshapes  = std::vector<Object *>(beginning, middling);
    auto rightshapes = std::vector<Object *>(middling, ending);

    assert(objects.size() == (leftshapes.size() + rightshapes.size()));

    node->left  = recursiveBuild(leftshapes);
    node->right = recursiveBuild(rightshapes);

    node->bounds = Union(node->left->bounds, node->right->bounds);
  }

  return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const {
  assert(root && "BVH must be built before intersection check!");

  Intersection isect{};
  uint32_t layerCount = 0;
  getIntersection(root, ray, isect, layerCount);
  return isect;
}

void BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray, Intersection &isect, uint32_t layerCount) {
  // std::cout << "layer " << layerCount++ << std::endl;

  if (node->bounds.IntersectP(ray)) {
    float tNear = std::numeric_limits<float>::max();

    // this is a leaf node & the object stored inside the node is hit
    if (node->object) {
      Vector2f uv{};
      Vector3f normal{};
      Object *object;
      Material *material;

      if (node->object->intersect(ray, tNear, normal, uv, &object, &material)) {
        // make sure only the nearest intersection will update data
        if (isect.distance > tNear) {
          isect.happened = true;
          isect.coords   = ray.origin + tNear * ray.direction;
          isect.distance = tNear;
          isect.uv       = uv;
          isect.normal   = normal;
          isect.obj      = object;
          isect.m        = material;
        }
        // isect.print();
      }
    }

    // the left and right node will be allocated at the same time
    if (node->left) {
      getIntersection(node->left, ray, isect, layerCount);
      getIntersection(node->right, ray, isect, layerCount);
    }
  }
}