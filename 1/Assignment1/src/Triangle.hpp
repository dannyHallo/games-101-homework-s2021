//
// Created by LEI XU on 4/11/19.
//

#pragma once

#include <eigen3/Eigen/Eigen>

using namespace Eigen;

class Triangle {
public:
  Vector3d v[3]; /*the original coordinates of the triangle, v0, v1, v2 in
                    counter clockwise order*/
  /*Per vertex values*/
  Vector3d color[3];      // color at each vertex;
  Vector2d tex_coords[3]; // texture u,v
  Vector3d normal[3];     // normal vector for each vertex

  // Texture *tex;
  Triangle();

  Vector3d a() const { return v[0]; }
  Vector3d b() const { return v[1]; }
  Vector3d c() const { return v[2]; }

  void setVertex(int ind, Vector3d ver);                /*set i-th vertex coordinates */
  void setNormal(int ind, Vector3d n);                  /*set i-th vertex normal vector*/
  void setColor(int ind, double r, double g, double b); /*set i-th vertex color*/
  void setTexCoord(int ind, double s, double t);        /*set i-th vertex texture coordinate*/
  std::array<Vector4d, 3> toVector4() const;
};
