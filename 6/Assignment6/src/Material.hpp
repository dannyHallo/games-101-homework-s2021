#pragma once

#include "Vector.hpp"

enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION };

class Material {
public:
  MaterialType m_type;
  Vector3f m_color;
  Vector3f m_emission;
  float ior;
  float Kd, Ks, Ka;
  float specularExponent;
  // Texture tex;

  Material(MaterialType t = DIFFUSE_AND_GLOSSY, Vector3f c = Vector3f(1, .83f, .83f), Vector3f e = Vector3f(0, 0, 0));
  MaterialType getType();
  Vector3f getColor();
  Vector3f getColorAt(double u, double v);
  Vector3f getEmission();
};

inline Material::Material(MaterialType t, Vector3f c, Vector3f e)
    : m_type{t}, m_color{c}, m_emission{e}, ior{1.3f}, Kd{0.8f}, Ks{0.2f}, Ka{.1f}, specularExponent{125} {}

inline MaterialType Material::getType() { return m_type; }
inline Vector3f Material::getColor() { return m_color; }
inline Vector3f Material::getEmission() { return m_emission; }

inline Vector3f Material::getColorAt(double u, double v) { return Vector3f(); }
