#ifndef SIMULATOR_CONVERSIONS_H_
#define SIMULATOR_CONVERSIONS_H_

#include <iostream>

#include "Matrix.h"
#include "Quaternion.h"
#include "Transformation.h"
#include "Vector.h"

#include "glutils/color.h"
#include "glutils/gltraits.h"

using namespace mathtool;

/*--------------------- Conversion from PMPL to glutils ----------------------*/

inline glutils::transform ToGLUtils(const Transformation& _t) {
  const auto& r = _t.rotation().matrix();
  const auto& t = _t.translation();
  // Ignore silly narrowing warnings here.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
  // Note that the glutils transform is in COLUMN MAJOR order to match OpenGL's
  // convention.
  return glutils::transform{
      r[0][0], r[1][0], r[2][0], 0, r[0][1], r[1][1], r[2][1], 0,
      r[0][2], r[1][2], r[2][2], 0, t[0],    t[1],    t[2],    1};
#pragma GCC diagnostic pop
}

inline glutils::vector3f ToGLUtils(const Vector3d& _v) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
  return {_v[0], _v[1], _v[2]};
#pragma GCC diagnostic pop
}

glutils::color StringToColor(const std::string& _color);

/*----------------------------------------------------------------------------*/

#endif
