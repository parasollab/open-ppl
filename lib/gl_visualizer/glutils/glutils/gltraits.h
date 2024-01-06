#ifndef GLUTILS_GLTRAITS_H_
#define GLUTILS_GLTRAITS_H_

#include <array>

#ifdef __APPLE__
  #include <OpenGL/gl.h>
#elif _WIN32
  #define NOMINMAX
  #include <windows.h>
  #include <GL/gl.h>
#else
  #include <GL/gl.h>
#endif

#include "nonstd/vector.h"


namespace glutils {

  ///@name Constants
  ///@{
  /// Precision chosen based on numeric_limits<GLfloat>::digits10.
  /// The constants PI and TWOPI can be overridden by a defining macro for them.

#ifndef PI
  static constexpr GLfloat PI    = 3.14159;
#endif
#ifndef TWOPI
  static constexpr GLfloat TWOPI = 6.28319;
#endif
  static constexpr GLfloat RadPerDeg = PI / 180;
  static constexpr GLfloat DegPerRad = 180 / PI;

  ///@}
  ///@name Vector Representations
  ///@{

  typedef nonstd::vector_type<GLfloat,  2> vector2f;
  typedef nonstd::vector_type<GLdouble, 2> vector2d;
  typedef nonstd::vector_type<GLfloat,  3> vector3f;
  typedef nonstd::vector_type<GLdouble, 3> vector3d;

  ///@}
  ///@name Transforms
  ///@{

  /// An OpenGL transform matrix in standard form (column major).
  typedef std::array<GLfloat, 16> transform;

  /// Apply an OpenGL transform matrix to the current GL stack.
  /// @param _t The transform to apply.
  void apply_transform(const transform& _t) noexcept;

  /// Generate an identity transform.
  transform identity_transform() noexcept;

  ///@}

}

/*-------------------------- Inlined Functions -------------------------------*/

inline
void
glutils::
apply_transform(const glutils::transform& _t) noexcept
{
  glMultMatrixf(_t.data());
}


inline
glutils::transform
glutils::
identity_transform() noexcept
{
  return glutils::transform{1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1};
}

/*----------------------------------------------------------------------------*/

#endif
