#ifndef GLUTILS_DRAW_H_
#define GLUTILS_DRAW_H_

#include <cmath>
#include <cstddef>

#include "glutils/gltraits.h"


namespace glutils {

  //////////////////////////////////////////////////////////////////////////////
  /// Simple routines for drawing primitive shapes in OpenGL.
  //////////////////////////////////////////////////////////////////////////////
  namespace draw {

    ///@name 2D Primitives
    ///@{

    /// Draw a pseudo-circle with _segments sides on the x-y plane.
    /// @param[in] _radius The circle radius.
    /// @param[in] _segments The number of segments to use.
    void
    circle(const GLfloat _radius, const size_t _segments = 16);


    /// Draw a wire pseudo-circle.
    /// @param[in] _radius The circle radius.
    /// @param[in] _segments The number of segments to use.
    void
    circle_frame(const GLfloat _radius, const size_t _segments = 16);

    ///@}
    ///@name 3D Primitives
    ///@{

    /// Draw a sphere centered at the origin with axis along the z direction.
    /// @param[in] _radius The sphere radius.
    /// @param[in] _segments The number of segments to use.
    void
    sphere(const GLfloat _radius, const size_t _segments = 16);


    /// Draw a wire sphere centered at the origin with axis along the z direction.
    /// @param[in] _radius The sphere radius.
    /// @param[in] _segments The number of segments to use.
    void
    sphere_frame(const GLfloat _radius, const size_t _segments = 16);


    /// Draw a box centered at the origin.
    /// @param[in] _lenX The full length in the x direction.
    /// @param[in] _lenY The full length in the y direction.
    /// @param[in] _lenZ The full length in the z direction.
    void
    box(const GLfloat _lenX, const GLfloat _lenY, const GLfloat _lenZ);


    /// Draw a wire box centered at the origin.
    /// @param[in] _lenX The full length in the x direction.
    /// @param[in] _lenY The full length in the y direction.
    /// @param[in] _lenZ The full length in the z direction.
    void
    box_frame(const GLfloat _lenX, const GLfloat _lenY, const GLfloat _lenZ);


    /// Draw a cone with the base centered at the origin and tip pointed away
    /// from the camera.
    /// @param[in] _radius The radius of the base.
    /// @param[in] _height The height of the cone.
    /// @param[in] _segments The number of segments to use for the sides.
    void
    cone(const GLfloat _radius, const GLfloat _height,
        const size_t _segments = 16);


    /// Draw a wire cone with the base centered at the origin and tip pointed away
    /// from the camera.
    /// @param[in] _radius The radius of the base.
    /// @param[in] _height The height of the cone.
    /// @param[in] _segments The number of segments to use for the sides.
    void
    cone_frame(const GLfloat _radius, const GLfloat _height,
        const size_t _segments = 16);


    /// Draw a cylinder centered at the origin and oriented along the z-axis.
    /// @param[in] _radius The cylinder radius.
    /// @param[in] _length The length perpendicular to the radius.
    /// @param[in] _segments The number of segments to use for the side wall.
    void
    cylinder(const GLfloat _radius, const GLfloat _length,
        const size_t _segments = 16);


    /// Draw a wire cylinder.
    /// @param[in] _radius The cylinder radius.
    /// @param[in] _length The length perpendicular to the radius.
    /// @param[in] _segments The number of segments to use for the side wall.
    void
    cylinder_frame(const GLfloat _radius, const GLfloat _length,
        const size_t _segments = 16);

    ///@}

  }

}

#endif
