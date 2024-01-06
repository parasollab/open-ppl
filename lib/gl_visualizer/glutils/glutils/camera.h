#ifndef GLUTILS_CAMERA_H_
#define GLUTILS_CAMERA_H_

#include "glutils/gltraits.h"

namespace glutils {

  //////////////////////////////////////////////////////////////////////////////
  /// A gluLookAt camera.
  ///
  /// The camera is located at @ref m_pos and looks at @ref m_at. The vectors
  /// @ref m_x, @ref m_y, and @ref m_z describe the viewing orientation in global
  /// coordinates.
  //////////////////////////////////////////////////////////////////////////////
  class camera {

    ///@name Perspective
    ///@{

    vector3f m_pos;    ///< The camera's position in world coordinates.
    vector3f m_at;     ///< The camera's viewing target in world coordinates.
    vector3f m_x;      ///< The screen-right direction in world coordinates.
    vector3f m_y;      ///< The screen-up direction in world coordinates.
    vector3f m_z;      ///< The screen-out direction in world coordinates.

    ///@}

    public:

      ///@name Construction
      ///@{

      /// The default configuration places the camera at the origin looking down
      /// the -z axis with +y pointing up.
      camera(const vector3f& _pos = {0, 0, 0},
             const vector3f& _at  = {0, 0,-1},
             const vector3f& _up  = {0, 1, 0});

      ///@}
      ///@name Rendering Function
      ///@{

      /// Apply the current perspective to the GL scene.
      /// @note Should be called before other rendering calls.
      void apply_view() const noexcept;

      ///@}
      ///@name Projection Functions
      ///@{
      /// Translate between window and world coordinates.

      /// Project a point from world coordinates to window coordinates.
      /// @param[in] _world   The point in world coordinates.
      /// @return             The same point in window coordinates.
      vector3f world_to_window(const vector3f& _world) const noexcept;

      /// Project a point from window coordinates to world coordinates.
      ///
      /// The projected point is the intersection of a ray shot from the window
      /// coordinates (_x, _y) along the screen-in direction and the plane
      /// defined by a reference point _p and normal _n.
      /// @param[in] _x The input window-x coordinate.
      /// @param[in] _y The input window-y coordinate.
      /// @param[in] _p The reference point in world coordinates. Default is
      ///               the origin.
      /// @param[in] _n The reference normal in world coordinates. Default is the
      ///               screen-out direction (0, 0, 1).
      /// @return       The projected point in world coordinates.
      vector3f window_to_world(double _x, double _y,
          const vector3f& _p = vector3f(), const vector3f& _n = {0, 0, 1})
          const noexcept;

      ///@}
      ///@name Accessors
      ///@{

      /// Get the camera position.
      const vector3f& pos() const noexcept {return m_pos;}

      /// Get the viewing position.
      const vector3f& at() const noexcept {return m_at;}

      /// Get the screen-X direction.
      const vector3f& x() const noexcept {return m_x;}

      /// Get the screen-Y direction.
      const vector3f& y() const noexcept {return m_y;}

      /// Get the screen-Z direction.
      const vector3f& z() const noexcept {return m_z;}

      /// Get the viewing direction.
      vector3f dir() const noexcept {return -m_z;}

      ///@}
      ///@name Controls
      ///@{

      /// Set the position only.
      /// @param[in] _pos The new camera position.
      void position(const vector3f& _pos) noexcept;

      /// Set the position and viewing point.
      /// @param[in] _pos The new camera position.
      /// @param[in] _at The new camera viewing target.
      void position(const vector3f& _pos, const vector3f& _at) noexcept;

      /// Set the position, viewing point, and orientation.
      /// @param[in] _pos The new camera position.
      /// @param[in] _at  The new camera viewing target
      /// @param[in] _up  The new camera up direction.
      void position(const vector3f& _pos, const vector3f& _at,
          const vector3f& _up) noexcept;

      /// Set the viewing point only.
      /// @param[in] _at  The new camera viewing target.
      void target(const vector3f& _at) noexcept;

      /// Set the viewing point and orientation.
      /// @param[in] _at  The new camera viewing target.
      /// @param[in] _up  The new camera up direction.
      void target(const vector3f& _at, const vector3f& _up) noexcept;

      /// Set the orientation.
      /// @param[in] _up  The new camera up direction.
      void orient(const vector3f& _up) noexcept;

      /// Move the camera position and the viewing point.
      /// @param[in] _pos The translation vector for the camera position.
      /// @param[in] _at  The translation vector for the viewing point.
      void incr(const vector3f& _pos, const vector3f& _at  = {0, 0, 0}) noexcept;

      /// Move the camera position and viewing point together.
      /// @param[in] _delta The translation vector in world coordinates.
      void translate(const vector3f& _delta) noexcept;

      /// Rotate the camera position and viewing point counter-clockwise about a
      /// reference axis.
      /// @param[in] _radians The magnitude of rotation in radians.
      /// @param[in] _axis The reference axis.
      void rotate(const GLfloat _radians, const vector3f& _axis) noexcept;

      /// Rotate the camera position and viewing point counter-clockwise about a
      /// reference axis and point.
      /// @param[in] _radians The magnitude of rotation in radians.
      /// @param[in] _axis The reference axis.
      /// @param[in] _point The reference point.
      void rotate(const GLfloat _radians,
                  const vector3f& _axis,
                  const vector3f& _point) noexcept;

      /// Translate the camera position in the screen Z direction.
      /// @param[in] _deltaZ The \c Z translation distance in world coordinates.
      void zoom(const GLfloat _deltaZ) noexcept;

      /// Translate the camera position in the screen X and Y directions.
      /// @param[in] _deltaX The \c X translation distance in world coordinates.
      /// @param[in] _deltaY The \c Y translation distance in world coordinates.
      void pan(const GLfloat _deltaX, const GLfloat _deltaY) noexcept;

      ///@}

    private:

      ///@name Helpers
      ///@{

      void validate() noexcept; ///< Ensure that the orientation is orthonormal.

      ///@}
  };

}

#endif
