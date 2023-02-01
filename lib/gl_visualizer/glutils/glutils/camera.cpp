#include "camera.h"

#ifdef __APPLE__
  #include <OpenGL/glu.h>
#else
  #include <GL/glu.h>
#endif

#include "nonstd/runtime.h"

namespace glutils {

  /*------------------------------ Construction ------------------------------*/

  camera::
  camera(const vector3f& _pos, const vector3f& _at, const vector3f& _up)
  {
    position(_pos, _at, _up);
  }

  /*--------------------------- Rendering Function ---------------------------*/

  void
  camera::
  apply_view() const noexcept
  {
    gluLookAt(m_pos[0], m_pos[1], m_pos[2],
               m_at[0],  m_at[1],  m_at[2],
                m_y[0],   m_y[1],   m_y[2]);
  }

  /*--------------------------- Projection Functions -------------------------*/

  vector3f
  camera::
  world_to_window(const vector3f& _world) const noexcept
  {
    // Get GL context
    double model_view[16], projection[16];
    int view_port[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, view_port);

    // Project to window
    vector3d window;
    gluProject( _world[0],  _world[1],  _world[2],
               model_view, projection,  view_port,
               &window[0], &window[1], &window[2]);
    return static_cast<vector3f>(window);
  }


  vector3f
  camera::
  window_to_world(double _windowX, double _windowY, const vector3f& _p,
      const vector3f& _n) const noexcept
  {
    // Get GL context
    double model_view[16], projection[16];
    int view_port[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, view_port);

    // Shoot a ray from (_windowX, _windowY) along the screen-in direction
    vector3d source, target;
    gluUnProject(  _windowX,   _windowY,          0,
                 model_view, projection,  view_port,
                 &source[0], &source[1], &source[2]);
    gluUnProject(  _windowX,   _windowY,          1,
                 model_view, projection,  view_port,
                 &target[0], &target[1], &target[2]);
    vector3f ray = static_cast<vector3f>((target - source).normalize());

    // Ensure that the ray isn't perpendicular to the reference normal
    vector3f norm = _n.hat();
    double dot = ray * norm;
    if(fabs(dot) < std::numeric_limits<double>::epsilon()) {
      std::cerr << "Error: camera could not project perpendicular to reference "
                << "plane!" << std::endl;
      return vector3f();
    }

    // Return the intersection of ray with the reference plane.
    vector3f s = static_cast<vector3f>(source);
    return s + ray * ((_p - s) * norm / dot);
  }

  /*------------------------------- Controls ---------------------------------*/

  void
  camera::
  position(const vector3f& _pos) noexcept
  {
    m_pos = _pos;
    validate();
  }


  void
  camera::
  position(const vector3f& _pos, const vector3f& _at) noexcept
  {
    m_pos = _pos;
    m_at  = _at;
    validate();
  }


  void
  camera::
  position(const vector3f& _pos, const vector3f& _at, const vector3f& _up)
      noexcept
  {
    m_pos = _pos;
    m_at  = _at;
    m_y   = _up;
    validate();
  }


  void
  camera::
  target(const vector3f& _at) noexcept
  {
    m_at = _at;
    validate();
  }


  void
  camera::
  target(const vector3f& _at, const vector3f& _up) noexcept
  {
    m_at = _at;
    m_y  = _up;
    validate();
  }


  void
  camera::
  orient(const vector3f& _up) noexcept
  {
    m_y = _up;
    validate();
  }


  void
  camera::
  incr(const vector3f& _pos, const vector3f& _at) noexcept
  {
    m_pos += _pos;
    m_at += _at;
    validate();
  }


  void
  camera::
  translate(const vector3f& _delta) noexcept
  {
    incr(_delta, _delta);
  }


  void
  camera::
  zoom(const GLfloat _deltaZ) noexcept
  {
    vector3f delta = dir() * _deltaZ;
    translate(delta);
  }


  void
  camera::
  pan(const GLfloat _deltaX, const GLfloat _deltaY) noexcept
  {
    vector3f delta = (m_x * _deltaX) + (m_y * _deltaY);
    translate(delta);
  }


  void
  camera::
  rotate(const GLfloat _radians, const vector3f& _axis) noexcept
  {
    rotate(_radians, _axis, m_pos);
  }


  void
  camera::
  rotate(const GLfloat _radians, const vector3f& _axis, const vector3f& _point)
      noexcept
  {
    if(nonstd::approx(_radians, GLfloat(0))) return;
    vector3f pos_arm = m_pos - _point;
    vector3f at_arm = m_at - _point;
    pos_arm.rotate(_axis, _radians);
    at_arm.rotate(_axis, _radians);
    position(pos_arm + _point, at_arm + _point);
  }

  /*-------------------------------- Helpers ---------------------------------*/

  void
  camera::
  validate() noexcept
  {
    /// \ref m_z is guaranteed to remain in exactly the same direction, while
    /// \ref m_x and \ref m_y may be adjusted to restore orthogonality.
    /// \bug if \ref m_z becomes the 0 vector, this function breaks down. need a
    ///      check/recovery for this condition
    m_z = (m_pos - m_at).hat();
    m_x = (m_y % m_z).hat();
    m_y = (m_z % m_x).hat();
  }

  /*--------------------------------------------------------------------------*/

}
