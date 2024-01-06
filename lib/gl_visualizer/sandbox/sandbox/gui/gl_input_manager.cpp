#include "gl_input_manager.h"

#include "gl_widget.h"

#include "nonstd/numerics.h"

#include "glutils/camera.h"
#include "glutils/draw.h"


/*------------------------------ Button State --------------------------------*/

void
gl_input_manager::
button_state::
reset() noexcept
{
  drag = 0;
  click.setX(0);
  click.setY(0);
}

/*------------------------------ Construction --------------------------------*/

gl_input_manager::
gl_input_manager(gl_widget* const _gl) :
    QWidget(_gl), m_gl(_gl)
{
  // Mouse selection
  connect(this, SIGNAL(mouse_selection(QPoint)),
          m_gl,   SLOT(select(QPoint)));
  connect(this, SIGNAL(mouse_selection(QPoint, QPoint)),
          m_gl,   SLOT(select(QPoint, QPoint)));

  // Mouse hover
  connect(this, SIGNAL(mouse_hover(QPoint)),
          m_gl,   SLOT(hover(QPoint)));
  connect(this, SIGNAL(mouse_hover(QPoint, QPoint)),
          m_gl,   SLOT(hover(QPoint, QPoint)));

  // Camera control
  connect(this, SIGNAL(pan_camera(double, double)),
          m_gl, SLOT(pan_camera(const double, const double)));
  connect(this, SIGNAL(rotate_camera(double, double)),
          m_gl, SLOT(rotate_camera(const double, const double)));
  connect(this, SIGNAL(orbit_camera(double, double)),
          m_gl, SLOT(orbit_camera(const double, const double)));
  connect(this, SIGNAL(zoom_camera(double)),
          m_gl, SLOT(zoom_camera(const double)));
}


gl_input_manager::
~gl_input_manager() = default;

/*------------------------------ Input Events --------------------------------*/

void
gl_input_manager::
mousePressEvent(QMouseEvent* _e)
{
  switch(_e->button()) {
    case Qt::LeftButton:
      click_left(_e, true);
      break;
    case Qt::MiddleButton:
      click_middle(_e, true);
      break;
    case Qt::RightButton:
      click_right(_e, true);
      break;
    default:;
  }
}


void
gl_input_manager::
mouseReleaseEvent(QMouseEvent* _e)
{
  switch(_e->button()) {
    case Qt::LeftButton:
      click_left(_e, false);
      break;
    case Qt::MiddleButton:
      click_middle(_e, false);
      break;
    case Qt::RightButton:
      click_right(_e, false);
      break;
    default:;
  }
}


void
gl_input_manager::
mouseMoveEvent(QMouseEvent* _e)
{
  if(!_e->buttons()) {
    // If no buttons were pressed during the movement, just update mouse position.
    emit mouse_hover(_e->pos());
  }
  else {
    // There is at least one button held down. Determine the displacement.
    const QPoint delta = _e->pos() - m_hover;

    // Process the drag even for each button.
    if(_e->buttons() & Qt::LeftButton)
      drag_left(_e, delta);
    if(_e->buttons() & Qt::MiddleButton)
      drag_middle(_e, delta);
    if(_e->buttons() & Qt::RightButton)
      drag_right(_e, delta);
  }

  // Update last hover position.
  m_hover = _e->pos();
}


void
gl_input_manager::
mouseDoubleClickEvent(QMouseEvent*)
{ }


void
gl_input_manager::
wheelEvent(QWheelEvent* _e)
{
  // Do nothing if the middle button is pressed down.
  if(_e->buttons() & Qt::MiddleButton)
    return;

  // Scale wheel distance quadratically to make vigorous scrolling zoom faster.
  double wheelDist = _e->angleDelta().y() / 60.;
  wheelDist *= wheelDist * nonstd::sign(wheelDist) * m_sensitivity;

  emit zoom_camera(wheelDist);
}


void
gl_input_manager::
keyPressEvent(QKeyEvent* _e)
{
  _e->ignore();
}


void
gl_input_manager::
keyReleaseEvent(QKeyEvent* _e)
{
  _e->ignore();
}


void
gl_input_manager::
tabletEvent(QTabletEvent*)
{ }

/*------------------------------ Mouse Actors --------------------------------*/

void
gl_input_manager::
click_left(QMouseEvent* _e, const bool _press)
{
  if(_press) {
    // Handle click.
    m_left.click = _e->pos();
  }
  else {
    // Handle unclick.
    // If there is an active middle or right drag, do nothing.
    if(m_right.drag || m_middle.drag)
      ;
    // If there is no active left drag, this is a point selection.
    else if(!m_left.drag)
      emit mouse_selection(_e->pos());
    // Otherwise, this is an area selection.
    else
      emit mouse_selection(m_left.click, _e->pos());

    m_left.reset();
  }
}


void
gl_input_manager::
click_middle(QMouseEvent* _e, const bool _press)
{
  if(_press) {
    // Handle click.
    m_middle.click = _e->pos();
  }
  else {
    // Handle unclick.
    m_middle.reset();
  }
}


void
gl_input_manager::
click_right(QMouseEvent* _e, const bool _press)
{
  if(_press) {
    // Handle click.
    m_right.click = _e->pos();
  }
  else {
    // Handle unclick.
    m_right.reset();
  }
}


void
gl_input_manager::
drag_left(QMouseEvent* _e, const QPoint& _delta)
{
  // Add the drag distance.
  m_left.drag += _delta.manhattanLength();

  // If we have not exceeded the deadzone, do nothing.
  if(m_left.drag < m_deadzone)
    return;

  // If the right mouse button is also down, do nothing. Otherwise, emit hover
  // signal for the box from the click point to here.
  if(!(_e->buttons() & Qt::RightButton))
    emit mouse_hover(m_left.click, _e->pos());
}


void
gl_input_manager::
drag_middle(QMouseEvent* /*_e*/, const QPoint& _delta)
{
  // Add the drag distance.
  m_middle.drag += _delta.manhattanLength();

  // If we have not exceeded the deadzone, do nothing.
  if(m_middle.drag < m_deadzone)
    return;

  // Pan the camera view.
  emit pan_camera(_delta.x() * m_sensitivity, -_delta.y() * m_sensitivity);
}


void
gl_input_manager::
drag_right(QMouseEvent* _e, const QPoint& _delta)
{
  // Add the drag distance.
  m_right.drag += _delta.manhattanLength();

  // If we have not exceeded the deadzone, do nothing.
  if(m_right.drag < m_deadzone)
    return;

  /// @TODO Currently we use pi/180 as an additional scaling factor compared
  ///       with translation. The sensitivities should be made separate and
  ///       user-configurable.
  const double x = -_delta.x() * m_sensitivity * glutils::RadPerDeg,
               y = -_delta.y() * m_sensitivity * glutils::RadPerDeg;

  // If the left mouse button is also held down, rotate the camera about itself.
  // Otherwise, orbit the camera about the origin.
  if(_e->buttons() & Qt::LeftButton)
    emit rotate_camera(x, y);
  else
    emit orbit_camera(x, y);
}

/*----------------------------------------------------------------------------*/
