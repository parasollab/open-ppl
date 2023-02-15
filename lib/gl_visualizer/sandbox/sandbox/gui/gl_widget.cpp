#include "gl_widget.h"

#include <array>

#ifdef __APPLE__
  #include <OpenGL/glu.h>
#else
  #include <GL/glu.h>
#endif
#include <QtGui>

#include "glutils/camera.h"
#include "glutils/color.h"

#include "sandbox/base_visualization.h"

#include "gl_input_manager.h"


using namespace std;
using namespace glutils;

/*------------------------------- Construction -------------------------------*/

gl_widget::
gl_widget(QWidget* _parent) :
    QOpenGLWidget(_parent)
{
  // Set widget size and initialize GL.
  setMinimumSize(800, 600);
  makeCurrent();
  initializeGL();

  // Create camera and set default position.
  m_camera = new glutils::camera();
  m_camera->position(glutils::vector3f{0,0,10}, glutils::vector3f{0,0,0},
      glutils::vector3f{0,1,0});

  // Create controller manager.
  m_input_manager = new gl_input_manager(this);

  // Set up rendering clock.
  m_clock = new QTimer(this);
  m_clock->setInterval(33);
  connect(m_clock, SIGNAL(timeout()),
          this,      SLOT(updateGL()));

  // Use the mouse and refrain from propagating unhandled events to (_parent).
  this->setMouseTracking(true);
  this->setAttribute(Qt::WA_NoMousePropagation);

  // Set focus policy to accept key press events.
  this->setFocusPolicy(Qt::StrongFocus);
}


gl_widget::
~gl_widget()
{
  m_clock->stop();
  delete m_camera;
}

/*------------------------------ Accessors -----------------------------------*/

glutils::camera*
gl_widget::
camera() const
{
  return m_camera;
}


base_visualization*
gl_widget::
visualization() const
{
  return m_visualization;
}


void
gl_widget::
visualization(base_visualization* const _s)
{
  m_visualization = _s;
}

/*--------------------------- Simulation Control -----------------------------*/

void
gl_widget::
start()
{
  emit status_message(QString("Starting visualization..."), 2000);

  m_visualization->start();
  m_clock->start();
}


void
gl_widget::
reset()
{
  emit status_message(QString("Stopping visualization..."), 2000);

  m_clock->stop();
  m_visualization->reset();
  m_camera->position(glutils::vector3f{0,0,10}, glutils::vector3f{0,0,0},
      glutils::vector3f{0,1,0});
  update();
}

/*------------------------------ Camera Control ------------------------------*/

void
gl_widget::
pan_camera(const double _x, const double _y) {
  m_camera->pan(_x, _y);
}


void
gl_widget::
zoom_camera(const double _z) {
  m_camera->zoom(_z);
}


void
gl_widget::
rotate_camera(const double _x, const double _y) {
  m_camera->rotate(_x, m_camera->y());
  m_camera->rotate(_y, m_camera->x());
}


void
gl_widget::
orbit_camera(const double _x, const double _y) {
  m_camera->rotate(_x, m_camera->y(), glutils::vector3f());
  m_camera->rotate(_y, m_camera->x(), glutils::vector3f());
}

/*-------------------------------- Selection ---------------------------------*/

/// Make a box from two QPoints. Returns x-center, y-center, width, height.
inline
std::array<size_t, 4>
make_box(const QPoint& _p1, const QPoint& _p2)
{
  const size_t x = static_cast<size_t>((_p1.x() + _p2.x()) / 2);
  const size_t y = static_cast<size_t>((_p1.y() + _p2.y()) / 2);
  const size_t w = static_cast<size_t>(std::abs(_p1.x() - _p2.x()));
  const size_t h = static_cast<size_t>(std::abs(_p1.y() - _p2.y()));
  return std::array<size_t, 4>{x, y, w, h};
}


void
gl_widget::
select(const size_t _x, const size_t _y, const size_t _w, const size_t _h)
{
  // Qt convention is backward from OpenGL. Invert y coordinate to fix.
  m_visualization->render_select(_x, QWidget::height() - _y, _w, _h);
}


void
gl_widget::
select(QPoint _p1, QPoint _p2)
{
  auto b = make_box(_p1, _p2);
  select(b[0], b[1], b[2], b[3]);
}


void
gl_widget::
select(QPoint _p)
{
  select(_p.rx(), _p.ry(), 1, 1);
}


void
gl_widget::
hover(const size_t _x, const size_t _y, const size_t _w, const size_t _h)
{
  // Qt convention is backward from OpenGL. Invert y coordinate to fix.
  m_visualization->render_hover(_x, QWidget::height() - _y, _w, _h);
}


void
gl_widget::
hover(QPoint _p1, QPoint _p2)
{
  auto b = make_box(_p1, _p2);
  hover(b[0], b[1], b[2], b[3]);
}


void
gl_widget::
hover(QPoint _p)
{
  hover(_p.rx(), _p.ry(), 1, 1);
}

/*------------------------------- GL Functions -------------------------------*/

void
gl_widget::
initializeGL()
{
  // Set clear color to white.
  glClearColor(1., 1., 1., 1.);

  // Enable material coloring with glColor.
  glEnable(GL_COLOR_MATERIAL);

  // Create some lights to show 3d objects as 3d.
  glEnable(GL_LIGHTING);
  glLightfv(GL_LIGHT0, GL_AMBIENT, color::dark_grey);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, color::white);
  glLightfv(GL_LIGHT0, GL_SPECULAR, color::white);
  glEnable(GL_LIGHT0);

  // Position the lights.
  GLfloat light0Pos[] = {100., 100., 100., 1.};
  glLightfv(GL_LIGHT0, GL_POSITION, light0Pos);

  // Enable depth test.
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearDepth(1.0);

  // Specify the alpha blending function. Blending doesn't play nice with depth
  // and must be enabled separately.
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Enable point anti-aliasing.
  glEnable(GL_POINT_SMOOTH);

  // Enable smooth shading.
  glShadeModel(GL_SMOOTH);

  // Enable back-face culling to see through boundaries.
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
}


void
gl_widget::
paintGL()
{
  // Erase the previous frame.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Set view and lighting.
  glLoadIdentity();
  m_camera->apply_view();

  // Render the scene.
  if(m_visualization)
    m_visualization->render();

  // Push data.
  glFlush();
}


void
gl_widget::
resizeGL(int _w, int _h)
{
  // Set the GL window size
  glViewport(0, 0, GLint(_w), GLint(_h));

  // Set the viewing volume
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, ((GLfloat)_w/(GLfloat)_h), .1, 10000);

  // Return to model view mode
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/*------------------------------- Input Forwarding ---------------------------*/

void
gl_widget::
mousePressEvent(QMouseEvent* _e)
{
  m_input_manager->mousePressEvent(_e);
}


void
gl_widget::
mouseReleaseEvent(QMouseEvent* _e)
{
  m_input_manager->mouseReleaseEvent(_e);
}


void
gl_widget::
mouseMoveEvent(QMouseEvent* _e)
{
  m_input_manager->mouseMoveEvent(_e);
}


void
gl_widget::
mouseDoubleClickEvent(QMouseEvent* _e)
{
  m_input_manager->mouseDoubleClickEvent(_e);
}


void
gl_widget::
wheelEvent(QWheelEvent* _e)
{
  m_input_manager->wheelEvent(_e);
}


void
gl_widget::
keyPressEvent(QKeyEvent* _e)
{
  m_input_manager->keyPressEvent(_e);
}


void
gl_widget::
keyReleaseEvent(QKeyEvent* _e)
{
  m_input_manager->keyReleaseEvent(_e);
}


void
gl_widget::
tabletEvent(QTabletEvent* _e)
{
  m_input_manager->tabletEvent(_e);
}

/*----------------------------------------------------------------------------*/
