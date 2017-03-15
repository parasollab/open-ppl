#include "DrawableBody.h"

#include "glutils/color.h"

#include "Geometry/Bodies/Body.h"
#include "Visualization/DrawableMultiBody.h"


/*------------------------------ Construction --------------------------------*/

DrawableBody::
DrawableBody(DrawableMultiBody* const _parent, const Body* const _body)
  : m_parent(_parent), m_body(_body) { }

/*--------------------------- drawable Overrides -----------------------------*/

void
DrawableBody::
select() noexcept {
  m_parent->select();
}


void
DrawableBody::
deselect() noexcept {
  m_parent->deselect();
}


void
DrawableBody::
highlight() noexcept {
  m_parent->highlight();
}


void
DrawableBody::
unhighlight() noexcept {
  m_parent->unhighlight();
}

/*----------------------- drawable_call_list Overrides -----------------------*/

void
DrawableBody::
build() {
  /// @TODO Set color from pmpl body.
  glColor3fv(glutils::color::blue);
  build_select();
}


void
DrawableBody::
build_select() {
  glLineWidth(1);
  glBegin(GL_TRIANGLES);
  const auto& polyhedron = m_body->GetPolyhedron();
  for(const auto& polygon : polyhedron.GetPolygonList())
    for(size_t i = 0; i < 3; ++i)
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
  glEnd();
}


void
DrawableBody::
build_selected() {
  glColor3fv(glutils::color::yellow);
  glLineWidth(4);

  const auto& polyhedron = m_body->GetPolyhedron();
  for(const auto& polygon : polyhedron.GetPolygonList())
  {
    glBegin(GL_LINE_LOOP);
    for(size_t i = 0; i < 3; ++i)
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
    glEnd();
  }
}


void
DrawableBody::
build_highlighted() {
  // None for now.
}

/*----------------------------------------------------------------------------*/
