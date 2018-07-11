#include "DrawableBody.h"

#include "glutils/color.h"

#include "Geometry/Bodies/Body.h"
#include "Visualization/DrawableMultiBody.h"


/*------------------------------ Construction --------------------------------*/

DrawableBody::
DrawableBody(DrawableMultiBody* const _parent, Body* const _body)
  : m_parent(_parent), m_body(_body) { }

/*------------------------------- Body Support -------------------------------*/

Body*
DrawableBody::
GetBody() const noexcept {
  return m_body;
}

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
  auto color = m_body->GetColor();
  glColor4fv(color);

  /// @todo See if we can support proper rendering of transparent objects
  ///       without needing to z-sort every drawable.

  glBegin(GL_TRIANGLES);
  const auto& polyhedron = m_body->GetPolyhedron();
  for(const auto& polygon : polyhedron.GetPolygonList())
    for(size_t i = 0; i < 3; ++i) {
      glNormal3dv(static_cast<const GLdouble*>(polygon.GetNormal()));
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
    }
  glEnd();
}


void
DrawableBody::
build_select() {
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
  glDisable(GL_LIGHTING);
  glColor4fv(glutils::color::yellow);
  glLineWidth(4);

  const auto& polyhedron = m_body->GetPolyhedron();
  for(const auto& polygon : polyhedron.GetPolygonList())
  {
    glBegin(GL_LINE_LOOP);
    for(size_t i = 0; i < 3; ++i)
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
    glEnd();
  }
  glEnable(GL_LIGHTING);
}


void
DrawableBody::
build_highlighted() {
  // None for now.
}

/*----------------------------------------------------------------------------*/
