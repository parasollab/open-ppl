#include "DrawableBoundary.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/GMSPolyhedron.h"

DrawableBoundary::
DrawableBoundary(const Boundary* _boundary, const glutils::color& _color, bool _solid) :
  glutils::drawable_call_list(),
  m_boundary(_boundary), m_color(_color), m_solid(_solid) {}

void
DrawableBoundary::
build() {
  const bool transparent = m_color[3] != 1.;

  glColor4fv(m_color);
  glLineWidth(1.0);

  if(!m_solid)
    glDisable(GL_LIGHTING);
  else
    glDisable(GL_CULL_FACE);
  if(transparent) {
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
  }

  auto type = m_solid ? GL_TRIANGLES : GL_LINE_LOOP;
  const auto& polyhedron = m_boundary->MakePolyhedron();
  for(const auto& polygon : polyhedron.GetPolygonList())
  {
    glBegin(type);
    for(size_t i = 0; i < 3; ++i)
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
    glEnd();
  }

  if(!m_solid)
    glEnable(GL_LIGHTING);
  else
    glEnable(GL_CULL_FACE);
  if(transparent) {
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }
}

void
DrawableBoundary::
build_select() {
}
