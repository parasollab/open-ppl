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
  glColor4fv(m_color);
  
  auto type = m_solid ? GL_TRIANGLES : GL_LINE_LOOP;
  const auto& polyhedron = m_boundary->MakePolyhedron();
  for(const auto& polygon : polyhedron.GetPolygonList())
  {
    glBegin(type);
    for(size_t i = 0; i < 3; ++i)
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
    glEnd();
  }
}

void
DrawableBoundary::
build_select() {
}

