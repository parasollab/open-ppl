#include "DrawableBoundary.h"

#include "Geometry/Boundaries/Boundary.h"


DrawableBoundary::
DrawableBoundary(const Boundary* _boundary, const glutils::color& _color, bool _wire) :
  DrawablePolyhedron(m_polyhedron, _color, _wire),
  m_polyhedron(_boundary->MakePolyhedron()) {}


void
DrawableBoundary::
build() {

  const bool transparent = m_color[3] != 1.0;

  if(!this->m_wire)
    glDisable(GL_CULL_FACE);
  if(transparent) {
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
  }

  DrawablePolyhedron::build();

  if(!this->m_wire)
    glEnable(GL_CULL_FACE);
  if(transparent) {
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }
}
