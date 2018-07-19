#include "DrawablePolyhedron.h"

DrawablePolyhedron::
DrawablePolyhedron(const GMSPolyhedron& _polyhedron,
    const glutils::color& _color, bool _wire) :
    glutils::drawable_display_list(),
    m_polyhedron(_polyhedron),
    m_color(_color),
    m_wire(_wire) {}

void
DrawablePolyhedron::
ToggleRenderMode() {
  m_wire = !m_wire;
  this->uninitialize();
}

void
DrawablePolyhedron::
build() {
  glColor4fv(m_color);

  if(m_wire) {
    const bool lighting = glIsEnabled(GL_LIGHTING);
    // if lighting is on, then turn it off
    if(lighting)
      glDisable(GL_LIGHTING);
    glLineWidth(1.0);
    const auto triangleList = BuildAdjacencyMap();

    const auto& points = m_polyhedron.GetVertexList();
    glBegin(GL_LINES);
    for(auto& elem : triangleList) {
      glVertex3dv(static_cast<const GLdouble*>(points[elem.first]));
      glVertex3dv(static_cast<const GLdouble*>(points[elem.second]));
    }
    glEnd();
    if(lighting)
      glEnable(GL_LIGHTING);
  }
  else {
    glDisable(GL_LIGHTING);
    for(const auto& polygon : m_polyhedron.GetPolygonList()) {
      glBegin(GL_TRIANGLES);
      for(size_t i = 0; i < 3; ++i) {
        glNormal3dv(static_cast<const GLdouble*>(polygon.GetNormal()));
        glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
      }
      glEnd();
    }

    // if it was on then turn it back on
     glEnable(GL_LIGHTING);

  }

}

void
DrawablePolyhedron::
build_select() {
  glBegin(GL_TRIANGLES);
  for(const auto& polygon : m_polyhedron.GetPolygonList()) {
    for(size_t i = 0; i < 3; ++i)
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
  }
  glEnd();
}

void
DrawablePolyhedron::
build_selected() {
  const auto triangleList = BuildAdjacencyMap();


  glDisable(GL_LIGHTING);
  glColor4fv(glutils::color::yellow);
  glLineWidth(4);

  const auto& points = m_polyhedron.GetVertexList();
  glBegin(GL_LINES);
  for(auto& elem : triangleList) {
    glVertex3dv(static_cast<const GLdouble*>(points[elem.first]));
    glVertex3dv(static_cast<const GLdouble*>(points[elem.second]));
  }

  glEnd();

  glEnable(GL_LIGHTING);
}

std::set<std::pair<size_t, size_t>>
DrawablePolyhedron::
BuildAdjacencyMap() {
  std::set<std::pair<size_t, size_t>> triangleList;
  const auto& triangles = m_polyhedron.GetPolygonList();
  for(auto triangle = triangles.begin(); triangle != triangles.end(); ++triangle) {
    const auto& n1 = triangle->GetNormal();
    for(auto iter = triangle + 1; iter < triangles.end(); ++iter) {
      const auto& n2 = iter->GetNormal();

      if(n1 != n2) {
        auto commonEdge = triangle->CommonEdge(*iter);
        if(commonEdge.first != -1 and commonEdge.second != -1)
          triangleList.emplace((size_t) commonEdge.first, (size_t) commonEdge.second);
      }
    }
  }
  return triangleList;
}
