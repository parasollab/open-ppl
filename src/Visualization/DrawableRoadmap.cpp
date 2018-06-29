#include "DrawableRoadmap.h"

DrawableRoadmap::
~DrawableRoadmap() = default;

void
DrawableRoadmap::
draw() {
  
  // sets the color
  glColor4fv(m_color);
  
  // sets the size of each point (radius in pixels?)
  glPointSize(8.0);

  // says to draw a point primative
  glBegin(GL_POINTS);
  for(auto& v : m_cfgs)
    glVertex3fv(v.begin());
  glEnd();
  
  // sets the width of the line
  glLineWidth(1.0);
 
  for(const auto& edge : m_edges) {

    // after the first vertex, for each new vertice
    // added a new line will be drawn.
    // Lets say n vertices were drawn, n-1 vertices will
    // rendered. Start from 0 to 1, 1 to 2, 2 to 3 and so on.
    glBegin(GL_LINE_STRIP);
    for(const auto& inter : edge)
      glVertex3fv(inter.begin());
    glEnd();
  }
}

void
DrawableRoadmap::
draw_select() {}

void
DrawableRoadmap::
draw_selected() {}

void
DrawableRoadmap::
draw_highlighted() {}
