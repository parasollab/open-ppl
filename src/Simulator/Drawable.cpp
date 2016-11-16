#include "Drawable.h"

#include "glutils/color.h"
#include "glutils/obj_file.h"
#include "glutils/triangulated_model.h"

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"

/*------------------------------- Construction -------------------------------*/

Drawable::
Drawable(const std::string& _filename) : glutils::drawable_call_list() {
  glutils::obj_file in(_filename);

  // Make a temporary model from the obj file. We will use it to build the call
  // lists and then throw it away.
  m_model = new glutils::triangulated_model;
  in >> *m_model;

  std::cout << "Reading model from " << _filename
            << "\n\tvertices: " << m_model->num_points()
            << "\n\tfacets: " << m_model->num_facets() << std::endl;
}


//Drawable::
//Drawable(glutils::triangulated_model* _t) {
//  m_model = new glutils::triangulated_model(_t);
//}


Drawable::
~Drawable() {
  // Delete the temporary model in case we never build its call lists.
  delete m_model;
}

/*----------------------- drawable_call_list Overrides -----------------------*/

void
Drawable::
build() {
  // Draw walls.
  glColor3fv(glutils::color::blue);

  glBegin(GL_TRIANGLES);
  for(auto iter = m_model->facets_begin(); iter != m_model->facets_end(); ++iter)
  {
    const auto& facet = *iter;

    for(const auto& index : facet)
      glVertex3fv(static_cast<const GLfloat*>(facet.get_point(index)));
  }
  glEnd();

  std::cout << "Building model:"
            << "\n\tvertices: " << m_model->num_points()
            << "\n\tfacets: " << m_model->num_facets() << std::endl;
}


void
Drawable::
build_selected() {
  glColor3fv(glutils::color::yellow);

  for(auto iter = m_model->facets_begin(); iter != m_model->facets_end(); ++iter)
  {
    const auto& facet = *iter;

    glBegin(GL_LINE_LOOP);
    for(const auto& index : facet)
      glVertex3fv(static_cast<const GLfloat*>(facet.get_point(index)));
    glEnd();
  }
}


void
Drawable::
build_highlighted() {
  // This is the last build instruction. We can release the memory for the model
  // now since it won't be used again.
  delete m_model;
  m_model = nullptr;
}

/*----------------------------------------------------------------------------*/
