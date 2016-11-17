#include "Drawable.h"

#include "glutils/color.h"
#include "glutils/obj_file.h"
#include "glutils/triangulated_model.h"

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"

#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"

/*------------------------------- Construction -------------------------------*/

Drawable::
Drawable(const std::string& _filename) {
  // Make a temporary model from the obj file. We will use it to build the call
  // lists and then throw it away.
  m_model = new glutils::triangulated_model;
  glutils::obj_file(_filename) >> *m_model;
}


Drawable::
Drawable(MultiBody* _m) {
  // Get the multibody's obj file and use it to create a bullet body.
  /// @TODO Parse all components of the multibodies instead of just the first.
  /// @TODO Don't rely on obj file - build model directly from multibody data.
  std::string filename;

  StaticMultiBody* sbody = dynamic_cast<StaticMultiBody*>(_m);
  if(sbody)
    filename = sbody->GetFixedBody(0)->GetFilePath();
  else {
    ActiveMultiBody* abody = dynamic_cast<ActiveMultiBody*>(_m);
    filename = abody->GetFreeBody(0)->GetFilePath();
  }

  m_model = new glutils::triangulated_model;
  glutils::obj_file(filename) >> *m_model;
}


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
