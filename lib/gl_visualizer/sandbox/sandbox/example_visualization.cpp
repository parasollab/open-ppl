#include "example_visualization.h"

#include "glutils/color.h"
#include "glutils/draw.h"
#include "glutils/drawable_display_list.h"
#include "nonstd/io.h"


// Example of a drawable object.
class drawable_sphere :
    public glutils::drawable_display_list
{

  virtual void build() override
  {
    glColor4f(.5, .1, .5, .5);
    glEnable(GL_BLEND);
    glDepthMask(GL_FALSE);
    glLineWidth(1);
    glutils::draw::sphere(5);
    glutils::draw::sphere_frame(5);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  virtual void build_select() override
  {
    glLineWidth(1);
    glutils::draw::sphere(5);
  }

  virtual void build_selected() override
  {
    glColor4fv(glutils::color::yellow);
    glLineWidth(4);
    glutils::draw::sphere_frame(5);
  }

  virtual void build_highlighted() override
  {
    glEnable(GL_BLEND);
    glDepthMask(GL_FALSE);
    glLineWidth(1);
    glColor4f(0, .1, .6, .4);
    glutils::draw::sphere(5.1);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  /*
  // Example for how to hide an object on selection.
  virtual void select() noexcept override
  {
    hide();
    glutils::drawable::select();
  }

  // Example for how to unhide an object on deselection.
  virtual void deselect() noexcept override
  {
    unhide();
    glutils::drawable::deselect();
  }
  */

};

/*----------------------------- Construction ---------------------------------*/

example_visualization::
example_visualization()
{
  add_drawable(new drawable_sphere);
  add_drawable(new drawable_sphere);
  add_drawable(new drawable_sphere);

  reset();
}

/*-------------------------- Visualization Interface -------------------------*/

void
example_visualization::
render()
{
  this->update();
  base_visualization::render();
}


void
example_visualization::
start()
{
  // Compute some movement for each sphere.
  for(size_t i = 0; i < 3; ++i) {
    // Start from the base transform.
    auto t = glutils::identity_transform();
    switch(i) {
      case 1:
        t[12] = -5;
        break;
      case 2:
        t[12] = 5;
      default:;
    }

    auto d = this->m_drawables[i];

    // Push 6 seconds worth of sinusoidal movement onto the transform stack.
    // At 30fps this is 180 frames.
    for(size_t j = 0; j < 180; ++j) {
      t[13] = 5 * std::sin((i * 10 + j) * glutils::TWOPI / 60);
      d->push_transform(t);
    }
  }
}


void
example_visualization::
reset()
{
  for(size_t i = 0; i < 3; ++i) {
    auto t = glutils::identity_transform();
    switch(i) {
      case 1:
        t[12] = -5;
        break;
      case 2:
        t[12] = 5;
      default:;
    }

    auto d = this->m_drawables[i];
    d->clear_transform();
    d->push_transform(t);
    d->update_transform();
  }
}

/*----------------------------------------------------------------------------*/
