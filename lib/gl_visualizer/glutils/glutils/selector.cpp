#include "glutils/selector.h"

#include <algorithm>
#include <iostream>
#include <utility>

#ifdef __APPLE__
  #include <OpenGL/glu.h>
#else
  #include <GL/glu.h>
#endif

#include "glutils/drawable.h"


namespace glutils {

  /*-------------------------- Map Management --------------------------------*/

  void
  selector::
  add_drawable(drawable* _d)
  {
    m_selection_map[_d->id()] = _d;
    m_color_map[_d->picking_color()] = _d;
  }


  void
  selector::
  remove_drawable(drawable* _d)
  {
    {
      auto iter = m_selection_map.find(_d->id());
      if(iter != m_selection_map.end()) {
        m_hits.erase(iter->second);
        m_unhits.erase(iter->second);
        m_selection_map.erase(iter);
      }
    }
    {
      auto iter = m_color_map.find(_d->picking_color());
      if(iter != m_color_map.end()) {
        m_hits.erase(iter->second);
        m_unhits.erase(iter->second);
        m_color_map.erase(iter);
      }
    }
  }

  /*------------------------------ Selection ---------------------------------*/

  const selector::hit_list&
  selector::
  hits() const noexcept
  {
    return m_hits;
  }


  const selector::hit_list&
  selector::
  unhits() const noexcept
  {
    return m_unhits;
  }


  void
  selector::
  select(const size_t _x, const size_t _y, const size_t _w, const size_t _h)
  {
    setup_gl_context(_x, _y, _w, _h);
    if(m_debug)
      std::cout << "\tUsing GL selection..." << std::endl;

    // Set the selection buffer, switch to selection mode, and initalize the
    // name stack.
    glSelectBuffer(m_buffer_size, m_buffer);
    glRenderMode(GL_SELECT);
    glInitNames();

    // Render each drawable for selection.
    for(auto& d : m_selection_map)
      d.second->render_select();

    // Return to render mode and parse the hit buffer.
    const GLint num_hits = glRenderMode(GL_RENDER);

    // Select everything in the target window if it was larger than 5 pixels.
    const bool select_all = _w > 5 or _h > 5;

    parse_selection_buffer(num_hits, select_all);

    restore_gl_context();
  }


  void
  selector::
  color_pick(const size_t _x, const size_t _y)
  {
    setup_gl_context(_x, _y, 1, 1);
    if(m_debug)
      std::cout << "\tUsing color picking..." << std::endl;

    // Need to disable lighting when color picking.
    const bool use_lighting = glIsEnabled(GL_LIGHTING);
    if(use_lighting)
      glDisable(GL_LIGHTING);

    // Render each drawable for color picking.
    for(auto& d : m_selection_map)
      d.second->render_color_pick();

    // Read pixels and look for color.
    unsigned char buffer[3];
    glReadPixels(0, 0, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, buffer);
    color c(buffer[0] / 255., buffer[1] / 255., buffer[2] / 255.);

    if(m_debug)
      std::cout << "\tcolor result: " << c << std::endl;

    hit_list new_hits = m_color_map.count(c) ? hit_list{m_color_map[c]}
                                             : hit_list{};
    update_hits(std::move(new_hits));

    // Turn lighting back on if needed.
    if(use_lighting)
      glEnable(GL_LIGHTING);

    restore_gl_context();
  }

  /*--------------------------------- Debug ----------------------------------*/

  void
  selector::
  debug(const bool _show)
  {
    m_debug = _show;
  }

  /*-------------------------------- Helpers ---------------------------------*/

  void
  selector::
  setup_gl_context(const size_t _x, const size_t _y, const size_t _w,
      const size_t _h)
  {
    if(m_debug)
      std::cout << "glutils::selector: "
                << "Selecting at (" << _x << ", " << _y << ") with range ["
                << _w << ":" << _h << "]" << std::endl;

    // Resize the projection matrix (i.e. picking window) to select only items
    // near the target.
    // Grab current viewport.
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    // Grab current projection matrix.
    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    // Save the current projection matrix.
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();

    // Set up projection matrix to match picking view/box.
    glLoadIdentity();                        // Start from origin.
    gluPickMatrix(_x, _y, _w, _h, viewport); // Set picking window.
    glMultMatrixd(projection);               // Set current view.

    // Prepare to draw for selection.
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }


  void
  selector::
  restore_gl_context()
  {
    // Restore the previous projection matrix and return to modelview mode.
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
  }


  void
  selector::
  parse_selection_buffer(const GLint _num_hits, const bool _all)
  {
    if(m_debug)
      std::cout << "\tParsing " << (_all ? "all" : "nearest") << " of "
                << _num_hits << " hits." << std::endl;

    GLfloat nearest_distance = std::numeric_limits<GLfloat>::max();
    GLuint nearest_id = 0;
    std::set<GLuint> hits;

    // Parse current hits.
    GLuint* buffer = m_buffer - 1;
    for(GLint i = 0; i < _num_hits; ++i) {
      // Get the number of names on the stack for this hit.
      GLuint num_names = *++buffer;

      // Get the near distance for this hit.
      float z_near = static_cast<float>(*++buffer) / 0x7fffffff;
      // float z_far  = static_cast<float>(m_buffer[2]) / 0x7fffffff;
      ++buffer; // skip far distance.

      // Add each name to the hit list.
      for(size_t n = 0; n < num_names; ++n) {
        // Add this name.
        const GLuint name = *++buffer;
        hits.insert(name);

        // Track nearest hit.
        if(z_near < nearest_distance) {
          nearest_distance = z_near;
          nearest_id = name;
        }
      }
    }

    // Get the drawables that were selected.
    hit_list new_hits;
    if(!_all && _num_hits > 0)
      new_hits.insert(m_selection_map[nearest_id]);
    else
      for(const auto hit : hits)
        if(m_selection_map.count(hit))
          new_hits.insert(m_selection_map[hit]);

    update_hits(std::move(new_hits));
  }


  void
  selector::
  update_hits(hit_list&& _hits)
  {
    // The unhits are drawables that were previously selected but aren't anymore.
    hit_list unhits;
    std::set_difference(m_hits.begin(), m_hits.end(), _hits.begin(), _hits.end(),
        std::inserter(unhits, unhits.begin()));

    m_unhits = std::move(unhits);
    m_hits = std::move(_hits);

    if(m_debug) {
      std::cout << "\tHits:";
      for(const auto& hit : m_hits)
        std::cout << " " << hit;

      std::cout << "\n\tUnhits:";
      for(const auto& unhit : m_unhits)
        std::cout << " " << unhit;

      std::cout << std::endl;
    }
  }

  /*--------------------------------------------------------------------------*/

}
