#include "glutils/drawable.h"

namespace glutils {

  /*------------------------------ Construction ------------------------------*/

  drawable::
  drawable() noexcept
    : m_selection_id(generate_selection_id())
    , m_picking_color(generate_picking_color())
  {
    push_transform(identity_transform());
  }


  drawable::
  drawable(const drawable& _d) noexcept
    : m_selection_id(generate_selection_id())
    , m_picking_color(generate_picking_color())
    , m_transforms(_d.m_transforms)
  { }


  drawable::
  drawable(drawable&& _d) noexcept
    : m_selection_id(_d.m_selection_id)
    , m_picking_color(_d.m_picking_color)
    , m_selected(_d.m_selected.load())
    , m_highlighted(_d.m_highlighted.load())
    , m_transforms(std::move(_d.m_transforms))
  {
    _d.m_selected = false;
    _d.m_highlighted = false;
  }


  void
  drawable::
  initialize()
  {
    m_initialized = true;
  }


  void
  drawable::
  uninitialize()
  {
    m_initialized = false;
  }


  drawable::
  ~drawable() = default;

  /*-------------------------------- Rendering -------------------------------*/

  void
  drawable::
  render()
  {
    if(m_hidden)
      return;

    glPushMatrix();

    if(!m_initialized)
      initialize();

    apply_transform(m_transforms.front());

    draw();
    if(m_selected)
      draw_selected();
    if(m_highlighted)
      draw_highlighted();;

    glPopMatrix();
  }


  void
  drawable::
  render_select()
  {
    if(m_hidden)
      return;

    glPushMatrix();

    apply_transform(m_transforms.front());

    glPushName(m_selection_id);
    draw_select();
    glPopName();

    glPopMatrix();
  }


  void
  drawable::
  render_color_pick()
  {
    if(m_hidden)
      return;

    glPushMatrix();

    apply_transform(m_transforms.front());

    glColor4fv(m_picking_color);
    draw_select();

    glPopMatrix();
  }


  void
  drawable::
  hide() noexcept
  {
    m_hidden = true;
  }


  void
  drawable::
  unhide() noexcept
  {
    m_hidden = false;
  }


  bool
  drawable::
  is_hidden() const noexcept
  {
    return m_hidden;
  }

  /*------------------------------- Transform --------------------------------*/

  void
  drawable::
  push_transform(const transform& _t) noexcept
  {
    m_transforms.push(_t);
  }


  void
  drawable::
  update_transform() noexcept
  {
    if(m_transforms.size() > 1)
      m_transforms.pop();
  }


  void
  drawable::
  clear_transform(const transform& _t) noexcept
  {
    m_transforms = std::queue<transform, std::list<transform>>();
    m_transforms.push(_t);
  }

  /*---------------------- Highlighting and Selection ------------------------*/

  GLuint
  drawable::
  id() const noexcept
  {
    return m_selection_id;
  }


  const color&
  drawable::
  picking_color() const noexcept
  {
    return m_picking_color;
  }


  bool
  drawable::
  selected() const noexcept
  {
    return m_selected.load();
  }


  void
  drawable::
  select() noexcept
  {
    m_selected = true;
  }


  void
  drawable::
  deselect() noexcept
  {
    m_selected = false;
  }


  bool
  drawable::
  highlighted() const noexcept
  {
    return m_highlighted.load();
  }


  void
  drawable::
  highlight() noexcept
  {
    m_highlighted = true;
  }


  void
  drawable::
  unhighlight() noexcept
  {
    m_highlighted = false;
  }

  /*------------------------ Selection ID Generation -------------------------*/

  std::mutex
  drawable::m_selection_id_gate;


  GLuint
  drawable::
  generate_selection_id() noexcept
  {
    m_selection_id_gate.lock();
    static GLuint next = 0;
    GLuint id = ++next;
    m_selection_id_gate.unlock();
    return id;
  }


  color
  drawable::
  generate_picking_color() noexcept
  {
    m_selection_id_gate.lock();
    static GLuint next = 0;
    GLuint code = ++next;
    /// @TODO Return colors from destroyed objects to a pool of available
    ///       colors. Currently the program will crash after ~16M objects have
    ///       been instantiated.
    nonstd::assert_msg(next < pow(256, 3), "glutils::drawable:: no more picking "
        "colors available.");
    m_selection_id_gate.unlock();

    // Convert code to a 24-bit color (1 byte each for r,g,b).
    /// @TODO Figure out how to poll the system for the correct color bit depth
    ///       and adapt accordingly. Assuming 24-bit seems to be working
    ///       empirically on many systems, but robustness requires that we talk
    ///       to the GL context to be sure.
    unsigned char r, g, b;
    r = g = b = 0;

    r = code % 256;             // Lowest byte is red.
    if(code > 255)
      g = (code -= 256) % 256;  // Next byte is green.
    if(code > 255)
      b = (code -= 256) % 256;  // Next byte is blue.

    return color(r / 255., g / 255., b / 255.);
  }

  /*--------------------------------------------------------------------------*/

}
