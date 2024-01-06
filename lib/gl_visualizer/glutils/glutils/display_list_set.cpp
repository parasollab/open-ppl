#include "glutils/display_list_set.h"

#include "nonstd/exception.h"


namespace glutils {

  /*------------------------------ Construction ------------------------------*/

  display_list_set::
  display_list_set(const size_t _num) :
      m_num(_num)
  { }


  display_list_set::
  ~display_list_set()
  {
    uninitialize();
  }


  void
  display_list_set::
  initialize()
  {
    // Generate a call list for rendering, selection, and highlighting.
    m_firstID = glGenLists(m_num);

    // If we got 0, there was a problem getting the lists.
    if(m_firstID == 0)
      throw nonstd::exception(WHERE) << "glutils::display_list_set::initialize() "
                                     << "error: could not allocate OpenGL "
                                     << "display lists for the object.";

    m_ready = true;
  }


  void
  display_list_set::
  uninitialize()
  {
    if(m_firstID != 0)
      glDeleteLists(m_firstID, m_num);
    m_firstID = 0;

    m_ready = false;
  }

  /*--------------------------------------------------------------------------*/

}
