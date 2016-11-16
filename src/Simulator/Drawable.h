#ifndef DRAWABLE_H_
#define DRAWABLE_H_

#include <string>

#include "glutils/drawable_call_list.h"

class btBoxShape;
namespace glutils {
  class triangulated_model;
}

////////////////////////////////////////////////////////////////////////////////
/// Instructions for drawing an object in the simulator.
////////////////////////////////////////////////////////////////////////////////
class Drawable : public glutils::drawable_call_list {

  ///@name Internal State
  ///@{

  glutils::triangulated_model* m_model{nullptr};

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a drawable representation of a simulated object from an obj
    /// file.
    /// @param[in] _filename The obj file to build from.
    Drawable(const std::string& _filename);

    /// Construct a drawable representation of a simulated object from a
    /// triangulated model.
    //Drawable(glutils::triangulated_model* _t);

    virtual ~Drawable();

    ///@}

  protected:

    ///@name drawable_call_list Overrides
    ///@{

    virtual void build() override;
    virtual void build_selected() override;
    virtual void build_highlighted() override;

    ///@}
};

#endif
