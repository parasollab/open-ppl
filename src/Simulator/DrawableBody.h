#ifndef DRAWABLE_BODY_H_
#define DRAWABLE_BODY_H_

#include "glutils/drawable_call_list.h"

class Body;
class Drawable;


////////////////////////////////////////////////////////////////////////////////
/// Drawable representation of a pmpl Body.
////////////////////////////////////////////////////////////////////////////////
class DrawableBody final : public glutils::drawable_call_list {

  ///@name Internal State
  ///@{

  Drawable* const m_parent; ///< The owning Drawable object.
  const Body* const m_body; ///< The Body to draw.

  ///@}

  public:

    ///@name Construction
    ///@{

    DrawableBody(Drawable* const _parent, const Body* const _body);

    virtual ~DrawableBody() = default;

    ///@}

  protected:

    ///@name drawable_call_list Overrides
    ///@{

    virtual void build() override;
    virtual void build_select() override;
    virtual void build_selected() override;
    virtual void build_highlighted() override;

    ///@}

};

#endif
