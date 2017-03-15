#ifndef DRAWABLE_MULTIBODY_H_
#define DRAWABLE_MULTIBODY_H_

#include <cstddef>
#include <string>
#include <vector>

#include "glutils/drawable.h"

#include "DrawableBody.h"

class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Instructions for drawing a pmpl MultiBody in the Simulator.
////////////////////////////////////////////////////////////////////////////////
class DrawableMultiBody final : public glutils::drawable {

  ///@name Internal State
  ///@{

  std::vector<DrawableBody> m_bodies; ///< Drawables for each sub-body.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a drawable representation of a PMPL multibody.
    /// @param[in] _m The PMPL multibody to draw.
    DrawableMultiBody(MultiBody* _m);

    virtual ~DrawableMultiBody() = default;

    ///@}
    ///@name MultiBody Support
    ///@{

    /// Get the number of bodies in this drawable.
    size_t GetNumBodies() const noexcept;

    /// Push a transform for a given body.
    /// @param[in] _i The index of the body to update.
    /// @param[in] _t The transform to push.
    void PushTransform(const size_t _i, const glutils::transform& _t);

    /// Update the transform queue for all bodies.
    void UpdateTransform();

    ///@}
    ///@name Drawable Overrides
    ///@{

    virtual void draw() override;
    virtual void draw_select() override;
    virtual void draw_selected() override;
    virtual void draw_highlighted() override;

    virtual void select() noexcept override;
    virtual void deselect() noexcept override;

    virtual void highlight() noexcept override;
    virtual void unhighlight() noexcept override;

    ///@}

};

#endif
