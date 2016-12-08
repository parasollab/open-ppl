#ifndef SIMPLE_CONTROLLER_H_
#define SIMPLE_CONTROLLER_H_

#include "ControllerMethod.h"

class Cfg;


////////////////////////////////////////////////////////////////////////////////
/// Chooses the control that drives most directly towards the target.
////////////////////////////////////////////////////////////////////////////////
class SimpleController : public ControllerMethod {

  public:

    ///@name Construction
    ///@{

    /// Construct a simple controller.
    /// @param[in] _r The robot to control.
    /// @param[in] _gain The direct error gain.
    SimpleController(Robot* const _r, const double _gain);

    virtual ~SimpleController() = default;

    ///@}

  protected:

    ///@name Control Selection Overrides
    ///@{

    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) override;

    ///@}
    ///@name Internal State
    ///@{

    const double m_gain; ///< The proportional error gain.

    ///@}

};

#endif
