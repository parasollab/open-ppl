#ifndef ICREATE_CONTROLLER_H_
#define ICREATE_CONTROLLER_H_

#include "ControllerMethod.h"

#include <limits>


////////////////////////////////////////////////////////////////////////////////
/// Chooses the control that drives most directly towards the target.
////////////////////////////////////////////////////////////////////////////////
class ICreateController : public ControllerMethod {

  public:

    ///@name Construction
    ///@{

    /// Construct a simple controller.
    /// @param[in] _r The robot to control.
    /// @param[in] _gain The direct error gain.
    /// @param[in] _max  The maximum force to request.
    ICreateController(Robot* const _r, const double _gain,
        const double _max = std::numeric_limits<double>::infinity());

    /// Construct a simple controller from an XML node.
    /// @param[in] _r The robot to control.
    /// @param[in] _node The XML node to parse.
    ICreateController(Robot* const _r, XMLNode& _node);

    virtual ~ICreateController() = default;

    ///@}

  protected:

    ///@name Control Selection Overrides
    ///@{

    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) override;

    ///@}

};

#endif
