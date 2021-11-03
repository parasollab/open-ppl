#ifndef PPL_DIFF_DRIVE_CONTROLLER_H_
#define PPL_DIFF_DRIVE_CONTROLLER_H_

#include "ControllerMethod.h"

#include <limits>

////////////////////////////////////////////////////////////////////////////////
/// Selects controls that require a differential drive robot to first turn
/// towards its target and then translate in the desired direction.
////////////////////////////////////////////////////////////////////////////////
class DiffDriveController : public ControllerMethod {

  public:

    ///@name Construction
    ///@{

    /// Construct a simple differential drive controller.
    /// @param[in] _r The robot to control.
    /// @param[in] _gain The direct error gain.
    /// @param[in] _max  The maximum force to request.
    DiffDriveController(Robot* const _r, const double _gain,
        const double _max = std::numeric_limits<double>::infinity());

    /// Construct a simple differential drive controller from an XML node.
    /// @param[in] _r The robot to control.
    /// @param[in] _node The XML node to parse.
    DiffDriveController(Robot* const _r, XMLNode& _node);

    /// Copy a controller for another robot.
    /// @param _r The destination robot.
    /// @param _c The controller to copy.
    DiffDriveController(Robot* const _r, const DiffDriveController& _c);

    virtual std::unique_ptr<ControllerMethod> Clone(Robot* const _r) const;

    virtual ~DiffDriveController();

    ///@}

  protected:

    ///@name Control Selection Overrides
    ///@{

    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) override;

    ///@}

};


#endif