#ifndef PID_FEEDBACK_H_
#define PID_FEEDBACK_H_

#include "ControllerMethod.h"

#include "ConfigurationSpace/Cfg.h"


////////////////////////////////////////////////////////////////////////////////
/// A controller that implements a simple PID control loop.
////////////////////////////////////////////////////////////////////////////////
class PIDFeedback : public ControllerMethod {

  public:

    ///@name Construction
    ///@{

    /// Construct a PID feedback controller.
    /// @param[in] _r The robot to control.
    /// @param[in] _p The proportional gain.
    /// @param[in] _i The integral gain.
    /// @param[in] _d The derivative gain.
    PIDFeedback(Robot* const _r, const double _p, const double _i,
        const double _d);

    virtual ~PIDFeedback() = default;

    ///@}
    ///@name Interface
    ///@{

    virtual Control operator()(const Cfg& _current, const Cfg& _target,
        const double _dt) override;

    ///@}

  protected:

    ///@name Control Selection Overrides
    ///@{

    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) override;

    ///@}
    ///@name Helpers
    ///@{

    /// PID controllers re-initialize every time they change targets.
    /// @param[in] _target The target configuration.
    void Initialize(const Cfg& _target);

    ///@}
    ///@name Internal State
    ///@{

    const double m_p;  ///< The gain for proportional error.
    const double m_i;  ///< The gain for integral error.
    const double m_d;  ///< The gain for derivative error.

    Cfg m_previousError;       ///< The last step's error value.
    Cfg m_integral;            ///< The integrated error history.
    Cfg m_target;              ///< The previous target.
    bool m_initialized{false}; ///< Is previous error available?

    ///@}

};

#endif
