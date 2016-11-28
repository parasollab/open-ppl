#ifndef PID_FEEDBACK_H_
#define PID_FEEDBACK_H_

#include "SteeringFunctionMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// A steering function that implements a simple PID control loop.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PIDFeedback : public SteeringFunctionMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    PIDFeedback(const double _p, const double _i, const double _d);

    virtual ~PIDFeedback() = default;

    ///@}
    ///@name Interface
    ///@{

    virtual Control operator()(const CfgType& _current, const CfgType& _target)
        override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// PID steering functions re-initialize every time they change targets.
    /// @param[in] _target The target configuration.
    void Initialize(const CfgType& _target);

    /// Compute the desired force using PID control.
    /// @param[in] _current The current configuration.
    /// @return The ideal generalized force according to the PID rule.
    std::vector<double> ComputeDesiredForce(const CfgType& _current);

    /// Compute the control that produces the closest force to the ideal.
    /// @param[in] _current The current configuration.
    /// @param[in] _force The desired force.
    /// @return The control whos result is nearest to _force.
    Control ComputeNearestControl(const CfgType& _current,
        std::vector<double>&& _force);

    ///@}
    ///@name Internal State
    ///@{

    double m_p{1};  ///< The gain for proportional error.
    double m_i{1};  ///< The gain for integral error.
    double m_d{1};  ///< The gain for derivative error.

    CfgType m_previousError;   ///< The last step's error value.
    CfgType m_integral;        ///< The integrated error history.
    CfgType m_target;          ///< The previous target.
    bool m_initialized{false}; ///< Is previous error available?

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
PIDFeedback<MPTraits>::
PIDFeedback(const double _p, const double _i, const double _d) : m_p(_p),
    m_i(_i), m_d(_d) { }

/*-------------------------------- Interface ---------------------------------*/

template <typename MPTraits>
Control
PIDFeedback<MPTraits>::
operator()(const CfgType& _current, const CfgType& _target) {
  // Initialize on first use and whenever we change targets.
  if(!m_initialized || _target != m_target)
    Initialize(_target);

  // Compute the ideal force.
  auto force = ComputeDesiredForce(_current);

  // Return the control that produces the nearest result.
  return ComputeNearestControl(_current, std::move(force));
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
PIDFeedback<MPTraits>::
Initialize(const CfgType& _target) {
  m_initialized   = true;
  m_target        = _target;
  m_previousError = CfgType(_target.GetRobotIndex());
  m_integral      = CfgType(_target.GetRobotIndex());
}


template <typename MPTraits>
std::vector<double>
PIDFeedback<MPTraits>::
ComputeDesiredForce(const CfgType& _current) {
  /// @TODO replace hard-coded timestep with one from the problem.
  constexpr static double dt = .01;

  // Compute the error terms.
  const CfgType error = m_target - _current;
  m_integral += error * dt;
  const CfgType derivative = (error - m_previousError) / dt;

  const CfgType control = error * m_p +
                          m_integral * m_i +
                          derivative * m_d;

  m_previousError = error;

  return control.GetData();
}


template <typename MPTraits>
Control
PIDFeedback<MPTraits>::
ComputeNearestControl(const CfgType& _current, std::vector<double>&& _force) {
  //auto robot = m_target->GetRobot();
  /// @TODO Check each control to see which produces the nearest thing to _force.
  ///       Prefer continuous controls if both are available.
  /// @TODO Rank force similarity first by direction and then by magnitude.

  // Return the best available control.
  Control best;
  return best;
}

/*----------------------------------------------------------------------------*/

#endif
