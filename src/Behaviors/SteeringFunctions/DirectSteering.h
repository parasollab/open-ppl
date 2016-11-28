#ifndef DIRECT_STEERING_H_
#define DIRECT_STEERING_H_

#include "SteeringFunctionMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Chooses the control that drives most directly towards the target.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DirectSteering : public SteeringFunctionMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    DirectSteering(const double _gain);

    virtual ~DirectSteering() = default;

    ///@name Interface
    ///@{

    virtual Control operator()(const CfgType& _current, const CfgType& _target)
        override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Compute the desired force.
    std::vector<double> ComputeDesiredForce(const CfgType& _delta) const;

    /// Compute the control that produces the closest force to the ideal.
    /// @param[in] _current The current configuration.
    /// @param[in] _force The desired force.
    /// @return The control whos result is nearest to _force.
    Control ComputeNearestControl(std::vector<double>&& _force) const;

    ///@}
    ///@name Internal State
    ///@{

    double m_gain{.02}; ///< The proportional error gain.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
DirectSteering<MPTraits>::
DirectSteering(const double _gain) : m_gain(_gain) { }

/*-------------------------------- Interface ---------------------------------*/

template <typename MPTraits>
Control
DirectSteering<MPTraits>::
DirectSteering(const CfgType& _current, const CfgType& _target) {
  const CfgType delta = _target - _current;
  auto force = ComputeDesiredForce(delta);
  return ComputeNearestControl(std::move(force));
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
inline
std::vector<double>
DirectSteering<MPTraits>::
ComputeDesiredForce(const CfgType& _delta) const {
  return _delta * m_gain;
}


template <typename MPTraits>
Control
DirectSteering<MPTraits>::
ComputeNearestControl(std::vector<double>&& _force) const {
  /// @TODO Ask actuators/robot/something for nearest control.
}

/*----------------------------------------------------------------------------*/

#endif
