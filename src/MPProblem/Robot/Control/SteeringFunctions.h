#ifndef STEERING_FUNCTIONS_H_
#define STEERING_FUNCTIONS_H_

#include <vector>

#include "MPProblem/ConfigurationSpace/Cfg.h"


////////////////////////////////////////////////////////////////////////////////
/// Steering functions determine the generalized force needed to move a robot
/// from one configuration to another.
////////////////////////////////////////////////////////////////////////////////
struct SteeringFunction {

  ///@name Construction
  ///@{

  virtual ~SteeringFunction();

  ///@}
  ///@name Interface
  ///@{

  /// Steer a robot from a starting to an ending configuration.
  /// @param[in] _s The start configuration.
  /// @param[in] _e The end configuration.
  /// @return A generalized force that will steer the robot from the start _s
  ///         to the end _e.
  virtual std::vector<double> operator()(const Cfg& _s, const Cfg& _e) = 0;

  ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// A steering function that implements a simple PID control loop.
////////////////////////////////////////////////////////////////////////////////
struct PIDFeedback : public SteeringFunction {

  ///@name Construction
  ///@{

  PIDFeedback(const double _p, const double _i, const double _d);
  virtual ~PIDFeedback();

  ///@}
  ///@name Interface
  ///@{

  virtual std::vector<double> operator()(const Cfg& _s, const Cfg& _e) override;

  ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_p{1};  ///< The gain for proportional error.
    double m_i{1};  ///< The gain for integral error.
    double m_d{1};  ///< The gain for derivative error.

    Cfg m_previousError;       ///< The last step's error value.
    Cfg m_integral;            ///< The integrated error history.
    bool m_initialized{false}; ///< Is previous error available?

    ///@}

};

#endif
