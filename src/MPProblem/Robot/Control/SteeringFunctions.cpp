#include "SteeringFunctions.h"


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SteeringFunction ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

SteeringFunction::
~SteeringFunction() = default;

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PIDFeedback ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

PIDFeedback::
PIDFeedback(const double _p, const double _i, const double _d) : m_p(_p),
    m_i(_i), m_d(_d) { }


PIDFeedback::
~PIDFeedback() = default;


std::vector<double>
PIDFeedback::
operator()(const Cfg& _s, const Cfg& _e) {
  // Initialize previous error and history on first use.
  if(!m_initialized) {
    m_initialized = true;
    auto robotIndex = _s.GetRobotIndex();
    m_previousError = Cfg(robotIndex);
    m_integral      = Cfg(robotIndex);
  }

  /// @TODO replace hard-coded timestep with one from the problem.
  constexpr static double dt = .01;

  // Compute the error terms.
  const Cfg error = _e - _s;
  m_integral += error * dt;
  const Cfg derivative = (error - m_previousError) / dt;

  const Cfg control = error * m_p +
                      m_integral * m_i +
                      derivative * m_d;

  m_previousError = error;

  return control.GetData();
}

/*----------------------------------------------------------------------------*/
