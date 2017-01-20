#include "PIDFeedback.h"


/*------------------------------- Construction -------------------------------*/

PIDFeedback::
PIDFeedback(Robot* const _r, const double _p, const double _i, const double _d)
  : ControllerMethod(_r), m_p(_p), m_i(_i), m_d(_d) { }

/*-------------------------------- Interface ---------------------------------*/

Control
PIDFeedback::
operator()(const Cfg& _current, const Cfg& _target, const double _dt) {
  // Initialize on first use and whenever we change targets.
  if(!m_initialized || _target != m_target)
    Initialize(_target);

  return ControllerMethod::operator()(_current, _target, _dt);
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<double>
PIDFeedback::
ComputeDesiredForce(const Cfg& _current, const Cfg&, const double _dt) {
  // Compute the error terms.
  const Cfg error = m_target - _current;
  m_integral += error * _dt;
  const Cfg derivative = (error - m_previousError) / _dt;

  const Cfg control = error * m_p +
                          m_integral * m_i +
                          derivative * m_d;

  m_previousError = error;

  return control.GetData();
}

/*--------------------------------- Helpers ----------------------------------*/

void
PIDFeedback::
Initialize(const Cfg& _target) {
  m_initialized   = true;
  m_target        = _target;
  m_previousError = Cfg(_target.GetRobot());
  m_integral      = Cfg(_target.GetRobot());
}

/*----------------------------------------------------------------------------*/
