#include "ControllerMethod.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/PMPLExceptions.h"

#include "nonstd/container_ops.h"


/*------------------------------ Construction --------------------------------*/

ControllerMethod::
ControllerMethod(Robot* const _r) : m_robot(_r) { }

/*-------------------------------- Interface ---------------------------------*/

Control
ControllerMethod::
operator()(const Cfg& _current, const Cfg& _target, const double _dt) {
  // Compute the ideal force.
  auto force = this->ComputeDesiredForce(_current, _target, _dt);

  // Return the control that produces the nearest result.
  return this->ComputeNearestControl(_current, std::move(force));
}

/*---------------------------- Control Selection -----------------------------*/

Control
ControllerMethod::
ComputeNearestControl(const Cfg& _current, std::vector<double>&& _force) {
  // Find the control which produces the nearest thing to _force.
  // Prefer continuous controls if both are available.
  if(m_robot->GetControlSpace())
    return ComputeNearestContinuousControl(_current, std::move(_force));
  else if(m_robot->GetControlSet())
    return ComputeNearestDiscreteControl(_current, std::move(_force));
  else {
    throw RunTimeException(WHERE, "Robot has no controls.");
    return Control();
  }
}


Control
ControllerMethod::
ComputeNearestContinuousControl(const Cfg& _current,
    std::vector<double>&& _force) {
  /// @TODO Find the nearest control in the space.
  throw RunTimeException(WHERE, "Not yet implemented. Please implement or use"
      " a discretized control set.");
}

#include "MPProblem/Robot/Actuator.h"

Control
ControllerMethod::
ComputeNearestDiscreteControl(const Cfg& _current, std::vector<double>&& _force) {
  Control best;
  double bestDot = -1;
  auto desired = nonstd::unit(_force);
  std::cout << "desired force:" << desired << std::endl;

  // Rank force similarity first by direction and then by magnitude.
  for(const auto& control : *m_robot->GetControlSet()) {
    auto force = nonstd::unit(control.GetForce());
    double dot = nonstd::dot<double>(desired, force);
    if(dot > bestDot) {
      best = control;
      bestDot = dot;
    }
  }

  std::cout << "best control/force:\n\t" << best << "\n\t";
  if(best.actuator)
    std::cout << best.actuator->ComputeForce(best.signal) << std::endl;

  // Assert that the selected control is sensible.
  if(bestDot == -1)
    throw RunTimeException(WHERE, "Best control is directly counter-productive. "
        "This is probably an error in the control set definition.");

  return best;
}

/*----------------------------------------------------------------------------*/
