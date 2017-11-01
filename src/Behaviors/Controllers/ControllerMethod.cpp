#include "ControllerMethod.h"

#include "Behaviors/Controllers/ControlSetGenerators.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Actuator.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "nonstd/container_ops.h"


/*------------------------------ Construction --------------------------------*/

ControllerMethod::
ControllerMethod(Robot* const _r) : m_robot(_r) { }


ControllerMethod::
ControllerMethod(Robot* const _r, XMLNode& _node) : ControllerMethod(_r) {
  m_debug = _node.Read("debug", false, false, "Show debug output");

  // Parse control set node(s) if provided.
  for(auto& child : _node)
    if(child.Name() == "ControlSet") {
      // Generate a control set from this node.
      auto controls = XMLControlSetGenerator(_r, child);

      // If we don't have a control set yet, generate one directly from the XML
      // node.
      if(!m_controls) {
        SetControlSet(controls);
        continue;
      }

      // Otherwise a control set already exists. Add the controls from this node
      // and ensure there are no duplicates.
      std::copy(controls->begin(), controls->end(),
          std::back_inserter(*m_controls));
      RemoveDuplicateControls(m_controls);
    }
}


ControllerMethod::
~ControllerMethod() {
  delete m_controls;
}

/*-------------------------------- Interface ---------------------------------*/

Control
ControllerMethod::
operator()(const Cfg& _current, const Cfg& _target, const double _dt) {
  if(m_debug)
    std::cout << "Computing desired force to go from " << _current << " to "
              << _target
              << "\nTarget / Current = " << _target / _current
              << std::endl;

  // Compute the ideal force.
  auto force = this->ComputeDesiredForce(_current, _target, _dt);

  // Return the control that produces the nearest result.
  return this->ComputeNearestControl(_current, std::move(force));
}


ControlSet*
ControllerMethod::
GetControlSet() noexcept {
  return m_controls;
}


void
ControllerMethod::
SetControlSet(ControlSet* const _c) noexcept {
  delete m_controls;
  m_controls = _c;
}

/*---------------------------- Control Selection -----------------------------*/

Control
ControllerMethod::
ComputeNearestControl(const Cfg& _current, std::vector<double>&& _force) {
  // Find the control which produces the nearest thing to _force.
  // If the controller has a discrete control set, we must choose from that
  // (limited) set of controls. Otherwise, we can use any control in the
  // actuators' control spaces.
  if(this->GetControlSet())
    return ComputeNearestDiscreteControl(_current, std::move(_force));
  else
    return ComputeNearestContinuousControl(_current, std::move(_force));
}


Control
ControllerMethod::
ComputeNearestContinuousControl(const Cfg& _current,
    std::vector<double>&& _force) {
  Control best;
  double bestDot = -1;

  // Find the unit vector of the desired force in the robot's local coordinates.
  auto desiredDirection = nonstd::unit(_force);

  // Check each actuator and find the nearest control.
  for(const auto& actuatorPair : m_robot->GetActuators()) {
    const auto& actuator = actuatorPair.second;

    // Compute the nearest
    const auto signal = actuator->ComputeNearestSignal(_force);
    const auto unitForce = nonstd::unit(actuator->ComputeForce(signal));

    const double dot = nonstd::dot<double>(desiredDirection, unitForce);
    if(dot >= bestDot) {
      best = Control{actuator, signal};
      bestDot = dot;
    }
  }

  // Debug.
  if(m_debug) {
    std::cout << "Computing best continuous control...\n" << std::endl
              << "\tdesired force (local): " << _force << std::endl
              << "\tbest control:          " << best << std::endl;
    if(best.actuator)
      std::cout << "\tbest force (local):    "
                << best.actuator->ComputeForce(best.signal) << std::endl;
    std::cout << "\tnearest control has directional similarity " << bestDot
              << std::endl;
  }

  // Assert that the selected control is sensible.
  if(bestDot == -1)
    throw RunTimeException(WHERE, "Best control is directly counter-productive. "
        "This is probably an error in the control set definition.");

  return best;
}


Control
ControllerMethod::
ComputeNearestDiscreteControl(const Cfg& _current, std::vector<double>&& _force) {
  Control best;
  double bestDot = -1;

  // Find the unit vector of the desired force in the robot's local coordinates.
  auto desiredDirection = nonstd::unit(_force);

  // Rank force similarity first by direction and then by magnitude.
  for(const auto& control : *GetControlSet()) {
    const auto unifForce = nonstd::unit(control.GetForce());
    const double dot = nonstd::dot<double>(desiredDirection, unifForce);
    if(dot > bestDot) {
      best = control;
      bestDot = dot;
    }
  }

  // Debug.
  if(m_debug) {
    std::cout << "Computing best discrete control..." << std::endl
              << "\tdesired force (local): " << desiredDirection << std::endl
              << "\tbest control:          " << best << std::endl;
    if(best.actuator)
      std::cout << "\tbest force:    "
                << best.actuator->ComputeForce(best.signal) << std::endl;
    std::cout << "\tnearest control has directional similarity " << bestDot
              << std::endl;
  }

  // Assert that the selected control is sensible.
  if(bestDot == -1)
    throw RunTimeException(WHERE, "Best control is directly counter-productive. "
        "This is probably an error in the control set definition.");

  return best;

}

/*----------------------------------------------------------------------------*/
