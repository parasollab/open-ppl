#include "Controller.h"

#include "ControlGenerators.h"
#include "SteeringFunctions.h"

#include "nonstd/container_ops.h"


/*-------------------------------- Construction ------------------------------*/

Controller::
Controller(Robot* const _r) : m_robot(_r) {}


Controller::
~Controller() {
  delete m_steeringFunction;
}

/*-------------------------- Controller Properties ---------------------------*/

void
Controller::
ComputeControls(const ControlGenerator& _g) {
  m_controls = _g.GenerateDiscreteSet(m_robot);
}


void
Controller::
SetSteeringFunction(SteeringFunction* const _f) {
  delete m_steeringFunction;
  m_steeringFunction = _f;
}

/*--------------------------- Planning Interface -----------------------------*/

const std::set<Control>&
Controller::
GetControls() const {
  return m_controls;
}


Cfg
Controller::
Test(const Control& _c, const Cfg& _start, const size_t _ticks) const {
  /// @TODO
  return _start;
}

/*-------------------------- Simulation Interface ----------------------------*/

bool
Controller::
Ready() const noexcept {
  return m_ready;
}


void
Controller::
SetTarget(const Cfg& _c) {
  m_target = _c;
}


void
Controller::
Step() {
  /// @TODO Get the start state from the simulated robot.
  Cfg start;

  auto desiredForce = (*m_steeringFunction)(start, m_target);
  auto unitForce = nonstd::unit(desiredForce);

  // Find the control that produces the closest result to the desired force.
  /// @TODO After continuous controls are implemented, account for that here.
  ///       For continuous signal over a discrete set, reduce the signal value
  ///       if needed to match the desired force.
  ///       For continuous control space, find the closest point within the
  ///       control space to the desired force.
  Control best;
  double bestDot = -1;
  for(const auto& control : m_controls) {
    auto force = nonstd::unit(control.GetForce());

    double dot = nonstd::dot<double>(unitForce, force);
    if(dot > bestDot) {
      bestDot = dot;
      best = control;
    }
  }

  // Apply the best control.
  best.Execute();
}

/*----------------------------------------------------------------------------*/
