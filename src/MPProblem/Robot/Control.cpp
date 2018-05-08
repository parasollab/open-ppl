#include "Control.h"

#include "Actuator.h"
#include "Utilities/PMPLExceptions.h"

#include "nonstd/io.h"
#include "nonstd/numerics.h"

#include <algorithm>
#include <sstream>


/*------------------------------ Construction --------------------------------*/

Control::
Control() = default;


Control::
Control(Actuator* const _actuator, const Signal& _signal)
  : actuator(_actuator), signal(_signal) {
}

/*--------------------------- Simulation Interface ---------------------------*/

std::vector<double>
Control::
GetForce() const {
  if(actuator)
    // If we have an actuator, ask it for the force.
    return actuator->ComputeForce(signal);
  else
    // Otherwise this is a coast control - return a 0 force vector.
    return std::vector<double>(signal.size(), 0);
}


void
Control::
Execute() const {
  if(actuator)
    // If we have an actuator, ask it to execute this control.
    actuator->Execute(signal);
  // Otherwise this is a coast control - we don't need to do anything.
}


void
Control::
Execute(btMultiBody* const _model) const {
  if(actuator)
    actuator->Execute(signal, _model);
}

/*-------------------------- Ordering and Equality ---------------------------*/

bool
Control::
operator<(const Control& _rhs) const noexcept {
  if(actuator < _rhs.actuator)
    return true;
  else if(actuator > _rhs.actuator)
    return false;
  else {
    for(size_t i = 0; i < signal.size(); ++i) {
      if(signal[i] < _rhs.signal[i])
        return true;
      else if(signal[i] > _rhs.signal[i])
        return false;
    }
  }
  return false;
}


bool
Control::
operator==(const Control& _rhs) const noexcept {
  return actuator == _rhs.actuator && signal == _rhs.signal;
}


bool
Control::
operator!=(const Control& _rhs) const noexcept {
  return !(*this == _rhs);
}

/*-------------------------------- Display -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Control& _c) {
  if(_c.signal.empty())
    return _os;
  _os << (_c.actuator ? _c.actuator->GetLabel() : "coast") << " ";
  return _os << nonstd::print_container(_c.signal);
}

/*------------------------------- Control Set --------------------------------*/

std::vector<double>
AggregatedControlVector(const ControlSet& _controls) {
  // We can't do anything without some controls.
  if(_controls.empty())
    return {};

  // Add up the control signals and 'forces', which may actually be velocities
  // for robots with 1st order dynamics (like iCreates).
  std::vector<double> force  = _controls[0].GetForce(),
                      signal = _controls[0].signal;

  for(size_t i = 1; i < _controls.size(); ++i) {
    const auto& signalI = _controls[i].signal;
    const auto forceI = _controls[i].GetForce();

    std::transform(signal.begin(), signal.end(), signalI.begin(), signal.begin(),
        std::plus<double>());
    std::transform(force.begin(), force.end(), forceI.begin(), force.begin(),
        std::plus<double>());
  }

  // If any signals have exceeded the sensible range, this control set is asking
  // the robot to exceed its defined limitations. Throw an error rather than risk
  // damaging the hardware.
  for(const auto val : signal)
    if(!nonstd::approx_in_bounds(val, -1., 1.)) {
      std::ostringstream oss;
      oss << "Requested signal " << signal << " exceeds robot limits!";
      throw RunTimeException(WHERE, oss.str());
    }

  // If the signal is OK, return the aggregated force.
  return force;
}

/*----------------------------------------------------------------------------*/
