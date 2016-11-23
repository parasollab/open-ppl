#include "Control.h"

#include "MPProblem/Robot/Actuation/Actuator.h"


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

/*----------------------------------------------------------------------------*/

// Old parsing stuff that I will probably throw away.

//#include "Utilities/IOUtils.h"
//void
//Control::
//Read(istream& _is, CountingStreamBuffer& _cbs) {
//  m_power.clear();
//  string control;
//  getline(_is, control);
//  istringstream iss(control);
//  double v;
//  while(iss >> v)
//    m_power.push_back(v);
//}
//
//
//ostream&
//operator<<(ostream& _os, const Control& _c) {
//  ostringstream oss;
//  for(const auto& v : _c)
//    oss << v << " ";
//  string s = oss.str();
//  return _os << s.substr(0, s.length()-1);
//}
