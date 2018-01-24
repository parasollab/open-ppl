#include "QueuedHardwareInterface.h"

#include <algorithm>
#include <sstream>

#include "Utilities/PMPLExceptions.h"
#include "nonstd/io.h"
#include "nonstd/numerics.h"


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

QueuedHardwareInterface::Command::
Command(const ControlSet& _c, const double _s) : controls(_c), seconds(_s) { }


bool
QueuedHardwareInterface::Command::
operator==(const Command& _other) const noexcept {
  return controls == _other.controls and nonstd::approx(seconds, _other.seconds);
}

/*------------------------------- Construction -------------------------------*/

QueuedHardwareInterface::
QueuedHardwareInterface(const std::string& _name, const std::string& _ip,
    const unsigned short _port, const double _communicationTime)
    : HardwareInterface(_name, _ip, _port, _communicationTime) {}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~ QueuedHardwareInterface ~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------ Command Queue -------------------------------*/

void
QueuedHardwareInterface::
EnqueueCommand(const ControlSet& _controls, const double _seconds) {
  this->EnqueueCommand({_controls, _seconds});
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<double>
QueuedHardwareInterface::
AggregatedControlVector(const ControlSet& _controls) const {
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

    // I don't know why the STL calls it 'vector', but doesn't give us += ...
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
