#include "HardwareInterface.h"

#include <algorithm>
#include <sstream>

#include "Utilities/PMPLExceptions.h"
#include "nonstd/io.h"
#include "nonstd/numerics.h"


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

HardwareInterface::Command::
Command(const ControlSet& _c, const double _s) : controls(_c), seconds(_s) { }


bool
HardwareInterface::Command::
operator==(const Command& _other) const noexcept {
  return controls == _other.controls and nonstd::approx(seconds, _other.seconds);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Hardware Interface ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

HardwareInterface::
HardwareInterface(const std::string& _name, const std::string& _ip,
    const unsigned short _port) : m_name(_name), m_ip(_ip), m_port(_port) {}

/*------------------------ Hardware Robot Properties -------------------------*/

const std::string&
HardwareInterface::
GetName() const noexcept {
  return m_name;
}


void
HardwareInterface::
SetName(const std::string& _name) noexcept {
  m_name = _name;
}


const std::string&
HardwareInterface::
GetIP() const noexcept {
  return m_ip;
}


void
HardwareInterface::
SetIP(const std::string& _ip) noexcept {
  m_ip = _ip;
}


const unsigned short
HardwareInterface::
GetPort() const noexcept {
  return m_port;
}


void
HardwareInterface::
SetPort(const unsigned short _port) noexcept {
  m_port = _port;
}

/*------------------------------ Command Queue -------------------------------*/

void
HardwareInterface::
EnqueueCommand(const ControlSet& _controls, const double _seconds) {
  this->EnqueueCommand({_controls, _seconds});
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<double>
HardwareInterface::
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

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ XML Factory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#include "Utilities/XMLNode.h"

#ifdef PMPL_USE_ICREATE
#include "ICreateInterface.h"
#include "NetbookInterface.h"
#endif

HardwareInterface*
HardwareInterfaceFactory(XMLNode& _node) {
  // Get the IP and port.
  const std::string ip = _node.Read("ip", true, "",
      "The IPv4 address for the robot hardware.");

  unsigned short port = _node.Read<unsigned short>("port", false, 0, 0,
      std::numeric_limits<unsigned short>::max(),
      "The on-board controller port.");

  // For some reason I can't get GCC to ignore an unused variable warning here,
  // assigning to self as a hacky work-around.
  port = port;

  // Get the hardware type and downcase.
  const std::string parsedType = _node.Read("hardware", true, "",
      "The type of hardware.");
  std::string type = parsedType;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  // Match the type string to known interfaces.
#ifdef PMPL_USE_ICREATE
  if(type == "icreate")
    return port == 0 ? new ICreateInterface(ip)
                     : new ICreateInterface(ip, port);
#endif

  throw ParseException(_node.Where(),
      "Unrecognized hardware '" + parsedType + "'.");
  return nullptr;
}

/*----------------------------------------------------------------------------*/
