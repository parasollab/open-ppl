#include "HardwareInterface.h"

#include <algorithm>

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Hardware Interface ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

HardwareInterface::
HardwareInterface(const std::string& _name, const std::string& _ip,
    const unsigned short _port, const double _communicationTime) : m_name(_name),
    m_ip(_ip), m_port(_port), m_communicationTime(_communicationTime) {}

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


unsigned short
HardwareInterface::
GetPort() const noexcept {
  return m_port;
}


void
HardwareInterface::
SetPort(const unsigned short _port) noexcept {
  m_port = _port;
}


double
HardwareInterface::
GetCommunicationTime() const noexcept {
  return m_communicationTime;
}


void
HardwareInterface::
SetCommunicationTime(const double _t) noexcept {
  m_communicationTime = _t;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ XML Factory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifdef PMPL_USE_ICREATE
#include "ICreateInterface.h"
#include "ArucoDetectorInterface.h"
#endif

HardwareInterface*
HardwareInterfaceFactory(XMLNode& _node) {
  // Get the IP and port.
  const std::string ip = _node.Read("ip", true, "",
      "The IPv4 address for the robot hardware.");

  unsigned short port = _node.Read<unsigned short>("port", true, 0, 0,
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
    return new ICreateInterface(ip, port);
  else if(type == "aruco")
    return new ArucoDetectorInterface(ip, port);
#endif

  throw ParseException(_node.Where(),
      "Unrecognized hardware '" + parsedType + "'.");
  return nullptr;
}

/*----------------------------------------------------------------------------*/
