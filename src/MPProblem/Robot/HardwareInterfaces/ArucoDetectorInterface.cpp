#include "ArucoDetectorInterface.h"

#include "Commands.h"

#include "nonstd/numerics.h"
#include "nonstd/tcp_socket.h"

#include <unistd.h>

#include <iostream>

/// The measured time that we need to send a message to the aruco detector.
static constexpr double arucoDetectorCommunicationTime = .3;


////////////////////////////////////////////////////////////////////////////////
/// A packet of information about a detected aruco marker's position in the
/// camera's local frame.
///
/// The packet is used to send data over the network in integer format. The
/// floating point values (x,y,angle) will be multiplied by s_factor so that
/// they can be transported as integers.
////////////////////////////////////////////////////////////////////////////////
struct packet {

  static constexpr double s_factor = 10000;

  int id{0}, x{0}, y{0}, angle{0};

  packet() = default;

  packet(int _id, int _x, int _y, int _angle)
      : id(_id), x(_x), y(_y), angle(_angle) {}

};


////////////////////////////////////////////////////////////////////////////////
/// An observation of a marker relative to the camera's current position.
////////////////////////////////////////////////////////////////////////////////
struct ArucoObservation {

  size_t id{0};

  double x{0}, y{0}, angle{0};

  ArucoObservation() = default;

  ArucoObservation(const packet& _p)
    : id(_p.id),
      x(_p.x / packet::s_factor),
      y(_p.y / packet::s_factor),
      angle(_p.angle / packet::s_factor)
  { }

};

/*------------------------------- Construction -------------------------------*/

ArucoDetectorInterface::
ArucoDetectorInterface(const std::string& _ip, const unsigned short _port)
    : SensorInterface("ArucoDetector", _ip, _port, arucoDetectorCommunicationTime)
{
  m_socket = new nonstd::tcp_socket();
}


ArucoDetectorInterface::
~ArucoDetectorInterface() {
  delete m_socket; // Disconnect is automatic.
}

/*----------------------------- Sensor Interface -----------------------------*/

SensorInterface::SensorType
ArucoDetectorInterface::
GetType() const noexcept {
  return SensorInterface::SensorType::Transformation;
}


void
ArucoDetectorInterface::
SendCommand(const SensorCommand& _c) {
  // Connect to the netbook's socket for aruco data.
  /// @TODO It seems like we should be able to connect just once here...
  m_socket->connect(m_ip, std::to_string(m_port));

  // Clear the previous measurement.
  m_observations.clear();

  // Get the number of markers observed.
  char c;
  *m_socket >> c;
  size_t count = c;

  // Receive a packet for each marker and save the observation.
  for(size_t i = 0; i < count; ++i) {
    packet p;
    *m_socket >> p;

    /// @TODO What is this check for?
    if(p.x == -1 or p.y == -1)
      continue;

    m_observations.emplace_back(p);
  }

  m_socket->disconnect();
}


std::vector<double>
ArucoDetectorInterface::
GetLastMeasurement() {
  return EstimateGlobalPositionFromObservations();
}


std::vector<double>
ArucoDetectorInterface::
GetUncertainty() {
  /// @TODO Determine our measurement uncertainty.
  return {};
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<double>
ArucoDetectorInterface::
EstimateGlobalPositionFromObservations() {
  // If we didn't see any markers, we cannot return a reasonable estimate.
  if(m_observations.empty())
    return {};

  /// @TODO Use the marker map to determine what we saw.
  return {};
}

/*----------------------------------------------------------------------------*/
