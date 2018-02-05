#include "ArucoDetectorInterface.h"

#include <iostream>

#include <unistd.h>

#include "nonstd/numerics.h"
#include "nonstd/tcp_socket.h"

/// The measured time that we need to send a message to the aruco detector.
static constexpr double arucoDetectorCommunicationTime = .3;


////////////////////////////////////////////////////////////////////////////////
/// A packet of information about a detected aruco marker.
////////////////////////////////////////////////////////////////////////////////
struct packet {

  int id{0}, x{0}, y{0}, angle{0};

  packet() = default;

  packet(int _id, int _x, int _y, int _angle)
      : id(_id), x(_x), y(_y), angle(_angle) {}

};

/*------------------------------- Construction -------------------------------*/

ArucoDetectorInterface::
ArucoDetectorInterface(const std::string& _ip, const unsigned short _port)
    : HardwareInterface("ArucoDetector", _ip, _port,
        arucoDetectorCommunicationTime) {
}


ArucoDetectorInterface::
~ArucoDetectorInterface() {
  delete m_socket; // Disconnect is automatic.
}

/*------------------------------ Accessors -------------------------------*/

size_t
ArucoDetectorInterface::
GetNumMarkersSeen() {
  return m_numMarkersSeen;
}

/*------------------------------ Command Queue -------------------------------*/

std::vector<double>
ArucoDetectorInterface::
GetCoordinatesFromMarker() {
  // Connect to the netbook's socket for aruco data.
  m_socket = new nonstd::tcp_socket(m_ip, std::to_string(m_port));
  size_t count;
  char c;
  *m_socket >> c;
  count = c;

  // The detector will attempt to loacalize itself for each marker that it sees.
  // Receive these estimates here and sum.
  double x = 0,
         y = 0,
         angle = 0;

  // Set the number of markers seen at this point
  m_numMarkersSeen = count;
  for(size_t i = 0; i < count; ++i) {
    packet p1;
    *m_socket >> p1;

    if(p1.x != -1 and p1.y != -1) {
      x += p1.x/10000.0;
      y += p1.y/10000.0;
      angle += p1.angle/10000.0;
    }
  }

  m_socket->disconnect();

  // Divide the summed estimates by the number of estimates to produce an
  // average.
  /// @TODO Consider using a kalman filter instead of averaging.
  double totalMarkers = (double)count;
  if(totalMarkers > 0) {
    x = x / totalMarkers;
    y = y / totalMarkers;
    angle = angle / totalMarkers;
    std::vector<double> coordinates = {x, y, (angle)*(180/M_PI)};
    return coordinates;
  }

  // If we didn't see any markers, we cannot return a reasonable estimate.
  return {};
}

/*----------------------------------------------------------------------------*/
