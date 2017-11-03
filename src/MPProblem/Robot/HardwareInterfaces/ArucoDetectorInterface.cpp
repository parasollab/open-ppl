#include "ArucoDetectorInterface.h"

#include <unistd.h>

#include "nonstd/numerics.h"

#include "nonstd/tcp_socket.h"

#include "HardwareInterface.h"

#include "packet.h"


/// The measured time that we need to send a message to the aruco detector.
static constexpr double arucoDetectorCommunicationTime = .3;

/*------------------------------- Construction -------------------------------*/

ArucoDetectorInterface::
ArucoDetectorInterface(const std::string& _ip, const unsigned short _port)
    : HardwareInterface("ArucoDetector", _ip, _port,
        arucoDetectorCommunicationTime) {
  // Connect to the netbook's socket for aruco data.
  m_socket = new nonstd::tcp_socket(m_ip, std::to_string(m_port));
}


ArucoDetectorInterface::
~ArucoDetectorInterface() {
  delete m_socket; // Disconnect is automatic.
}

/*------------------------------ Command Queue -------------------------------*/

std::vector<double>
ArucoDetectorInterface::
GetCoordinatesFromMarker() const {
  size_t count;
  //cout << "Sending packet (1,2,3)..." << endl;
  char c;
  *m_socket >> c;
  count = c;
  //cout << "Size of c " << count << endl;

  // The detector will attempt to loacalize itself for each marker that it sees.
  // Receive these estimates here and sum.
  double x = 0.0;
  double y = 0.0;
  double angle = 0.0;
  for(size_t i=0;i<count;i++) {
    packet p1;
    *m_socket >> p1;
    x += p1.x/10000.0;
    y += p1.y/10000.0;
    angle += p1.angle/10000.0;
  }

  // Divide the summed estimates by the number of estimates to produce an
  // average.
  /// @TODO Consider using a kalman filter instead of averaging.
  double totalMarkers = (double)count;
  x = x/totalMarkers;
  y = y/totalMarkers;
  angle = angle/totalMarkers;
  std::vector<double> coordinates = {x, y, (angle)*(180/M_PI)};
  return coordinates;
}

/*----------------------------------------------------------------------------*/
