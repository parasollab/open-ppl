#include "NetbookInterface.h"

#include <unistd.h>

#include "nonstd/numerics.h"

#include "utils/tcp_socket.h"

#include "HardwareInterface.h"

#include "packet.h"
/*------------------------------- Construction -------------------------------*/

NetbookInterface::
NetbookInterface(const std::string& _name,
    const std::string& _ip, const unsigned short _port)
    : m_name(_name),m_ip(_ip), m_port(_port) { }


NetbookInterface::
~NetbookInterface() {
}

/*------------------------------ Command Queue -------------------------------*/

std::vector<double>
NetbookInterface::
GetCoordinatesFromMarker() const {
  //cout << "Trying to get coordinates " << endl;
  utils::tcp_socket client(m_ip, "4002");
  //cout << "Connection successful " << endl;
  size_t count;
  //cout << "Sending packet (1,2,3)..." << endl;
  char c;
  client >> c;
  count = c;
  //cout << "Size of c " << count << endl;
  double x = 0.0;
  double y = 0.0;
  double angle = 0.0;
  for(size_t i=0;i<count;i++) {
    packet p1;
    client >> p1;
    x += p1.x/10000.0;
    y += p1.y/10000.0;
    angle += p1.angle/10000.0;
  }
  // Now let's average the values
  double totalMarkers = (double)count;
  x = x/totalMarkers;
  y = y/totalMarkers;
  angle = angle/totalMarkers;
  client.disconnect();
  std::vector<double> coordinates = {x, y, (angle)*(180/M_PI)};
  return coordinates;
}

/*----------------------------------------------------------------------------*/
