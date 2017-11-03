#ifndef NETBOOK_INTERFACE_H
#define NETBOOK_INTERFACE_H

#include "HardwareInterface.h"

#include <vector>

namespace nonstd {
  class tcp_socket;
}


////////////////////////////////////////////////////////////////////////////////
/// A hardware interface for receiving marker data from a netbook running the
/// aruco detector.
////////////////////////////////////////////////////////////////////////////////
class ArucoDetectorInterface : public HardwareInterface
{

  public:

    ///@name Construction
    ///@{

    ArucoDetectorInterface(const std::string& _ip,
        const unsigned short _port);

    virtual ~ArucoDetectorInterface();

    ///@}
    ///@name Detector Interface
    ///@{

    std::vector<double> GetCoordinatesFromMarker() const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    nonstd::tcp_socket* m_socket{nullptr}; ///< The TCP connection object.

    ///@}

};

#endif
