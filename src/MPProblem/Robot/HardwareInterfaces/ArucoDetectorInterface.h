#ifndef PMPL_ARUCO_DETECTOR_INTERFACE_H
#define PMPL_ARUCO_DETECTOR_INTERFACE_H

#include "SensorInterface.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nonstd {
  class tcp_socket;
}

class ArucoMarkerMap;
class ArucoObservation;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// A hardware interface for receiving marker data from a netbook running the
/// aruco detector.
////////////////////////////////////////////////////////////////////////////////
class ArucoDetectorInterface : public SensorInterface
{

  public:

    ///@name Construction
    ///@{

    ArucoDetectorInterface(XMLNode& _node, const std::string& _ip,
        const unsigned short _port = 4002);

    virtual ~ArucoDetectorInterface();

    ///@}
    ///@name SensorInterface overrides
    ///@{

    virtual SensorType GetType() const noexcept override;

    virtual void SendCommand(const SensorCommand& _c) override;

    virtual std::vector<mathtool::Transformation> GetLastTransformations()
        override;

    virtual std::vector<double> GetUncertainty() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::unique_ptr<nonstd::tcp_socket> m_socket; ///< TCP connection object.

    /// Marker data from the last measurement.
    std::vector<ArucoObservation> m_observations;

    /// Map of the known locations for aruco markers.
    std::unique_ptr<ArucoMarkerMap> m_markerMap;

    mutable std::mutex m_lock; ///< Lock for sending/retrieving data.

    ///@}

};

#endif
