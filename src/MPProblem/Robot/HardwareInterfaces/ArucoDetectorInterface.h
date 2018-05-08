#ifndef PMPL_ARUCO_DETECTOR_INTERFACE_H
#define PMPL_ARUCO_DETECTOR_INTERFACE_H

#include "SensorInterface.h"

#include <vector>

namespace nonstd {
  class tcp_socket;
}

class ArucoObservation;


////////////////////////////////////////////////////////////////////////////////
/// A hardware interface for receiving marker data from a netbook running the
/// aruco detector.
////////////////////////////////////////////////////////////////////////////////
class ArucoDetectorInterface : public SensorInterface
{

  public:

    ///@name Construction
    ///@{

    ArucoDetectorInterface(const std::string& _ip,
        const unsigned short _port = 4002);

    virtual ~ArucoDetectorInterface();

    ///@}
    ///@name SensorInterface overrides
    ///@{

    virtual SensorType GetType() const noexcept override;

    virtual void SendCommand(const SensorCommand& _c) override;

    virtual std::vector<double> GetLastMeasurement() override;

    virtual std::vector<double> GetUncertainty() override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Use the marker map to estimate the robot's global position from the
    /// current observations.
    /// @return Data for a 6-DOF cfg representing the robot's base
    ///         transformation.
    std::vector<double> EstimateGlobalPositionFromObservations();

    ///@}
    ///@name Internal State
    ///@{

    nonstd::tcp_socket* m_socket{nullptr}; ///< TCP connection object.

    /// Marker data from the last measurement.
    std::vector<ArucoObservation> m_observations;

    ///@}

};

#endif
