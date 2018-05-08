#ifndef PMPL_SENSOR_INTERFACE_H
#define PMPL_SENSOR_INTERFACE_H

#include "HardwareInterface.h"

class SensorCommand;


////////////////////////////////////////////////////////////////////////////////
/// An abstract interface for a hardware sensor.
////////////////////////////////////////////////////////////////////////////////
class SensorInterface : public HardwareInterface {

  public:

    ///@name Local Types
    ///@{

    using HardwareInterface::HardwareType;

    /// The supported sensor types.
    enum SensorType {Transformation, JointAngle};

    ///@}
    ///@name Construction
    ///@{

    using HardwareInterface::HardwareInterface;

    virtual ~SensorInterface();

    ///@}
    ///@name Hardware Properties
    ///@{

    virtual HardwareType GetHardwareType() const noexcept;

    ///@}
    ///@name Sensor Interface
    ///@{

    /// Indicates the type of sensor.
    /// @return The sensor type.
    virtual SensorType GetType() const noexcept = 0;

    /// Instruct the sensor to take a measurement.
    virtual void SendCommand(const SensorCommand& _c) = 0;

    /// Get the last measurement.
    virtual std::vector<double> GetLastMeasurement() = 0;

    /// Get the measurement uncertainty.
    /// @TODO This should maybe be a covariance matrix instead of an uncertainty
    ///       in each measured dimension.
    virtual std::vector<double> GetUncertainty() = 0;

    /// Get the measurement transformation matrix H such that a configuration c
    /// produces a measurement y = Hc.
    /// @TODO We need a dynamically sizable matrix for this, either dlib or
    ///       eigen will probably be the choice.

    ///@}
};

#endif
