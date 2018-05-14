#include "SensorInterface.h"

#include "Utilities/PMPLExceptions.h"


/*------------------------------- Construction -------------------------------*/

SensorInterface::
~SensorInterface() = default;

/*--------------------------- Hardware Interface -----------------------------*/

HardwareInterface::HardwareType
SensorInterface::
GetHardwareType() const noexcept {
  return HardwareInterface::HardwareType::Sensor;
}

/*----------------------------- Sensor Interface -----------------------------*/

std::vector<mathtool::Transformation>
SensorInterface::
GetLastTransformations() {
  throw RunTimeException(WHERE) << "Not implemented.";
}


std::vector<std::vector<double>>
SensorInterface::
GetLastJointAngles() {
  throw RunTimeException(WHERE) << "Not implemented.";
}

/*----------------------------------------------------------------------------*/
