#include "SensorInterface.h"


/*------------------------------- Construction -------------------------------*/

SensorInterface::
~SensorInterface() = default;

/*--------------------------- Hardware Interface -----------------------------*/

HardwareInterface::HardwareType
SensorInterface::
GetHardwareType() const noexcept {
  return HardwareInterface::HardwareType::Sensor;
}

/*----------------------------------------------------------------------------*/
