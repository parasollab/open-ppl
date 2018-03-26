
#include "BatteryBreak.h"


/*------------------------------- Construction -------------------------------*/

BatteryBreak::
BatteryBreak(Cfg _cfg, double _time)
  : m_place(_cfg), m_time(_time) {
}


BatteryBreak::
~BatteryBreak() = default;

/*-------------------------------- Accessors ---------------------------------*/

Cfg
BatteryBreak::
GetPlace() const noexcept {
  return m_place;
}


double
BatteryBreak::
GetTime() const noexcept {
  return m_time;
}

/*----------------------------------------------------------------------------*/
