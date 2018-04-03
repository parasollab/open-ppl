
#include "BatteryBreak.h"


/*------------------------------- Construction -------------------------------*/
BatteryBreak::
BatteryBreak(){}


BatteryBreak::
BatteryBreak(Cfg _cfg, double _time)
  : m_place(_cfg), m_time(_time) {
}

BatteryBreak::
BatteryBreak(const BatteryBreak& _break)
  : m_place(_break.GetPlace()), m_time(_break.GetTime()) {
}

BatteryBreak&
BatteryBreak::
operator=(const BatteryBreak& _break){
  m_time = _break.GetTime();
  m_place = _break.GetPlace();
  return *this;
}

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
