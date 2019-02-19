#include "Utilities/ClockClass.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>


/*------------------------------- ClockClass ---------------------------------*/

ClockClass::
ClockClass() {
  ClearClock();
}


void
ClockClass::
SetName(const std::string& _name) {
  m_clockName = _name;
}


void
ClockClass::
ClearClock() {
  m_sTime = m_uTime = m_suTime = m_uuTime = 0;
}


void
ClockClass::
StartClock() {
  struct rusage buf;
  getrusage(RUSAGE_THREAD, &buf);
  m_suTime = buf.ru_utime.tv_usec;
  m_sTime = buf.ru_utime.tv_sec;
}


void
ClockClass::
StopClock() {
  struct rusage buf;
  getrusage(RUSAGE_THREAD, &buf);
  m_uuTime += buf.ru_utime.tv_usec - m_suTime;
  m_uTime += buf.ru_utime.tv_sec - m_sTime;
}


void
ClockClass::
StopPrintClock(std::ostream& _os) {
  StopClock();
  PrintClock(_os);
}


void
ClockClass::
PrintClock(std::ostream& _os) {
  _os << m_clockName << ": " << GetSeconds() << " sec" << std::endl;
}


double
ClockClass::
GetSeconds() const {
  return (double)m_uuTime/1e6+m_uTime;
}


int
ClockClass::
GetUSeconds() const {
  return (int)(m_uTime * 1e6 + m_uuTime);
}

/*----------------------------------------------------------------------------*/
