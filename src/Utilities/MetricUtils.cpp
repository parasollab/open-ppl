#include "MetricUtils.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

#include "Utilities/PMPLExceptions.h"

struct rusage buf;


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
  getrusage(RUSAGE_SELF, &buf);
  m_suTime = buf.ru_utime.tv_usec;
  m_sTime = buf.ru_utime.tv_sec;
}


void
ClockClass::
StopClock() {
  getrusage(RUSAGE_SELF, &buf);
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
  _os << m_clockName << ": " << GetSeconds() << " sec" <<endl;
}


double
ClockClass::
GetSeconds() {
  return (double)m_uuTime/1e6+m_uTime;
}

int
ClockClass::
GetUSeconds() {
  return (int)(m_uTime * 1e6 + m_uuTime);
}

/*-------------------------------- StatClass ---------------------------------*/

StatClass::
StatClass() {
  ClearStats();
}


void
StatClass::
ClearStats() {
  m_numCollDetCalls.clear();
  m_lpInfo.clear();
  m_samplerInfo.clear();
  m_collDetCountByName.clear();
  m_isCollByName.clear();
  m_isCollTotal = 0;
}


int
StatClass::
IncNumCollDetCalls(const std::string& _cdName, const std::string& _callName) {
  m_numCollDetCalls[_cdName]++;
  // If a caller's name was provided
  // then increment the verification counter
  // with that name.
  m_collDetCountByName[_callName]++;
  return m_numCollDetCalls[_cdName];
}


size_t
StatClass::
GetIsCollTotal() {
  return m_isCollTotal;
}


void
StatClass::
IncCfgIsColl(const std::string& _callName) {
  m_isCollByName[_callName]++;
  m_isCollTotal++;
}


int
StatClass::
IncLPConnections(const std::string& _lpName, int _incr) {
  return std::get<1>(m_lpInfo[_lpName]) += _incr;
}


int
StatClass::
IncLPAttempts(const std::string& _lpName, int _incr) {
  return std::get<0>(m_lpInfo[_lpName]) += _incr;
}


int
StatClass::
IncLPCollDetCalls(const std::string& _lpName, int _incr) {
  return std::get<2>(m_lpInfo[_lpName]) += _incr;
}


void
StatClass::
IncNodesGenerated(const std::string& _samplerName, size_t _incr) {
  m_samplerInfo[_samplerName].second += _incr;
}


void
StatClass::
IncNodesAttempted(const std::string& _samplerName, size_t _incr) {
  m_samplerInfo[_samplerName].first += _incr;
}


void
StatClass::
StartClock(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  if(m_clockMap.find(_name) == m_clockMap.end())
    m_clockMap[_name].SetName(_name);
  m_clockMap[_name].StartClock();
}


void
StatClass::
StopClock(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].StopClock();
}


void
StatClass::
StopPrintClock(const std::string& _name, std::ostream& _os) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].StopPrintClock(_os);
}


void
StatClass::
PrintClock(const std::string& _name, std::ostream& _os) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].PrintClock(_os);
}


void
StatClass::
ClearClock(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].ClearClock();
}


double
StatClass::
GetSeconds(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  return m_clockMap[_name].GetSeconds();
}


int
StatClass::
GetUSeconds(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  return m_clockMap[_name].GetUSeconds();
}


double
StatClass::
GetStat(const std::string& _s) {
  return m_stats[_s];
}


void
StatClass::
SetStat(const std::string& _s, const double _v) {
  m_stats[_s] = _v;
}


void
StatClass::
IncStat(const std::string& _s, const double _v) {
  m_stats[_s] += _v;
}


std::vector<double>&
StatClass::
GetHistory(const std::string& _s) {
  return m_histories[_s];
}


void
StatClass::
AddToHistory(const std::string& _s, double _v) {
  m_histories[_s].push_back(_v);
}


void
StatClass::
SetAuxDest(const std::string& _s) {
  m_auxFileDest = _s;
}

/*------------------------------- MethodTimer --------------------------------*/

MethodTimer::
MethodTimer(StatClass* const _stats, const std::string& _label)
  : m_stats(_stats), m_label(_label) {
  m_stats->StartClock(m_label);
}


MethodTimer::
~MethodTimer() {
  m_stats->StopClock(m_label);
}

/*----------------------------------------------------------------------------*/
