#include "TMPLibrary/OccupiedInterval.h"

#include "MPProblem/Robot/Robot.h"

OccupiedInterval::
OccupiedInterval(){}

OccupiedInterval::
OccupiedInterval(Agent* _r, Cfg _sL, Cfg _eL, double _sT, double _eT) :
  m_agent(_r), m_startLocation(_sL), m_endLocation(_eL), m_startTime(_sT),
  m_endTime(_eT){
  }

OccupiedInterval::
OccupiedInterval(const OccupiedInterval& _other){
  m_agent = _other.m_agent;
  m_startLocation = _other.m_startLocation;
  m_endLocation = _other.m_endLocation;
  m_startTime = _other.m_startTime;
  m_endTime = _other.m_endTime;
  m_task = _other.m_task;
}

OccupiedInterval::
~OccupiedInterval(){}

OccupiedInterval&
OccupiedInterval::
operator=(const OccupiedInterval& _other){
  m_agent = _other.m_agent;
  m_startLocation = _other.m_startLocation;
  m_endLocation = _other.m_endLocation;
  m_startTime = _other.m_startTime;
  m_endTime = _other.m_endTime;
  m_task = _other.m_task;
  return *this;
}
Agent*
OccupiedInterval::
GetAgent(){
  return m_agent;
}

Cfg
OccupiedInterval::
GetStartLocation(){
  return m_startLocation;
}

Cfg
OccupiedInterval::
GetEndLocation(){
  return m_endLocation;
}

double
OccupiedInterval::
GetStartTime(){
  return m_startTime;
}

double
OccupiedInterval::
GetEndTime(){
  return m_endTime;
}

std::pair<Cfg, double>
OccupiedInterval::
GetStart(){
  return std::make_pair(m_startLocation, m_startTime);
}

std::pair<Cfg, double>
OccupiedInterval::
GetEnd(){
  return std::make_pair(m_endLocation, m_endTime);
}

std::shared_ptr<MPTask>
OccupiedInterval::
GetTask(){
  return m_task;
}

void
OccupiedInterval::
SetStartLocation (Cfg _start)
{
  m_startLocation = _start;
}

void
OccupiedInterval::
SetEndLocation (Cfg _end)
{
  m_endLocation = _end;
}
void
OccupiedInterval::
SetStartTime (double _start)
{
  m_startTime = _start;
}
void
OccupiedInterval::
SetEndTime (double _end)
{
  m_endTime = _end;
}
void
OccupiedInterval::
SetStart(Cfg _startLoc, double _startTime)
{
  m_startLocation = _startLoc;
  m_startTime = _startTime;
}

void
OccupiedInterval::
SetEnd(Cfg _endLoc, double _endTime)
{
  m_endLocation = _endLoc;
  m_endTime = _endTime;
}

void
OccupiedInterval::
SetTask(std::shared_ptr<MPTask> _task){
  m_task = _task;
}

bool
OccupiedInterval::
CheckTimeOverlap(OccupiedInterval _interval){
  double start = _interval.GetStartTime();
  double end = _interval.GetEndTime();

  if ((start >= m_startTime && start < m_endTime) ||
      (end <= m_endTime && end > m_startTime))
    return true;
  return false;
}

void
OccupiedInterval::
MergeIntervals(std::list <OccupiedInterval*>& _intervals){
  std::list<OccupiedInterval*>::iterator it = _intervals.begin();
  while (it != _intervals.end()){
    auto next = std::next(it);
    double nextTime = (*next)->GetStartTime();
    if ((*it)->GetAgent() != (*next)->GetAgent()){
      throw RunTimeException(WHERE) << "Unable to merge intervals. "
        << "Different agents exist within the given set of occupied intervals.";

      if (nextTime >= (*it)->GetStartTime() && nextTime <= (*it)->GetEndTime()){
        if((*next)->GetEndTime() > (*it)->GetEndTime()){
          (*it)->SetEndTime((*next)->GetEndTime());
          _intervals.erase(next);
        }
        else
          _intervals.erase(next);
      }
      else
        it ++;
    }
  }
}

bool
OccupiedInterval::
operator<(OccupiedInterval _interval){
  return m_startTime < _interval.GetStartTime();
}

bool
OccupiedInterval::
operator==(const OccupiedInterval& _interval) const {
  return (m_startTime == _interval.m_startTime
       && m_endTime == _interval.m_endTime
       && m_startLocation == _interval.m_startLocation
       && m_endLocation == _interval.m_endLocation
       && m_agent == _interval.m_agent);
}

std::string
OccupiedInterval::
Print(){
  std::string output;
  if(m_agent)
    output = m_agent->GetRobot()->GetLabel() +
    "\nStart time: " + std::to_string(m_startTime) +
    " Start Location: " + m_startLocation.PrettyPrint() +
    "\nEnd Time: " + std::to_string(m_endTime) +
    " End Location: " + m_endLocation.PrettyPrint();
  else
    output = "No robot"
    "\nStart time: " + std::to_string(m_startTime);
  return output;
}

