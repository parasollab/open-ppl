#include "TimeMetric.h"

TimeMetric::TimeMetric() {
  this->SetName("TimeMetric");
}

TimeMetric::TimeMetric(XMLNodeReader& _node, MPProblem* _problem)
  : MetricsMethod(_node, _problem) {
    this->SetName("TimeMetric");
    if(m_debug) PrintOptions(cout);
}

TimeMetric::~TimeMetric() {
}

void TimeMetric::PrintOptions(ostream& _os){
  _os << "Time allowed" << endl;
}


double
TimeMetric::operator()() {
  StatClass * timeStatClass = GetMPProblem()->GetStatClass();
  static int flag=0;
  string timeClockName = "Total running time";
  if(flag==0){
    timeStatClass->StartClock(timeClockName);
    flag=1;
  }
  timeStatClass->StopClock(timeClockName);
  timeStatClass->StartClock(timeClockName);
  return (double)timeStatClass->GetSeconds(timeClockName);
}
