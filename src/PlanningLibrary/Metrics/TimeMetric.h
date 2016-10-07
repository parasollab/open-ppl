#ifndef TIMEMETRIC_H
#define TIMEMETRIC_H

#include "MetricMethod.h"
#include "Utilities/MetricUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TimeMetric : public MetricMethod<MPTraits> {
  public:

    TimeMetric();
    TimeMetric(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    virtual ~TimeMetric() {}

    virtual void Print(ostream& _os) const;

    double operator()();

};

template<class MPTraits>
TimeMetric<MPTraits>::
TimeMetric() {
  this->SetName("TimeMetric");
}

template<class MPTraits>
TimeMetric<MPTraits>::
TimeMetric(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
  : MetricMethod<MPTraits>(_problem, _node) {
    this->SetName("TimeMetric");
}

template<class MPTraits>
void
TimeMetric<MPTraits>::
Print(ostream& _os) const {
  _os << "Time allowed" << endl;
}


template<class MPTraits>
double
TimeMetric<MPTraits>::
operator()() {
  StatClass* timeStatClass = this->GetStatClass();
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

#endif
