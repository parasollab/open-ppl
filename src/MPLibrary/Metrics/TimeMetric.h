#ifndef TIME_METRIC_H
#define TIME_METRIC_H

#include "MetricMethod.h"
#include "Utilities/MetricUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TimeMetric : public MetricMethod<MPTraits> {

  public:

    TimeMetric();
    TimeMetric(XMLNode& _node);
    virtual ~TimeMetric() {}

    virtual void Print(ostream& _os) const;

    double operator()();

};

template <typename MPTraits>
TimeMetric<MPTraits>::
TimeMetric() {
  this->SetName("TimeMetric");
}

template <typename MPTraits>
TimeMetric<MPTraits>::
TimeMetric(XMLNode& _node)
  : MetricMethod<MPTraits>(_node) {
    this->SetName("TimeMetric");
}

template <typename MPTraits>
void
TimeMetric<MPTraits>::
Print(ostream& _os) const {
  _os << "Time allowed" << endl;
}


template <typename MPTraits>
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
