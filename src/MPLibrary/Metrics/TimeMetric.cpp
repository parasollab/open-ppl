#include "TimeMetric.h"

#include "Utilities/MetricUtils.h"

/*------------------------------ Construction --------------------------------*/

std::string
TimeMetric::
s_clockName("Total Running Time");


TimeMetric::
TimeMetric() {
  this->SetName("TimeMetric");
}


TimeMetric::
TimeMetric(XMLNode& _node) : MetricMethod(_node) {
  this->SetName("TimeMetric");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
TimeMetric::
Initialize() {
  // Clear the previous clock (if any) and start again.
  auto stats =  this->GetStatClass();
  stats->ClearClock(s_clockName);
  stats->StartClock(s_clockName);
}

/*-------------------------- MetricMethod Overrides --------------------------*/

double
TimeMetric::
operator()() {
  auto stats = this->GetStatClass();

  // Report the elapsed time.
  stats->StopClock(s_clockName);
  stats->StartClock(s_clockName);

  return (double)stats->GetSeconds(s_clockName);
}

/*----------------------------------------------------------------------------*/
