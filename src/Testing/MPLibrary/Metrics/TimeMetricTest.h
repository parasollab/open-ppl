#ifndef PPL_TIME_METRIC_TEST_H_
#define PPL_TIME_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/TimeMetric.h"

class TimeMetricTest :  public TimeMetric,
                               public MetricMethodTest {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    TimeMetricTest();

    TimeMetricTest(MPProblem* _problem);

    ~TimeMetricTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestMetric() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/

TimeMetricTest::
TimeMetricTest() : TimeMetric() {}

TimeMetricTest::
TimeMetricTest(MPProblem* _problem) : MetricMethodTest(),
                                                             TimeMetric(){
  m_MPProblem = _problem;
}

TimeMetricTest::
~TimeMetricTest() {}



/*--------------------- Test Interface Functions ---------------------*/

typename TimeMetricTest::TestResult
TimeMetricTest::
TestMetric() {

  // Set up environment from parent
  double metric = Metric();
  auto stats = this->GetStatClass();

  // Report the elapsed time.
  stats->StopClock(s_clockName);
  stats->StartClock(s_clockName);

  double expected = (double)stats->GetSeconds(s_clockName);

  // Correct value?
  if(metric == expected){
    return std::make_pair(true,"Testing TimeMetric::PASSED");
  }
  return std::make_pair(true,"Testing TimeMetric, Wrong time elapsed recieved.");
}

#endif
