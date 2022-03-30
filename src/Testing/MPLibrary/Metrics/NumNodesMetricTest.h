#ifndef PPL_NUM_NODES_METRIC_TEST_H_
#define PPL_NUM_NODES_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/NumNodesMetric.h"

template <typename MPTraits> // TODO need to add template to each method (See TimeMetricTest)
class NumNodesMetricTest :  virtual public NumNodesMetric<MPTraits>,
                            public MetricMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NumNodesMetricTest();

    NumNodesMetricTest(MPProblem* _problem); // TODO see TimeMetricTest comment about this

    ~NumNodesMetricTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestMetric() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/

NumNodesMetricTest::
NumNodesMetricTest() : NumNodesMetric() {}

NumNodesMetricTest::
NumNodesMetricTest(MPProblem* _problem) : MetricMethodTest(),
                                                             NumNodesMetric(){
  m_MPProblem = _problem; // TODO Delete this (see time metric test comment)
}

NumNodesMetricTest::
~NumNodesMetricTest() {}



/*--------------------- Test Interface Functions ---------------------*/

typename NumNodesMetricTest::TestResult
NumNodesMetricTest::
TestMetric() {

  // Set up environment from parent
  double metric = Metric(); // TODO call methods of a parent class using this-> (ex. this->Metric())
  double expected = this->GetGroupRoadmap() ? (this->GetGroupRoadmap()->Size()) : (this->GetRoadmap()->Size());

  // Correct value?
  if(metric == expected){
    return std::make_pair(true,"Testing NumNodesMetric::PASSED"); // TODO try to setup the strings like in Testing/Sampelrs/UniformRandomSamplerTest.h
  }
  return std::make_pair(true,"Testing NumNodesMetric, Wrong number of nodes recieved.");
}

#endif
