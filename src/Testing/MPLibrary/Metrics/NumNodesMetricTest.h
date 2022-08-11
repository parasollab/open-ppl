#ifndef PPL_NUM_NODES_METRIC_TEST_H_
#define PPL_NUM_NODES_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/NumNodesMetric.h"

template <typename MPTraits>
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

    NumNodesMetricTest(XMLNode& _node);

    ~NumNodesMetricTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestMetric() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/
template <typename MPTraits>
NumNodesMetricTest<MPTraits>::
NumNodesMetricTest() : NumNodesMetric<MPTraits>() {}

template <typename MPTraits>
NumNodesMetricTest<MPTraits>::
NumNodesMetricTest(XMLNode& _node) : MetricMethod<MPTraits>(_node),
                                           NumNodesMetric<MPTraits>(_node) {}

template <typename MPTraits>
NumNodesMetricTest<MPTraits>::
~NumNodesMetricTest() {}



/*--------------------- Test Interface Functions ---------------------*/
template <typename MPTraits>
typename NumNodesMetricTest<MPTraits>::TestResult
NumNodesMetricTest<MPTraits>::
TestMetric() {

  // Set up environment from parent
  double metric = (*this)();
  double expected = this->GetGroupRoadmap() ? (this->GetGroupRoadmap()->Size()) : (this->GetRoadmap()->Size());

  // Correct value?
  if(metric == expected){
    return std::make_pair(true,"NumNodesMetric::PASSED");
  }
  return std::make_pair(false,"NumNodesMetric::FAILED, Wrong number of nodes recieved.");
}

#endif
