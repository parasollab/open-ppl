#ifndef PPL_NUM_EDGES_METRIC_TEST_H_
#define PPL_NUM_EDGES_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/NumEdgesMetric.h"

template <typename MPTraits>
class NumEdgesMetricTest :  virtual public NumEdgesMetric<MPTraits>,
                            public MetricMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NumEdgesMetricTest();

    NumEdgesMetricTest(XMLNode& _node);

    ~NumEdgesMetricTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestMetric() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/
template <typename MPTraits>
NumEdgesMetricTest<MPTraits>::
NumEdgesMetricTest() : NumEdgesMetric<MPTraits>() {}

template <typename MPTraits>
NumEdgesMetricTest<MPTraits>::
NumEdgesMetricTest(XMLNode& _node) : MetricMethod<MPTraits>(_node),
                                           NumEdgesMetric<MPTraits>(_node) {}
                                           
template <typename MPTraits>
NumEdgesMetricTest<MPTraits>::
~NumEdgesMetricTest() {}



/*--------------------- Test Interface Functions ---------------------*/
template <typename MPTraits>
typename NumEdgesMetricTest<MPTraits>::TestResult
NumEdgesMetricTest<MPTraits>::
TestMetric() {

  // Set up environment from parent
  double metric = (*this)();
  double expected = this->GetGroupTask() ? (this->GetGroupRoadmap()->get_num_edges()) : (this->GetRoadmap()->get_num_edges());

  // Correct value?
  if(metric == expected){
    return std::make_pair(true,"NumEdgesMetric::PASSED");
  }
  return std::make_pair(false,"NumEdgesMetric::FAILED, Wrong number of edges recieved.");
}

#endif
