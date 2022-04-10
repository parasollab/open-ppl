#ifndef PPL_METRIC_METHOD_TEST_H_
#define PPL_METRIC_METHOD_TEST_H_

#include "MPLibrary/Metrics/MetricMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class MetricMethodTest : virtual public MetricMethod<MPTraits>,
                         public TestBaseObject {

  public: 

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    MetricMethodTest();

    ~MetricMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    // Each Metric runs a different measurement
    virtual TestResult TestMetric() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    virtual double Metric();

    ///@}
    ///@name Helper Functions
    ///@{
  
    CfgType GetIndividualCfg();

    ///@}

};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MetricMethodTest<MPTraits>::
MetricMethodTest() {}

template <typename MPTraits>
MetricMethodTest<MPTraits>::
~MetricMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename MetricMethodTest<MPTraits>::TestResult
MetricMethodTest<MPTraits>::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result = TestMetric();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message); 
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
double
MetricMethodTest<MPTraits>::
Metric() {

  // Initialize roadmap for test
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  RoadmapType roadmap(robot);

  // Get cfgs for edge
  auto cfg1 = GetIndividualCfg();
  auto cfg2 = GetIndividualCfg();
  
  for(size_t i = 0; i < cfg2.PosDOF(); i++) {
    cfg2[i] = 5;
  }

  for(size_t i = cfg2.PosDOF(); i < cfg2.DOF(); i++) {
    cfg2[i+cfg2.PosDOF()] = .5;
  }

  // Add cfgs to roadmap
  auto first = roadmap.AddVertex(cfg1);
  auto second = roadmap.AddVertex(cfg2);

  // Add edge
  DefaultWeight<CfgType> weight;
  roadmap.AddEdge(first,second,weight);

  // There's no one default test scenario that applies to all metrics
  // All metrics test different measurements
  return (*this)();
}

/*----------------------- Helper Functions ---------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
MetricMethodTest<MPTraits>::
GetIndividualCfg() {
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  CfgType cfg(robot);
  return cfg;
}

#endif
