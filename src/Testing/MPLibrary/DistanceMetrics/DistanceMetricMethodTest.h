#ifndef PPL_DISTANCE_METRIC_METHOD_TEST_H_
#define PPL_DISTANCE_METRIC_METHOD_TEST_H_

#include "MPLibrary/DistanceMetrics/DistanceMetricMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class DistanceMetricMethodTest : public DistanceMetricMethod<MPTraits>,
                                 public TestBaseObject {

  public: 

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    DistanceMetricMethodTest();

    ~DistanceMetricMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestIndividualCfgDistance() = 0;

    virtual TestResult TestIndividualEdgeWeight() = 0;

    virtual TestResult TestIndividualScaleCfg() = 0;

    virtual TestResult TestGroupCfgDistance() = 0;

    virtual TestResult TestGroupEdgeWeight() = 0;

    virtual TestResult TestGroupScaleCfg() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    virtual double IndividualCfgDistance();

    virtual double IndividualEdgeWeight();

    virtual CfgType IndividualScaleCfg();

    virtual double GroupCfgDistance();

    virtual double GroupEdgeWeight();

    virtual GroupCfgType GroupScaleCfg();

    ///@}
    ///@name Helper Functions
    ///@{
  
    CfgType GetIndividualCfg();

    GroupCfgType GetGroupCfg();

    ///@}

};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
DistanceMetricMethodTest<MPTraits>::
DistanceMetricMethodTest() {}

template <typename MPTraits>
DistanceMetricMethodTest<MPTraits>::
~DistanceMetricMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename DistanceMetricMethodTest<MPTraits>::TestResult
DistanceMetricMethodTest<MPTraits>::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result = TestIndividualCfgDistance();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualEdgeWeight();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualScaleCfg();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupCfgDistance();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupEdgeWeight();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupScaleCfg();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message); 
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
IndividualCfgDistance() {
  auto cfg1 = GetIndividualCfg();
  auto cfg2 = GetIndividualCfg();
  
  for(size_t i = 0; i < cfg2.PosDOF(); i++) {
    cfg2[i] = 5;
  }

  for(size_t i = cfg2.PosDOF(); i < cfg2.DOF(); i++) {
    cfg2[i+cfg2.PosDOF()] = .5;
  }

  return this->Distance(cfg1,cfg2);
}

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
IndividualEdgeWeight() {
  //TODO::Implement this default function.
  return 0;
}

template <typename MPTraits>
typename MPTraits::CfgType
DistanceMetricMethodTest<MPTraits>::
IndividualScaleCfg() {
  //TODO::Implement this default function.
  return GetIndividualCfg();
}

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
GroupCfgDistance() {
  //TODO::Implement this default function.
  return 0;
}

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
GroupEdgeWeight() {
  //TODO::Implement this default function.
  return 0;
}

template <typename MPTraits>
typename MPTraits::GroupCfgType
DistanceMetricMethodTest<MPTraits>::
GroupScaleCfg() {
  //TODO::Implement this default function.
  return GetGroupCfg();
}

/*-------------------------- Helper Functions ------------------------*/
  
template <typename MPTraits>
typename MPTraits::CfgType
DistanceMetricMethodTest<MPTraits>::
GetIndividualCfg() {
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  CfgType cfg(robot);
  return cfg;
}

template <typename MPTraits>
typename MPTraits::GroupCfgType
DistanceMetricMethodTest<MPTraits>::
GetGroupCfg() {
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  GroupCfgType gcfg(group);
  return gcfg;
}

/*--------------------------------------------------------------------*/

#endif
