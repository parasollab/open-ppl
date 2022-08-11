#ifndef PPL_CONNECTION_METHOD_TEST_H_
#define PPL_CONNECTION_METHOD_TEST_H_

#include "MPLibrary/Connectors/ConnectorMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class ConnectorMethodTest : virtual public ConnectorMethod<MPTraits>,
                            public TestBaseObject {
  public: 
    
    ///@name Local Types
    ///@{
   
    typedef TestBaseObject::TestResult TestResult; 

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{
    
    ConnectorMethodTest();

    ~ConnectorMethodTest();
    
    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest();    
    
    ///@}
  
  protected:

    ///@name Interface Test Functions
    ///@{
    
    /// @TODO: Should the name be TestIndividualRobotConnect to be consistent with UniformSampler example?
    virtual TestResult IndividualRobotConnectTest() = 0;
  
    /// @TODO: Should the name be TestRobotGroupConnect to be consistent with UniformSampler example?

    virtual TestResult RobotGroupConnectTest() = 0;
 
    ///@}
    ///@name Default Function Calls
    ///@{
    
    // Unfortunately, these are going to be specific to the 
    // method, so you'll have to make your test from scratch.
 
    ///@}
    ///@name Helper Functions
    ///@{
   
    CfgType GetIndividualCfg();

    GroupCfgType GetGroupCfg();
 
    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
ConnectorMethodTest<MPTraits>::
ConnectorMethodTest() { }

template <typename MPTraits>
ConnectorMethodTest<MPTraits>::
~ConnectorMethodTest() { }

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename ConnectorMethodTest<MPTraits>::TestResult
ConnectorMethodTest<MPTraits>::
RunTest() {
  
  bool passed = true;
  std::string message = "";

  /// @TODO: Should the name be TestIndividualRobotConnect to be consistent with UniformSampler example?
  auto result = IndividualRobotConnectTest();
  passed = passed and result.first;
  message = message + result.second;

  result = RobotGroupConnectTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/
/*-------------------------- Helper Functions ------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
ConnectorMethodTest<MPTraits>::
GetIndividualCfg() {
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  CfgType cfg(robot);
  return cfg;
}

template <typename MPTraits>
typename MPTraits::GroupCfgType
ConnectorMethodTest<MPTraits>::
GetGroupCfg() {
  // @TODO: this code is broken.
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  auto groupRoadmap = this->GetMPLibrary()->GetGroupRoadmap(group);
  GroupCfgType gcfg(groupRoadmap);
  return gcfg;
}

/*--------------------------------------------------------------------*/
#endif
