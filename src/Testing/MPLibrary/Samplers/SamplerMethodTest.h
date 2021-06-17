#ifndef PPL_SAMPLER_METHOD_TEST_H_
#define PPL_SAMPLER_METHOD_TEST_H_

#include "MPLibrary/Samplers/SamplerMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class SamplerMethodTest : public SamplerMethod<MPTraits>, 
                          public TestBaseObject {
  public:
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    SamplerMethodTest();

    ~SamplerMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  private:
    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestIndividualCfgSample() = 0;

    virtual TestResult TestIndividualCfgSampleWithEEConstraint() = 0;

    virtual TestResult TestIndividualFilter() = 0;

    virtual TestResult TestGroupCfgSampleSingleBoundary() = 0;

    virtual TestResult TestGroupCfgSampleIndividualBoundaries() = 0;

    virtual TestResult TestGroupFilter() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    //TODO::Determine what the proper return type is as these functions
    //      get implemented. Each one should call the corresponding interface
    //      function in the SamplerMethod class and return the same output.
    //      The function should instantiate whatever input variables are 
    //      needed to test the underlying SamplerMethod function.

    virtual void IndividualCfgSample();

    virtual void IndividualCfgSampleWithEEConstraint();

    virtual void IndividualFilter();

    virtual void GroupCfgSampleSingleBoundary();

    virtual void GroupCfgSampleIndividualBoundaries();

    virtual void GroupFilter();

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
SamplerMethodTest<MPTraits>::
SamplerMethodTest() : SamplerMethod<MPTraits>() {}

template <typename MPTraits>
SamplerMethodTest<MPTraits>::
~SamplerMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename SamplerMethodTest<MPTraits>::TestResult
SamplerMethodTest<MPTraits>::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result = TestIndividualCfgSample();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualCfgSampleWithEEConstraint();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualFilter();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupCfgSampleSingleBoundary();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupCfgSampleIndividualBoundaries();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupFilter();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
IndividualCfgSample() {

  // Set MPLibrary to sample for single robot
  auto robot = this->GetMPProblem()->GetRobots()[0];
  auto task = this->GetMPProblem()->GetTask(robot);
  this->GetMPLibrary()->SetTask(task);

  //TODO::Finish test and decide function outputs. 
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
IndividualCfgSampleWithEEConstraint() {

  // Set MPLibrary to sample for single robot
  auto robot = this->GetMPProblem()->GetRobots()[0];
  auto task = this->GetMPProblem()->GetTask(robot);
  this->GetMPLibrary()->SetTask(task);

  //TODO::Finish test and decide function outputs. 
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
IndividualFilter() {

  // Set MPLibrary to sample for single robot
  auto robot = this->GetMPProblem()->GetRobots()[0];
  auto task = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(task);

  //TODO::Finish test and decide function outputs. 
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
GroupCfgSampleSingleBoundary() {

  // Set MPLibrary to sample for robot group
  auto group = this->GetMPProblem()->GetRobotGroups()[0];
  auto task = this->GetMPProblem()->GetTasks(group)[0];
  this->GetMPLibrary()->SetGroupTask(task);

  //TODO::Finish test and decide function outputs. 
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
GroupCfgSampleIndividualBoundaries() {

  // Set MPLibrary to sample for robot group
  auto group = this->GetMPProblem()->GetRobotGroups()[0];
  auto task = this->GetMPProblem()->GetTasks(group)[0];
  this->GetMPLibrary()->SetGroupTask(task);

  //TODO::Finish test and decide function outputs. 
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
GroupFilter() {

  // Set MPLibrary to sample for robot group
  auto group = this->GetMPProblem()->GetRobotGroups()[0];
  auto task = this->GetMPProblem()->GetTasks(group)[0];
  this->GetMPLibrary()->SetGroupTask(task);

  //TODO::Finish test and decide function outputs. 
}

/*--------------------------------------------------------------------*/
#endif
