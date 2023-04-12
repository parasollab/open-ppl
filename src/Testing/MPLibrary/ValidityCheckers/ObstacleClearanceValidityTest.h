#ifndef PPL_OBSTACLE_CLEARANCE_VALIDITY_TEST_H_
#define PPL_OBSTACLE_CLEARANCE_VALIDITY_TEST_H_

#include "ValidityCheckerMethodTest.h"
#include "MPLibrary/ValidityCheckers/ObstacleClearanceValidity.h"

template <typename MPTraits>
class ObstacleClearanceValidityTest : virtual public ObstacleClearanceValidity<MPTraits>,
                               public ValidityCheckerMethodTest<MPTraits> {

  public: 
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    ObstacleClearanceValidityTest();

    ObstacleClearanceValidityTest(XMLNode& _node);

    ~ObstacleClearanceValidityTest();

    ///@}
    ///@name Interface 
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    virtual std::vector<std::pair<bool,CfgType>> IndividualCfgValidity() override;
    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult IndividualCfgValidityTest() override;

    virtual TestResult GroupCfgValidityTest() override;

    ///@}

    //using ObstacleClearanceValidity<MPTraits>::m_name;
    template<typename T, typename U> friend class MethodSet;

};

/*--------------------------- Construction ---------------------------*/

template<typename MPTraits>
ObstacleClearanceValidityTest<MPTraits>::
ObstacleClearanceValidityTest() : ObstacleClearanceValidity<MPTraits>() {}

template<typename MPTraits>
ObstacleClearanceValidityTest<MPTraits>:: 
ObstacleClearanceValidityTest(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node),
                                         ObstacleClearanceValidity<MPTraits>(_node) {
}

template<typename MPTraits>
ObstacleClearanceValidityTest<MPTraits>::
~ObstacleClearanceValidityTest() {}

/*---------------------------- Interface -----------------------------*/

template <typename MPTraits>
bool
ObstacleClearanceValidityTest<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  return ObstacleClearanceValidity<MPTraits>::IsValidImpl(_cfg,_cdInfo,_callName);
}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename ObstacleClearanceValidityTest<MPTraits>::TestResult
ObstacleClearanceValidityTest<MPTraits>::
IndividualCfgValidityTest() {
  
  bool passed = true;
  std::string message = "";

  auto output = IndividualCfgValidity();

  // Make sure that every response is true.
  for(auto kv : output) {
    if(kv.first)
      continue;

    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled "
              "invalid.\n";
    break;
  }

  if(passed) {
    message = "IndividualCfgValidity::PASSED!\n";
  }
  else {
    message = "IndividualCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template<typename MPTraits>
typename ObstacleClearanceValidityTest<MPTraits>::TestResult
ObstacleClearanceValidityTest<MPTraits>::
GroupCfgValidityTest() {

  bool passed = true;
  std::string message = "";

  auto output = IndividualCfgValidity();

  // Make sure that every response is true.
  for(auto kv : output) {
    if(kv.first)
      continue;

    passed = false;
    message = message + "\n\tA group cfg was incorrectly labeled "
              "invalid.\n";
    break;
  }

  if(passed) {
    message = "GroupCfgValidity::PASSED!\n";
  }
  else {
    message = "GroupCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

/*-------------------------- ValidityCheckerMethod Overrides --------------------------*/

template <typename MPTraits>
std::vector<std::pair<bool,typename MPTraits::CfgType>>
ObstacleClearanceValidityTest<MPTraits>::
IndividualCfgValidity() {

  // Set the library for a single robot.
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto task = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(task.get());
  this->GetMPLibrary()->SetGroupTask(nullptr);

  std::vector<std::pair<bool,CfgType>> output;

  // Configuration Test 1 (Should Fail)
  auto p1 = CfgType(robot);
  std::istringstream p1Stream(" 0 0 0 0 0 0 0");
  p1.Read(p1Stream);
  bool valid = this->IsValid(p1,"Test");
  output.push_back(std::make_pair(!valid,p1));

  // Configuration Test 2 (Should Pass)
  auto p2 = CfgType(robot);
  std::istringstream p2Stream("-5 6 -5 0 0 0 0");
  p2.Read(p2Stream);
  bool valid2 = this->IsValid(p2,"Test");
  output.push_back(std::make_pair(valid2,p2));

  // Configuration Test 3 (Should Pass)
  auto p3 = CfgType(robot);
  std::istringstream p3Stream("5 -6 5 0 0 0 0");
  p3.Read(p3Stream);
  bool valid3 = this->IsValid(p3,"Test");
  output.push_back(std::make_pair(valid3,p3));

  // Configuration Test 4 (Should Pass)
  auto p4 = CfgType(robot);
  std::istringstream p4Stream("7 -6 -7 0 0 0 0");
  p4.Read(p4Stream);
  bool valid4 = this->IsValid(p4,"Test");
  output.push_back(std::make_pair(valid4,p4));

  // Configuration Test 5 (Should Fail)
  auto p5 = CfgType(robot);
  std::istringstream p5Stream("3 -4 3 0 0 0 0");
  p5.Read(p5Stream);
  bool valid5 = this->IsValid(p5,"Test");
  output.push_back(std::make_pair(!valid5,p5));

  // Configuration Test 6 (Should Fail)
  auto p6 = CfgType(robot);
  std::istringstream p6Stream("-2 4 -2 0 0 0 0");
  p6.Read(p6Stream);
  bool valid6 = this->IsValid(p6,"Test");
  output.push_back(std::make_pair(!valid6,p6));

  return output;
}

#endif
