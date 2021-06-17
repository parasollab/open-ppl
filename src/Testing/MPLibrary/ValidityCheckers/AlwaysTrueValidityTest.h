#ifndef PPL_ALWAYS_TRUE_VALIDITY_TEST_H_
#define PPL_ALWAYS_TRUE_VALIDITY_TEST_H_

#include "ValidityCheckerMethodTest.h"
#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"

template <typename MPTraits>
class AlwaysTrueValidityTest : public ValidityCheckerMethodTest<MPTraits>,
                               public AlwaysTrueValidity<MPTraits> {

  public: 
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    AlwaysTrueValidityTest();

    AlwaysTrueValidityTest(XMLNode& _node);

    ~AlwaysTrueValidityTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult IndividualCfgValidityTest() override;

    virtual TestResult GroupCfgValidityTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template<typename MPTraits>
AlwaysTrueValidityTest<MPTraits>::
AlwaysTrueValidityTest() : AlwaysTrueValidity<MPTraits>() {}

template<typename MPTraits>
AlwaysTrueValidityTest<MPTraits>:: 
AlwaysTrueValidityTest(XMLNode& _node) : AlwaysTrueValidity<MPTraits>(_node) {}

template<typename MPTraits>
AlwaysTrueValidityTest<MPTraits>::
~AlwaysTrueValidityTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename AlwaysTrueValidityTest<MPTraits>::TestResult
AlwaysTrueValidityTest<MPTraits>::
IndividualCfgValidityTest() {
  
  bool passed = true;
  std::string message = "";

  auto output = this->IndividualCfgValidity();

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
typename AlwaysTrueValidityTest<MPTraits>::TestResult
AlwaysTrueValidityTest<MPTraits>::
GroupCfgValidityTest() {

  bool passed = true;
  std::string message = "";

  auto output = this->IndividualCfgValidity();

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

/*--------------------------------------------------------------------*/
#endif
