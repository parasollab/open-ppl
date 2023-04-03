#ifndef PPL_TOPOLOGICAL_MAP_VALIDITY_TEST_H_
#define PPL_TOPOLOGICAL_MAP_VALIDITY_TEST_H_

#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#include "MPLibrary/ValidityCheckers/TopologicalMapValidity.h"
#include "ValidityCheckerMethodTest.h"

template <typename MPTraits>
class TopologicalMapValidityTest : virtual public TopologicalMapValidity<MPTraits>,
                                   virtual public ValidityCheckerMethodTest<MPTraits> {
 public:
  ///@name Local Types
  ///@{

  typedef TestBaseObject::TestResult TestResult;

  typedef typename MPTraits::CfgType CfgType;

  ///@}
  ///@name Construction
  ///@{

  TopologicalMapValidityTest();

  TopologicalMapValidityTest(XMLNode& _node);

  ~TopologicalMapValidityTest();

  ///@}
  ///@name Interface
  ///@{

  virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
                           const std::string& _callName) override;

  ///@}

 private:
  ///@name Test Interface Functions
  ///@{

  virtual TestResult IndividualCfgValidityTest() override;

  virtual TestResult GroupCfgValidityTest() override;

  ///@}

  // using AlwaysTrueValidity<MPTraits>::m_name;
  template <typename T, typename U>
  friend class MethodSet;
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
TopologicalMapValidityTest<MPTraits>::
    TopologicalMapValidityTest() : TopologicalMapValidity<MPTraits>() {}

template <typename MPTraits>
TopologicalMapValidityTest<MPTraits>::
    TopologicalMapValidityTest(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node),
                                                 TopologicalMapValidity<MPTraits>(_node) {
}

template <typename MPTraits>
TopologicalMapValidityTest<MPTraits>::
    ~TopologicalMapValidityTest() {}

/*---------------------------- Interface -----------------------------*/

template <typename MPTraits>
bool TopologicalMapValidityTest<MPTraits>::
    IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  return TopologicalMapValidity<MPTraits>::IsValidImpl(_cfg, _cdInfo, _callName);
}

/*--------------------- Test Interface Functions ---------------------*/

template <typename MPTraits>
typename TopologicalMapValidityTest<MPTraits>::TestResult
TopologicalMapValidityTest<MPTraits>::
    IndividualCfgValidityTest() {
  bool passed = true;
  std::string message = "";

  auto output = this->IndividualCfgValidity();

  // Make sure that every response is true.
  for (auto kv : output) {
    if (kv.first)
      continue;

    passed = false;
    message = message +
              "\n\tA cfg was incorrectly labeled "
              "invalid.\n";
    break;
  }

  if (passed) {
    message = "IndividualCfgValidity::PASSED!\n";
  } else {
    message = "IndividualCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed, message);
}

template <typename MPTraits>
typename TopologicalMapValidityTest<MPTraits>::TestResult
TopologicalMapValidityTest<MPTraits>::
    GroupCfgValidityTest() {
  bool passed = true;
  std::string message = "";

  auto output = this->IndividualCfgValidity();

  // Make sure that every response is true.
  for (auto kv : output) {
    if (kv.first)
      continue;

    passed = false;
    message = message +
              "\n\tA group cfg was incorrectly labeled "
              "invalid.\n";
    break;
  }

  if (passed) {
    message = "GroupCfgValidity::PASSED!\n";
  } else {
    message = "GroupCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
