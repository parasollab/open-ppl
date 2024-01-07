#ifndef PPL_TOPOLOGICAL_MAP_VALIDITY_TEST_H_
#define PPL_TOPOLOGICAL_MAP_VALIDITY_TEST_H_

#include "MPLibrary/ValidityCheckers/TopologicalMapValidity.h"
#include "ValidityCheckerMethodTest.h"

#include "MPLibrary/MPTools/TetGenDecomposition.h"

/**
 * Read in a topological map form 3d env
 * Use that to construct a topological map validilty chekcer
 * Sample two points, one that's valid and one that's not
 * Check whether the validity checker returns the right output for both of the points
 **/

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

};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
TopologicalMapValidityTest<MPTraits>::
TopologicalMapValidityTest() : TopologicalMapValidity<MPTraits>() {}

template <typename MPTraits>
TopologicalMapValidityTest<MPTraits>::
TopologicalMapValidityTest(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node), 
                                             TopologicalMapValidity<MPTraits>(_node) {}

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

  // Making workplace decomposition
  auto decomposer = new TetGenDecomposition();
  auto environment = this->GetMPProblem()->GetEnvironment();
  auto workplaceDecomposition = (*decomposer)(environment);
  delete decomposer;
  decomposer = nullptr;
  this->GetMPTools()->SetDecomposition("coarseDecomposition", workplaceDecomposition);

  // Making topological map
  auto topologicalMap = new TopologicalMap<MPTraits>(1, "coarseDecomposition");
  this->GetMPTools()->SetTopologicalMap("topologicalMap", topologicalMap);
  this->GetMPTools()->GetTopologicalMap("topologicalMap")->Initialize();
  this->m_tmLabel = "topologicalMap";

  // Getting robot to position
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto task = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(task.get());
  this->GetMPLibrary()->SetGroupTask(nullptr);

  bool passed = true;
  std::string message;
  vector<bool> output;

  // Valid configuration
  auto p1 = CfgType(robot);
  std::istringstream p1Stream("0 0 0 0 0 0 0");
  p1.Read(p1Stream);
  output.push_back((this->IsValid(p1, "Test")));

  // Invalid configuration
  auto p2 = CfgType(robot);
  std::istringstream p2Stream("-2 4 -2 0 0 0 0");
  p2.Read(p2Stream);
  output.push_back(!(this->IsValid(p2, "Test")));

  // Ensure all points sampled are correct
  for (auto kv : output) {
    if (kv)
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
  std::string message = "GroupCfgValidity not implemented for this test\n";
  return std::make_pair(passed, message);
}
/*--------------------------------------------------------------------*/
#endif
