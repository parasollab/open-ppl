#ifndef PPL_HIERARCHICALLP_TEST_H_
#define PPL_HIERARCHICALLP_TEST_H_

#include "Testing/MPLibrary/LocalPlanners/LocalPlannerMethodTest.h"
#include "MPLibrary/LocalPlanners/HierarchicalLP.h"

template <typename MPTraits>
class HierarchicalLPTest : virtual public HierarchicalLP<MPTraits>,
                         public LocalPlannerMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult      TestResult;

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    HierarchicalLPTest();

    HierarchicalLPTest(XMLNode& _node);

    ~HierarchicalLPTest();

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult IndividualRobotIsConnectedTest() override;

    virtual TestResult IndividualRobotBlindPathTest() override;

    virtual TestResult RobotGroupIsConnectedTest() override;

    virtual TestResult RobotGroupBlindPathTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
HierarchicalLPTest<MPTraits>::
HierarchicalLPTest() : HierarchicalLP<MPTraits>() {}

template <typename MPTraits>
HierarchicalLPTest<MPTraits>::
HierarchicalLPTest(XMLNode& _node) : LocalPlannerMethod<MPTraits>(_node),
                                   HierarchicalLP<MPTraits>(_node) {}

template <typename MPTraits>
HierarchicalLPTest<MPTraits>::
~HierarchicalLPTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename HierarchicalLPTest<MPTraits>::TestResult
HierarchicalLPTest<MPTraits>::
IndividualRobotIsConnectedTest() {
  bool passed = true;
  std::string message = "";

  this->SetLibraryRobot();

  // Test two configurations that are connectable using simply sl
  const CfgType c1 = this->GetIndividualCfg();

  CfgType c = this->GetIndividualCfg();
  c[0] += 10.0;
  const CfgType c2 = CfgType(c);

  CfgType col;
  LPOutput<MPTraits> lpOutput;

  auto stats = this->GetStatClass();
  
  stats->ClearStats();

  this->IndividualRobotIsConnected(c1, c2, col, &lpOutput);

  std::map<std::string, std::tuple<size_t, size_t, size_t>> LPMap = stats->m_lpInfo;
  bool result = (LPMap.find("StraightLine::sl") != LPMap.end()) && (LPMap.find("StraighLine::slAlwaysTrue") == LPMap.end());

  if (!result) {
    passed = false;
    message = message + "\n\tLocal planner was not acting as expected\n";
  }

  // Test two configurations that are connectable using sl then slAlwaysTrue
  c[0] = 0.0;
  c[1] = 15.0;
  const CfgType c3 = CfgType(c);

  stats->ClearStats();

  this->IndividualRobotIsConnected(c1, c3, col, &lpOutput);

  LPMap = stats->m_lpInfo;

  result = (LPMap.find("StraightLine::sl") != LPMap.end()) && (LPMap.find("StraightLine::slAlwaysTrue") != LPMap.end());

  if (!result) {
    passed = false;
    message = message + "\n\tLocal planner did not use expected local planners\n";
  }

  if (passed) {
    message = "IndividualRobotIsConnected::PASSED!\n";
  } else {
    message = "IndividualRobotIsConnected::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename HierarchicalLPTest<MPTraits>::TestResult
HierarchicalLPTest<MPTraits>::
IndividualRobotBlindPathTest() {
  bool passed = true;
  std::string message = "";

  // Check that a non-empty path is returned
  auto path = this->IndividualRobotBlindPath();
  bool result = (path.size() > 0);

  // Check that the path is a straight line along both segments
  if (result) {
    auto cfg = path[0];
    auto newCfg = path[1];
    std::size_t idx = 1;

    // Check that the first path segment contains configurations that are
    // constantly increasing in the x direction (and no other direction)
    while (passed and idx+1 < path.size() and fabs(newCfg[0] - 10.0) > 1e-7) {
      if (cfg[0] >= newCfg[0] or fabs(newCfg[1]) > 1e-7 
          or fabs(newCfg[2]) > 1e-7) {
        passed = false;
        break;
      }

      idx++;
      cfg = newCfg;
      newCfg = path[idx];
    }

    // Skip the endpoint of the first line segment
    idx++;
    cfg = newCfg;
    newCfg = path[idx];

    // Check that the second path segment contains configurations that are 
    // constantly increasing in the y direction (and no other direction)
    while (passed and idx+1 < path.size() and fabs(newCfg[1] - 10.0) > 1e-7) {
      if (cfg[1] >= newCfg[1] or fabs(newCfg[0] - 10.0) > 1e-7 
          or fabs(newCfg[2]) > 1e-7) {
        passed = false;
        break;
      }

      idx++;
      cfg = newCfg;
      newCfg = path[idx];
    }
  }

  if (!passed) {
    message = message + "\n\tLocal planner failed to find the correct blind path.\n";
  }

  if (passed) {
    message = "IndividualRobotBlindPath::PASSED!\n";
  } else {
    message = "IndividualRobotBlindPath::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename HierarchicalLPTest<MPTraits>::TestResult
HierarchicalLPTest<MPTraits>::
RobotGroupIsConnectedTest() {
  bool passed = true;
  std::string message = "";

  // TODO Need support for GroupCfgs

  if (passed) {
    message = "RobotGroupIsConnected::PASSED!\n";
  } else {
    message = "RobotGroupIsConnected::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename HierarchicalLPTest<MPTraits>::TestResult
HierarchicalLPTest<MPTraits>::
RobotGroupBlindPathTest() {
  bool passed = true;
  std::string message = "";

  // TODO Need support for GroupCfgs

  if (passed) {
    message = "RobotGroupIsConnected::PASSED!\n";
  } else {
    message = "RobotGroupIsConnected::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
