#ifndef PPL_TERRAIN_VALIDITY_CHECKER_TEST_H_
#define PPL_TERRAIN_VALIDITY_CHECKER_TEST_H_

#include "ValidityCheckerMethodTest.h"
#include "MPLibrary/ValidityCheckers/TerrainValidityChecker.h"

template <typename MPTraits>
class TerrainValidityCheckerTest : virtual public TerrainValidityChecker<MPTraits>,
                                   public ValidityCheckerMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    
    ///@}
    ///@name Construction
    ///@{

    TerrainValidityCheckerTest();

    TerrainValidityCheckerTest(XMLNode& _node);

    ~TerrainValidityCheckerTest();

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

    template<typename T, typename U> friend class MethodSet;
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
TerrainValidityCheckerTest<MPTraits>::
TerrainValidityCheckerTest() : TerrainValidityChecker<MPTraits>() {}

template <typename MPTraits>
TerrainValidityCheckerTest<MPTraits>::
TerrainValidityCheckerTest(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node),
                                             TerrainValidityChecker<MPTraits>(_node) {}

template <typename MPTraits>
TerrainValidityCheckerTest<MPTraits>::
~TerrainValidityCheckerTest() {}

/*---------------------------- Interface -----------------------------*/

template <typename MPTraits>
bool
TerrainValidityCheckerTest<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  return TerrainValidityChecker<MPTraits>::IsValidImpl(_cfg, _cdInfo, _callName);
}

/*--------------------- Test Interface Functions ---------------------*/

template <typename MPTraits>
typename TerrainValidityCheckerTest<MPTraits>::TestResult
TerrainValidityCheckerTest<MPTraits>::
IndividualCfgValidityTest() {

  bool passed = true;
  std::string message = "";

  // auto output = this->IndividualCfgValidity();
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto env = this->GetEnvironment();

  CfgType cfg(robot);

  for(auto& terrainTypes : env->GetTerrains()) {
    auto terrainCapability = terrainTypes.first;
    auto& terrain = terrainTypes.second[0];
    auto terrainBoundary = terrain.GetBoundaries()[0].get();
    if(terrainCapability == "a") {
      // Sample a point in the terrain boundary
      cfg.GetRandomCfg(terrainBoundary);

      // Check wrong capability in terrain
      robot->SetCapability("b");
      passed = passed and !this->IsValid(cfg, "TerrainValidityCheckerTest");

      // Check correct capability in terrain
      robot->SetCapability("a");
      passed = passed and this->IsValid(cfg, "TerrainValidityCheckerTest");

      // Check correct capability out of terrain
      std::vector<double> dofs = {20.0, 10.0, 20.0, 0.0, 0.0, 0.0};
      cfg.SetData(dofs);
      passed = passed and !this->IsValid(cfg, "TerrainValidityCheckerTest");

    } else {
      /// ???
      std::cout << "\n\n !!Should never get here!! \n\n" << std::endl;
      passed = false;
    }
  }

  if(passed) {
    message = "IndividualCfgValidity::PASSED!\n";
  }
  else {
    message = "IndividualCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);

}

template <typename MPTraits>
typename TerrainValidityCheckerTest<MPTraits>::TestResult
TerrainValidityCheckerTest<MPTraits>::
GroupCfgValidityTest() {
  bool passed = true;
  std::string message = "";
  
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