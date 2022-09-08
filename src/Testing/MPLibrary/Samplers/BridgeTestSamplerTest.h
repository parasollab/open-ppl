#ifndef PPL_BRIDGE_TEST_SAMPLER_TEST_H
#define PPL_BRIDGE_TEST_SAMPLER_TEST_H

#include "MPLibrary/Samplers/BridgeTestSampler.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

template <class MPTraits>
class BridgeTestSamplerTest : virtual public BridgeTestSampler<MPTraits>, 
                              public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    BridgeTestSamplerTest();

    BridgeTestSamplerTest(XMLNode& _node);

    ~BridgeTestSamplerTest();

    ///@}

  private: 

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestIndividualCfgSample() override;

    virtual TestResult TestIndividualCfgSampleWithEEConstraint() override;

    virtual TestResult TestIndividualFilter() override;

    virtual TestResult TestGroupCfgSampleSingleBoundary() override;

    virtual TestResult TestGroupCfgSampleIndividualBoundaries() override;

    virtual TestResult TestGroupFilter() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
BridgeTestSamplerTest<MPTraits>::
BridgeTestSamplerTest() : BridgeTestSampler<MPTraits>() {}

template <typename MPTraits>
BridgeTestSamplerTest<MPTraits>::
BridgeTestSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                        BridgeTestSampler<MPTraits>(_node) {}

template <typename MPTraits>
BridgeTestSamplerTest<MPTraits>::
~BridgeTestSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename BridgeTestSamplerTest<MPTraits>::TestResult
BridgeTestSamplerTest<MPTraits>::
TestIndividualCfgSample() {
  // Make sure that all "valids" are valid and inside the boundary 
  // and all "invalids" are invalid or outside the boundary.

  ///@todo Make sure the returned cfgs are actually on a bridge - this is 
  /// difficult to do without changing the sampler interface

  bool passed = true;
  std::string message = "";

  Boundary* boundary = this->GetEnvironment()->GetBoundary();
  std::vector<Cfg> valids;
  std::vector<Cfg> invalids;

  this->IndividualCfgSample(boundary, valids, invalids);

  auto vc = this->GetValidityChecker(this->m_vcLabel);
  std::string callee(this->GetNameAndLabel() + "::SamplerTest");
  for(auto cfg : valids) {
    if(vc->IsValid(cfg, callee) and !(this->m_useBoundary and !cfg.InBounds(boundary)))   
      continue;

    passed = false;
    message = message + "\n\tA configuration was incorrectly labeled "
              "valid for the given boundary.\n";
    break;
  }

  if(passed) {
    message = "IndividualCfgSample::PASSED!\n";
  }
  else {
    message = "IndividualCfgSample::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}


template <typename MPTraits>
typename BridgeTestSamplerTest<MPTraits>::TestResult
BridgeTestSamplerTest<MPTraits>::
TestIndividualCfgSampleWithEEConstraint() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->IndividualCfgSampleWithEEConstraint();

  if(passed) {
    message = "IndividualCfgSampleWithEEConstraint::PASSED!\n";
  }
  else {
    message = "IndividualCfgSampleWithEEConstraint::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}


template <typename MPTraits>
typename BridgeTestSamplerTest<MPTraits>::TestResult
BridgeTestSamplerTest<MPTraits>::
TestIndividualFilter() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->IndividualFilter();

  if(passed) {
    message = "IndividualFilter::PASSED!\n";
  }
  else {
    message = "IndividualFilter::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}


template <typename MPTraits>
typename BridgeTestSamplerTest<MPTraits>::TestResult
BridgeTestSamplerTest<MPTraits>::
TestGroupCfgSampleSingleBoundary() {
  // Make sure that all "valids" are valid and inside the boundary 
  // and all "invalids" are invalid or outside the boundary.

  ///@todo Make sure the returned cfgs are actually on a bridge - this is 
  /// difficult to do without changing the sampler interface

  bool passed = true;
  std::string message = "";

  Boundary* boundary = nullptr;
  std::vector<GroupCfgType> valids;
  std::vector<GroupCfgType> invalids;
  
  this->GroupCfgSampleSingleBoundary(boundary, valids, invalids);

  auto vc = this->GetValidityChecker(this->m_vcLabel);
  std::string callee(this->GetNameAndLabel() + "::SamplerTest");
  for(auto gcfg : valids) {
    if(vc->IsValid(gcfg, callee) and !(this->m_useBoundary and !gcfg.InBounds(boundary)))   
      continue;

    passed = false;
    message = message + "\n\tA group configuration was incorrectly labeled "
            "valid for the given boundary.\n";
    break;
  }

  if(passed) {
    message = "GroupCfgSampleSingleBoundary::PASSED!\n";
  }
  else {
    message = "GroupCfgSampleSingleBoundary::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}


template <typename MPTraits>
typename BridgeTestSamplerTest<MPTraits>::TestResult
BridgeTestSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {
  // Tests group robots with individual boundaries to make sure "valids" are in 
  // boundary and valid, and that "invalids" are not

  ///@todo Make sure the returned cfgs are actually on a bridge - this is 
  /// difficult to do without changing the sampler interface

  bool passed = true;
  std::string message = "";

  std::map<Robot*,const Boundary*> boundaryMap;
  for(auto robot : this->GetGroupRoadmap()->GetGroup()->GetRobots()) {
    boundaryMap.emplace(std::make_pair(robot, this->GetEnvironment()->GetBoundary()));
  }
  std::vector<GroupCfgType> valids;
  std::vector<GroupCfgType> invalids;
  
  this->GroupCfgSampleIndividualBoundaries(boundaryMap, valids, invalids);

  auto vc = this->GetValidityChecker(this->m_vcLabel);
  std::string callee(this->GetNameAndLabel() + "::SamplerTest");
  for(auto gcfg : valids) {
    if(!vc->IsValid(gcfg, callee)) {
      passed = false;
      message = message + "\n\tA group configuration was incorrectly labeled "
            "valid for the given boundary.\n";
      break;
    }

    if(this->m_useBoundary) {
      for(size_t i = 0; i < gcfg.GetNumRobots(); i++) {
        auto cfg = gcfg.GetRobotCfg(i);
        auto boundary = boundaryMap.at(cfg.GetRobot());
        if(cfg.InBounds(boundary))
          continue;
        
        passed = false;
        message = message + "\n\tA group configuration was incorrectly labeled "
              "valid for the given boundary.\n";
        break;
      }
    }
    
    if(!passed)
      break;
  }

  if(passed) {
    message = "GroupCfgSampleIndividualBoundary::PASSED!\n";
  }
  else {
    message = "GroupCfgSampleIndividualBoundary::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}


template <typename MPTraits>
typename BridgeTestSamplerTest<MPTraits>::TestResult
BridgeTestSamplerTest<MPTraits>::
TestGroupFilter() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->GroupFilter();

  if(passed) {
    message = "GroupFilter::PASSED!\n";
  }
  else {
    message = "GroupFilter::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

#endif
