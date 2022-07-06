#ifndef PPL_OBSTACLE_BASED_SAMPLER_H_
#define PPL_OBSTACLE_BASED_SAMPLER_H_

#include "MPLibrary/Samplers/UniformObstacleBasedSampler.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

template <class MPTraits>
class UniformObstacleBasedSamplerTest : virtual public ObstacleBasedSampler<MPTraits>,
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    UniformObstacleBasedSamplerTest();

    UniformObstacleBasedSamplerTest(XMLNode& _node);

    ~UniformObstacleBasedSamplerTest();

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
UniformObstacleBasedSamplerTest<MPTraits>::
UniformObstacleBasedSamplerTest() : UniformObstacleBasedSampler<MPTraits>() {}

template <typename MPTraits>
UniformObstacleBasedSamplerTest<MPTraits>::
UniformObstacleBasedSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           UniformObstacleBasedSampler<MPTraits>(_node) {}

template <typename MPTraits>
UniformObstacleBasedSamplerTest<MPTraits>::
~UniformObstacleBasedSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename UniformObstacleBasedSamplerTest<MPTraits>::TestResult
UniformObstacleBasedSamplerTest<MPTraits>::
TestIndividualCfgSample() {

  bool passed = true;
  std::string message = "";

  Boundary* boundary = nullptr;
  std::vector<Cfg> valids;
  std::vector<Cfg> invalids;

  this->IndividualCfgSample(boundary, valids, invalids);

  // Make sure that all valids are inside the boundary
  // and all invalids are outside the boundary.

  for(auto cfg : valids) {
    if(boundary->InBoundary(cfg))
      continue;

    passed = false;
    message = message + "\n\tA configuration was incorrectly labeled "
              "valid for the given boundary.\n";
    break;
  }

  for(auto cfg : invalids) {
    if(!boundary->InBoundary(cfg))
      continue;

    passed = false;
    message = message + "\n\tA configuration was incorrectly labeled "
              "invalid for the given boundary.\n";
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
typename UniformObstacleBasedSamplerTest<MPTraits>::TestResult
UniformObstacleBasedSamplerTest<MPTraits>::
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
typename UniformObstacleBasedSamplerTest<MPTraits>::TestResult
UniformObstacleBasedSamplerTest<MPTraits>::
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
typename UniformObstacleBasedSamplerTest<MPTraits>::TestResult
UniformObstacleBasedSamplerTest<MPTraits>::
TestGroupCfgSampleSingleBoundary() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->GroupCfgSampleSingleBoundary();

  if(passed) {
    message = "GroupCfgSampleSingleBoundary::PASSED!\n";
  }
  else {
    message = "GroupCfgSampleSingleBoundary::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename UniformObstacleBasedSamplerTest<MPTraits>::TestResult
UniformObstacleBasedSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->GroupCfgSampleIndividualBoundaries();

  if(passed) {
    message = "GroupCfgSampleIndividualBoundaries::PASSED!\n";
  }
  else {
    message = "GroupCfgSampleIndividualBoundaries::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename UniformObstacleBasedSamplerTest<MPTraits>::TestResult
UniformObstacleBasedSamplerTest<MPTraits>::
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