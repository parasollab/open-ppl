#ifndef PPL_OBSTACLE_BASED_SAMPLER_H_
#define PPL_OBSTACLE_BASED_SAMPLER_H_

#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

template <class MPTraits>
class ObstacleBasedSamplerTest : virtual public ObstacleBasedSampler<MPTraits>,
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    ObstacleBasedSamplerTest();

    ObstacleBasedSamplerTest(XMLNode& _node);

    ~ObstacleBasedSamplerTest();

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
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest() : ObstacleBasedSampler<MPTraits>() {}

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           ObstacleBasedSampler<MPTraits>(_node) {}

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
~ObstacleBasedSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
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
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
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
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
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
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
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
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
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
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
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
