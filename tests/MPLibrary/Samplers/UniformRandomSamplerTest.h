#ifndef PPL_UNIFORM_RANDOM_SAMPLER_H_
#define PPL_UNIFORM_RANDOM_SAMPLER_H_

#include "MPLibrary/Samplers/UniformRandomSampler.h"    //src
#include "SamplerMethodTest.h"

template <class MPTraits>
class UniformRandomSamplerTest : virtual public UniformRandomSampler<MPTraits>, 
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    UniformRandomSamplerTest();

    UniformRandomSamplerTest(XMLNode& _node);

    ~UniformRandomSamplerTest();

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
UniformRandomSamplerTest<MPTraits>::
UniformRandomSamplerTest() : UniformRandomSampler<MPTraits>() {}

template <typename MPTraits>
UniformRandomSamplerTest<MPTraits>::
UniformRandomSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           UniformRandomSampler<MPTraits>(_node) {}

template <typename MPTraits>
UniformRandomSamplerTest<MPTraits>::
~UniformRandomSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
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
    std::cout << "\n\tA configuration was incorrectly labeled valid for the given boundary." << std::endl;
    break;
  }

  for(auto cfg : invalids) {
    if(!boundary->InBoundary(cfg))   
      continue;

    passed = false;
    std::cout << "\n\tA configuration was incorrectly labeled invalid for the given boundary." << std::endl;
    break;
  }

  message = "\tFINISHED TestIndividualCfgSample";
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestIndividualCfgSampleWithEEConstraint() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->IndividualCfgSampleWithEEConstraint();

  message = "\tFINISHED TestIndividualCfgSampleWithEEConstraint";
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestIndividualFilter() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->IndividualFilter();

  message = "\tFINISHED TestIndividualFilter";
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestGroupCfgSampleSingleBoundary() {

  bool passed = true;
  std::string message = "";
  
  //TODO::Setup test of this function.
  //this->GroupCfgSampleSingleBoundary();

  message = "\tFINISHED TestGroupCfgSampleSingleBoundary";
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {

  bool passed = true;
  std::string message = "";
  
  //TODO::Setup test of this function.
  //this->GroupCfgSampleIndividualBoundaries();

  message = "\tFINISHED TestGroupCfgSampleIndividualBoundaries";
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestGroupFilter() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->GroupFilter();

  message = "\tFINISHED TestGroupFilter";
  return std::make_pair(passed,message);
}

#endif
