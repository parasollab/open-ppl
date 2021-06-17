#ifndef PPL_UNIFORM_RANDOM_SAMPLER_H_
#define PPL_UNIFORM_RANDOM_SAMPLER_H_

#include "MPLibrary/Samplers/UniformRandomSampler.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

template <class MPTraits>
class UniformRandomSamplerTest : public UniformRandomSampler<MPTraits>, 
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
UniformRandomSamplerTest(XMLNode& _node) : UniformRandomSampler<MPTraits>(_node) {}

template <typename MPTraits>
UniformRandomSamplerTest<MPTraits>::
~UniformRandomSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestIndividualCfgSample() {

  this->IndividualCfgSample();

  return TestResult();
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestIndividualCfgSampleWithEEConstraint() {

  this->IndividualCfgSampleWithEEConstraint();

  return TestResult();
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestIndividualFilter() {

  this->IndividualFilter();

  return TestResult();
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestGroupCfgSampleSingleBoundary() {
  
  this->GroupCfgSampleSingleBoundary();

  return TestResult();
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {
  
  this->GroupCfgSampleIndividualBoundaries();

  return TestResult();
}

template <typename MPTraits>
typename UniformRandomSamplerTest<MPTraits>::TestResult
UniformRandomSamplerTest<MPTraits>::
TestGroupFilter() {

  this->GroupFilter();

  return TestResult();
}

#endif
