#ifndef PPL_UNIFORM_RANDOM_SAMPLER_H_
#define PPL_UNIFORM_RANDOM_SAMPLER_H_

#include "MPLibrary/Samplers/GaussianSampler.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

template <class MPTraits>
class GaussianSamplerTest : virtual public GaussianSampler<MPTraits>, 
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    GaussianSamplerTest();

    GaussianSamplerTest(XMLNode& _node);

    ~GaussianSamplerTest();

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
GaussianSamplerTest<MPTraits>::
GaussianSamplerTest() : GaussianSampler<MPTraits>() {}

template <typename MPTraits>
GaussianSamplerTest<MPTraits>::
GaussianSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           GaussianSampler<MPTraits>(_node) {}

template <typename MPTraits>
GaussianSamplerTest<MPTraits>::
~GaussianSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename GaussianSamplerTest<MPTraits>::TestResult
GaussianSamplerTest<MPTraits>::
TestIndividualCfgSample() {
  // Make sure that all "valids" are valid and inside boundary
  // and all "invalids" are invalid or outside boundary.

  // And we maintain 68/93/97/99 splits. 

}

template <typename MPTraits>
typename GaussianSamplerTest<MPTraits>::TestResult
GaussianSamplerTest<MPTraits>::
TestIndividualCfgSampleWithEEConstraint() {

}

template <typename MPTraits>
typename GaussianSamplerTest<MPTraits>::TestResult
GaussianSamplerTest<MPTraits>::
TestIndividualFilter() {

}

template <typename MPTraits>
typename GaussianSamplerTest<MPTraits>::TestResult
GaussianSamplerTest<MPTraits>::
TestGroupCfgSampleSingleBoundary() {

}

template <typename MPTraits>
typename GaussianSamplerTest<MPTraits>::TestResult
GaussianSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {

}

template <typename MPTraits>
typename GaussianSamplerTest<MPTraits>::TestResult
GaussianSamplerTest<MPTraits>::
TestGroupFilter() {

}

#endif
