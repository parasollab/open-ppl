#ifndef PPL_MIX_SAMPLER_TEST_H_
#define PPL_MIX_SAMPLER_TEST_H_

#include "MPLibrary/Samplers/MixSampler.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

template <class MPTraits>
class MixSamplerTest : virtual public MixSampler<MPTraits>, 
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    MixSamplerTest();

    MixSamplerTest(XMLNode& _node);

    ~MixSamplerTest();

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
    ///@name SampleMethodTest Default Function Overrides
    ///@{

      void IndividualCfgSample(size_t& _numNodes, Boundary*& _boundary, 
                    std::vector<Cfg>& _cfgs, std::vector<double>& _sampleProportions);
    
    ///@}


};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MixSamplerTest<MPTraits>::
MixSamplerTest() : MixSampler<MPTraits>() {}

template <typename MPTraits>
MixSamplerTest<MPTraits>::
MixSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           MixSampler<MPTraits>(_node) {}

template <typename MPTraits>
MixSamplerTest<MPTraits>::
~MixSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename MixSamplerTest<MPTraits>::TestResult
MixSamplerTest<MPTraits>::
TestIndividualCfgSample() {

  bool passed = true;
  std::string message = "";

  Boundary* boundary = nullptr;
  std::vector<Cfg> cfgs;

  // Make a vector to calculate the sample instances of each of the samplers.
  std::vector<double> sampleProportions(this->samplers.size(), 0);

  size_t numNodes = 10000;

  this->IndividualCfgSample(numNodes, boundary, cfgs, sampleProportions);
  
  std::cout << cfgs.size() << std::endl;

  for(size_t i = 0; i < sampleProportions.size(); i++) {
    std::cout << this->GetSampler(sampler.first) << ": " << sampleProportions[i]/(numNodes) << std::endl;
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
typename MixSamplerTest<MPTraits>::TestResult
MixSamplerTest<MPTraits>::
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
typename MixSamplerTest<MPTraits>::TestResult
MixSamplerTest<MPTraits>::
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
typename MixSamplerTest<MPTraits>::TestResult
MixSamplerTest<MPTraits>::
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
typename MixSamplerTest<MPTraits>::TestResult
MixSamplerTest<MPTraits>::
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
typename MixSamplerTest<MPTraits>::TestResult
MixSamplerTest<MPTraits>::
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

template <typename MPTraits>
void
MixSamplerTest<MPTraits>::
IndividualCfgSample(size_t& _numNodes, Boundary*& _boundary, std::vector<Cfg>& _cfgs, std::vector<double>& _sampleProportions) {

  this->SetLibraryRobot();

  size_t maxAttempts = 1;

  if(!_boundary)
    _boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();

  for (size_t i = 0; i < _numNodes; i++) {
    this->Sample(1, maxAttempts,_boundary, std::back_inserter(_cfgs),
               std::back_inserter(_cfgs));
    int samplerIndex = this->samplerLabels.second;
    _sampleProportions[samplerIndex] += 1;
  }
}
#endif
