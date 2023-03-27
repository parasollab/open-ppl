#ifndef PPL_MIX_SAMPLER_TEST_H_
#define PPL_MIX_SAMPLER_TEST_H_

#include "MPLibrary/Samplers/MixSampler.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"
#include "Utilities/MPUtils.h"

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
                    std::vector<Cfg>& _cfgs, std::vector<size_t>& _sampleProportions);
    
    ///@}
    ///@Helpers
    ///@{

    double binom(int _n, int _k, double _p);

    double multinom(int _n, std::vector<size_t>& _k, std::vector<double>& _p);
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

  // Sample using MixSampler a bunch of times. Keep a track of which type sampler was 
  // used.

  // Make a vector to calculate the sample instances of each of the samplers.
  std::vector<size_t> sampleProportions(this->samplers.size(), 0);

  size_t numTrials= 500;

  this->IndividualCfgSample(numTrials, boundary, cfgs, sampleProportions);
  
  // The test for the mix sampler is going to do "n" = the number of samplers 
  // two-sided binomial p-test. If any of these fail, the mixsampler fails.
  //    I think technically this is the wrong test because we have multiple classes. 
  //    ie. we are checking if each class is within the acceptable bounds instead of
  //    if the error from all the classes is within the acceptable bounds.

  // Define the significance level
  double alpha = 0.2;

  for(size_t i = 0; i < sampleProportions.size(); i++) {
    // n = numTrials
    // k = sampleProportions[i];
    double p = this->samplers[i].second; //probability of success

    // Samplers.second contains the cummulative sums, so we need to subtract
    if (i > 0) { p = p - this->samplers[i-1].second; }

    // Conduct a binomial random variable p-test
    bool tempTest = binom_test(alpha, numTrials, sampleProportions[i], p);
    
    if (!tempTest) { passed = false; }
  }

  // Multinomial Statistical Test - overflowing
  //std::vector<double> proportions;
  //proportions.push_back(this->samplers[0].second);

  //for (size_t j = 1; j < this->samplers.size(); j++) {
  //  proportions.push_back(this->samplers[j].second - this->samplers[j-1].second);
  //} 

  //double criticalValue = multinom(numTrials, sampleProportions, proportions);
  //double p_value = 0;

  //std::vector<size_t> testProportions;
  //for (size_t i = 0; i <= numTrials; i++) {
  //  testProportions = {i, nuTrials - i};
  //  double tempValue = multinom(numTrials, testProportions, proportions);
  //  if (tempValue <= criticalValue) { p_value += tempValue; }
  //}

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
IndividualCfgSample(size_t& _numNodes, Boundary*& _boundary, std::vector<Cfg>& _cfgs, std::vector<size_t>& _sampleProportions) {

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

template <typename MPTraits>
double
MixSamplerTest<MPTraits>::
binom(int _n, int _k, double _p) {
  double binomCoeff = boost::math::binomial_coefficient<double>(_n, _k);
  double p_value = binomCoeff*std::pow(_p, _k)*std::pow(1-_p, _n - _k);
  return p_value;
}

template <typename MPTraits>
double
MixSamplerTest<MPTraits>::
multinom(int _n, std::vector<size_t>& _k, std::vector<double>& _p) {
  double n_fact = boost::math::factorial<double>(_n);
  std::cout << "n! good" << std::endl;
  double prod_term = 1;
  for (size_t j = 0; j < _k.size(); j++) {
    double numerator = std::pow(_p[j], _k[j]);
    std::cout << "power" << std::endl;
    double denominator = boost::math::factorial<double>(_k[j]);
    std::cout << "k!" << std::endl;
    prod_term *= (numerator/denominator);
  }

  return n_fact*prod_term;
}
#endif
