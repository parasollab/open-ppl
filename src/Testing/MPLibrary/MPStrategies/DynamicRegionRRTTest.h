#ifndef PPL_DYNAMIC_REGION_RRT_TEST_H_
#define PPL_DYNAMIC_REGION_RRT_TEST_H_

#include "MPLibrary/MPStrategies/DynamicRegionRRT.h"
#include "Testing/MPLibrary/MPStrategies/MPStrategyMethodTest.h"

template <class MPTraits>
class DynamicRegionRRTTest : virtual public DynamicRegionRRT<MPTraits>, 
                             public MPStrategyMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    typedef typename DynamicRegionRRT<MPTraits>::SamplingRegion SamplingRegion;

    ///@}
    ///@name Construction
    ///@{

    DynamicRegionRRTTest();

    DynamicRegionRRTTest(XMLNode& _node);

    ~DynamicRegionRRTTest();

    ///@}

  private: 

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestStrategy() override;

    TestResult TestCreateRegions();

    TestResult TestIsTouching();

    TestResult TestComputeProbabilities();

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
DynamicRegionRRTTest<MPTraits>::
DynamicRegionRRTTest() : DynamicRegionRRT<MPTraits>() {}

template <typename MPTraits>
DynamicRegionRRTTest<MPTraits>::
DynamicRegionRRTTest(XMLNode& _node) : DynamicRegionRRT<MPTraits>(_node), 
                                       MPStrategyMethod<MPTraits>(_node) {}

template <typename MPTraits>
DynamicRegionRRTTest<MPTraits>::
~DynamicRegionRRTTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename DynamicRegionRRTTest<MPTraits>::TestResult
DynamicRegionRRTTest<MPTraits>::
TestStrategy() {
  bool passed = true;
  std::string message = "";

  auto result = TestComputeProbabilities();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename DynamicRegionRRTTest<MPTraits>::TestResult
DynamicRegionRRTTest<MPTraits>::
TestComputeProbabilities() {

  bool passed = true;
  std::string message = "";

  // Create sampling regions
  SamplingRegion region1 = SamplingRegion({});
  SamplingRegion region2 = SamplingRegion({});
  SamplingRegion region3 = SamplingRegion({});

  this->m_regions.push_back(region1);
  this->m_regions.push_back(region2);
  this->m_regions.push_back(region3);

  this->m_regions[0].attempts = 2;
  this->m_regions[2].attempts = 10;
  this->m_regions[2].successes = 2;

  // Compute the probabilities of sampling from each region
  auto probs = this->ComputeProbabilities();

  double totalProb = 0.0;
  for (auto p : probs) {
    totalProb += p;
  }

  // Check that the size of the probability distribution is correct
  if (probs.size() != 4) {
    passed = false;
    message = message + "\n\tThe size of the probability distribution is incorrect.\n";
  } else if (probs[3] != this->m_explore / probs.size()) {
    passed = false;
    message = message + "\n\tThe probability of selecting the entire environment is incorrect.\n";
  }

  // Check that regions with a higher success rate have a higher probability
  if ((probs[1] <= probs[0]) or (probs[2] >= probs[0])) {
    passed = false;
    message = message + "\n\tThe region selection probabilities are incorrect.\n";
  }

  // Check that the probabilities form a valid probability distribution
  if (totalProb != 1.0) {
    passed = false;
    message = message + "\n\tThe probability distribution does not sum to 1.0.\n";
  }

  if(passed) {
    message = "ComputeProbabilities::PASSED!\n";
  }
  else {
    message = "ComputeProbabilities::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/

#endif
