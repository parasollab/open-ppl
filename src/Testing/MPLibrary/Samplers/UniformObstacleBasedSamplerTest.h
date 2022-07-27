#ifndef PPL_OBSTACLE_BASED_SAMPLER_H_
#define PPL_OBSTACLE_BASED_SAMPLER_H_

#include "MPLibrary/Samplers/UniformObstacleBasedSampler.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MetricUtils.h"

template <class MPTraits>
class UniformObstacleBasedSamplerTest : virtual public UniformObstacleBasedSampler<MPTraits>,
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{
    typedef typename MPTraits::CfgType        CfgType;
    typedef TestBaseObject::TestResult TestResult;
    typedef typename MPTraits::MPLibrary      MPLibrary;
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

    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
UniformObstacleBasedSamplerTest<MPTraits>::
UniformObstacleBasedSamplerTest() : UniformObstacleBasedSampler<MPTraits>() {
  m_medialAxisUtility = MedialAxisUtility<MPTraits>("pqp_solid", "euclidean",
                              false, false, 500, 500, true, true, true, 0.1, 5);

}

template <typename MPTraits>
UniformObstacleBasedSamplerTest<MPTraits>::
UniformObstacleBasedSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           UniformObstacleBasedSampler<MPTraits>(_node) {
  m_medialAxisUtility = MedialAxisUtility<MPTraits>("pqp_solid", "euclidean",
                                false, false, 500, 500, true, true, true, 0.1, 5);
                                           }

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
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker("rapid");

  // Make sure that all valids are inside the boundary
  // and all invalids are outside the boundary.
  // FindApproximateWitness
  MPLibrary* mpl = this->GetMPLibrary();

  // Initialize MedialAxisUtility
  if(!m_medialAxisUtility.IsInitialized()) {
    // Set MPLibrary pointer
    m_medialAxisUtility.SetMPLibrary(mpl);
    // Initialize
    m_medialAxisUtility.Initialize();
  }

  std::vector<CDInfo> cdInfo_vec;
  std::vector<double> minDistInfo_vec;
  // Iterate through valid cfgs

  for(auto cfg : valids) {
    std::cout << cfg.PrettyPrint();
    if(!vc->IsValid(cfg, callee)) { 
    passed = false;
    message = message + "\n\tA configuration was incorrectly labeled "
              "valid for the given boundary.\n";
    break;
  }

  CDInfo _cdInfo;
  CfgType _cfgClr;
  bool valid = false;

  // Find a nearest obstacle cfg
  valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

  // Cannot find a valid obstacle cfg    
  if (!valid) {
    passed = false;
    message = "Cannot find the closest obstacle from given cfg\n";
    break;
  }

  // Store cdInfo and minimum distance
  cdInfo_vec.push_back(_cdInfo);
  minDistInfo_vec.push_back(_cdInfo.m_minDist);


  // Average out the distances
  double avgDist = std::accumulate(minDistInfo_vec.begin(), minDistInfo_vec.end(), 0.0) / minDistInfo_vec.size();
  // Set a criteria for pass
  double c = 15*min(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
  std::cout << "(average distance)" << avgDist << " | (threshold)" << c << std::endl;

  // Compare them 
  if (avgDist > c) {
    passed = false;
    message = "Average minimum distance from cfgs to obstacles are too large: ";
    break;
  }
}

//same thing needs to be implemented for invalid configurations...


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