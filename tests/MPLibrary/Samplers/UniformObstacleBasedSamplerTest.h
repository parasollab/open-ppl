#ifndef PPL_UNIFORM_OBSTACLE_BASED_SAMPLER_H_
#define PPL_UNIFORM_OBSTACLE_BASED_SAMPLER_H_

#include "MPLibrary/Samplers/UniformObstacleBasedSampler.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"
#include "SamplerMethodTest.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MetricUtils.h"

template <class MPTraits>
class UniformObstacleBasedSamplerTest : virtual public UniformObstacleBasedSampler<MPTraits>,
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{
    typedef typename MPTraits::CfgType        CfgType;
    typedef typename MPTraits::GroupCfgType   GroupCfg;
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
  // double lMargin = this->GetMargin() * 1.2;

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
   // std::cout << cfg.PrettyPrint();
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
  double c = 2*min(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
  //std::cout << "(average distance)" << avgDist << " | (threshold)" << c << std::endl;

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
  
// Define necessary variables
  bool passed = true;
  std::string message = "";
// Call MPLibrary and Tasks
  auto mpl = this->GetMPLibrary();
  
  // Define necessary variables

  Boundary* boundary = nullptr;
  std::vector<GroupCfg> valids;
  std::vector<GroupCfg> invalids;
  std::vector<double> tmpMinDistVec;
  std::vector<double> minDistVec;
  double envRes;
  double avgDist;
  // double lMargin = this->GetMargin() * 1.2;
  
  // Call the Sampler
  this->GroupCfgSampleSingleBoundary(boundary, valids, invalids);
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker("rapid");

  // Initialize the Medial Axis Utilites
  m_medialAxisUtility.SetMPLibrary(mpl);
  m_medialAxisUtility.Initialize();
  auto groupTask = this->GetGroupTask();
  m_medialAxisUtility.GetMPLibrary()->SetGroupTask(groupTask);
  m_medialAxisUtility.GetMPLibrary()->SetTask(nullptr);

  // Iterate through valid cfgs
  // First check the number of free shell
    for (auto groupCfg : valids) {
      // Check the validity
      if(!vc->IsValid(groupCfg, callee)) {
        passed = false;
        message = message + "\n\tA configuration was incorrectly labeled "
                  "valid for the given boundary.\n";
        break;
      }

      tmpMinDistVec.empty();

      // Iterate through the individual task
      for (auto task : *groupTask) {
        // Initialize necessary variables
        CDInfo _cdInfo;
        CfgType _cfgClr;
        bool valid = false;
        
        // Set the individual task of the Medial Axis Utility
        m_medialAxisUtility.GetMPLibrary()->SetTask(&task);
        
        // Get the corresponding robot as its cfg
        auto robot = task.GetRobot();
        CfgType cfg = groupCfg.GetRobotCfg(robot);

        // Find a nearest obstacle cfg
        valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

        // Cannot find a valid obstacle cfg    
        if (!valid) {
          passed = false;
          message = "Cannot find the closest obstacle from given cfg\n";
          std::cout << message << std::endl;
          break;
        }

        // Store the minimum distance information of each individual robot
        tmpMinDistVec.push_back(_cdInfo.m_minDist);
      }

      // Find the shortest distance and store it in the vector
      auto minVal = std::min_element(tmpMinDistVec.begin(), tmpMinDistVec.end());
      minDistVec.push_back(*minVal);

      if (!passed)
        break;
    }
    
    // Set a criteria to pass
    envRes = 2*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();

    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
    }


  // Iterate through invalid cfgs
  // First check the number of free shell
    tmpMinDistVec.empty();
    minDistVec.empty();

    for (auto groupCfg : invalids) {
      // Check the validity
      if(vc->IsValid(groupCfg, callee)) {
        passed = false;
        message = message + "\n\tA configuration was incorrectly labeled "
                  "valid for the given boundary.\n";
        break;
      }

      tmpMinDistVec.empty();

      // Iterate through the individual task
      for (auto task : *groupTask) {
        // Initialize necessary variables
        CDInfo _cdInfo;
        CfgType _cfgClr;
        bool valid = false;

        // Set the individual task of the Medial Axis Utility
        m_medialAxisUtility.GetMPLibrary()->SetTask(&task);

        // Get the corresponding robot as its cfg
        auto robot = task.GetRobot();
        CfgType cfg = groupCfg.GetRobotCfg(robot);

        // Find a nearest obstacle cfg
        valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

        // Cannot find a valid obstacle cfg    
        if (!valid) {
          passed = false;
          message = "Cannot find the closest obstacle from given cfg\n";
          std::cout << message << std::endl;
          break;
        }

        // Store the minimum distance information of each individual robot
        tmpMinDistVec.push_back(_cdInfo.m_minDist);
      }

      // Find the shortest distance and store it in the vector
      auto minVal = std::min_element(tmpMinDistVec.begin(), tmpMinDistVec.end());
      minDistVec.push_back(abs(*minVal));

      if (!passed)
        break;
    }
    
    // set a criteria to pass
    envRes = 2*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();

    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
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
typename UniformObstacleBasedSamplerTest<MPTraits>::TestResult
UniformObstacleBasedSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {
  
  // Define necessary variables
  bool passed = true;
  std::string message = "";

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