#ifndef PPL_OBSTACLE_BASED_SAMPLER_H_
#define PPL_OBSTACLE_BASED_SAMPLER_H_

#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MetricUtils.h"


template <class MPTraits>
class ObstacleBasedSamplerTest : virtual public ObstacleBasedSampler<MPTraits>, 
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{
    typedef typename MPTraits::CfgType        CfgType;
    typedef TestBaseObject::TestResult        TestResult;
    typedef typename MPTraits::MPLibrary      MPLibrary;

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

    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    string m_vcLabel, m_dmLabel; ///< Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; ///< Number of free and collision shells
    bool m_useBBX; ///< Is the bounding box an obstacle?
    string m_pointSelection; ///< Needed for the WOBPRM
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest() : ObstacleBasedSampler<MPTraits>() {
  m_medialAxisUtility = MedialAxisUtility<MPTraits>("rapid", "euclidean",
                                false, false, 300, 300, false, true, false, 0.1, 5);
}

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           ObstacleBasedSampler<MPTraits>(_node) {

  m_vcLabel = _node.Read("vcLabel", false, m_vcLabel, "Validity Test Method");
  m_dmLabel = _node.Read("dmLabel", false, m_dmLabel, "Distance metric");

  m_useBBX = _node.Read("useBBX", true, false, "Use bounding box as obstacle");

  m_medialAxisUtility = MedialAxisUtility<MPTraits>("rapid", "euclidean",
                                false, false, 300, 300, m_useBBX, true, false, 0.1, 5);

  m_nShellsColl = _node.Read("nShellsColl", true, 3, 0, 10,
      "Number of collision shells");
  m_nShellsFree = _node.Read("nShellsFree", true, 3, 0, 10,
      "Number of free shells");
}

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
~ObstacleBasedSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestIndividualCfgSample() {

  // Get MPLibrary and Environment
  auto mpl = this->GetMPLibrary();
  auto env = this->GetEnvironment();

  // Define necessary variables
  bool passed = true;
  std::string message = "";

  Boundary* boundary = nullptr;
  std::vector<Cfg> valids;
  std::vector<Cfg> invalids;
  std::vector<double> minDistInfoVec;

  // Call the Sampler
  this->IndividualCfgSample(boundary, valids, invalids);
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker("rapid");

  // Initialize the Medial Axis Utilitiy
  // Set MPLibrary
  m_medialAxisUtility.SetMPLibrary(mpl);
  // Initialize
  m_medialAxisUtility.Initialize();

  // Iterate through valid cfgs
  for (auto cfg : valids){
    if(!vc->IsValid(cfg, callee)) {   
      passed = false;
      message = message + "\n\tA configuration was incorrectly labeled "
                "valid for the given boundary. n";
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
    minDistInfoVec.push_back(_cdInfo.m_minDist);
  }

  // Average out the distances
  double avgDist = std::accumulate(minDistInfoVec.begin(), minDistInfoVec.end(), 0.0) / minDistInfoVec.size();

  // std::cout << "Single Robot Single Boundary: " << avgDist << std::endl;

  // Set a criteria for pass
  double envRes = m_nShellsFree*max(env->GetPositionRes(), env->GetOrientationRes());

  // Compare them 
  if (avgDist > envRes) {
    passed = false;
    message = "Average minimum distance from valid cfgs to obstacles is too large \n";
  }

  // Iterate through invalid cfgs
  for (auto cfg : invalids){
    if(vc->IsValid(cfg, callee)) {   
      passed = false;
      message = message + "\n\tA configuration was incorrectly labeled "
                "invalid for the given boundary. n";
      break;
    }

    CDInfo _cdInfo;
    CfgType _cfgClr;
    bool valid = false;

    // Find a nearest obstacle cfg
    valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

    // Cannot find a invalid obstacle cfg    
    if (valid) {
      passed = false;
      message = "Cannot find the closest obstacle from given cfg\n";
      break;
    }

    // Store cdInfo and minimum distance
    minDistInfoVec.push_back(_cdInfo.m_minDist);
  }

  // Average out the distances
  double avgDist = std::accumulate(minDistInfoVec.begin(), minDistInfoVec.end(), 0.0) / minDistInfoVec.size();

  // std::cout << "Single Robot Single Boundary: " << avgDist << std::endl;

  // Set a criteria for pass
  double envRes = m_nShellsFree*max(env->GetPositionRes(), env->GetOrientationRes());

  // Compare them 
  if (avgDist > envRes) {
    passed = false;
    message = "Average minimum distance from cfgs to obstacles are too large \n";
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

  // Call MPLibrary and Tasks
  auto mpl = this->GetMPLibrary();
  
  // Define necessary variables
  bool passed = true;
  std::string message = "";

  Boundary* boundary = nullptr;
  std::vector<GroupCfg> valids;
  std::vector<GroupCfg> invalids;
  std::vector<double> minDistInfoVec;

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
  for (auto groupCfg : valids) {
    if(!vc->IsValid(groupCfg, callee)) {
      passed = false;
      message = message + "\n\tA configuration was incorrectly labeled "
                "valid for the given boundary. n";
      break;
    }

    std::vector<double> tmpMinDistVec;

    for (auto task : *groupTask) {
      CDInfo _cdInfo;
      CfgType _cfgClr;
      bool valid = false;
      
      m_medialAxisUtility.GetMPLibrary()->SetTask(&task);
      
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

      tmpMinDistVec.push_back(_cdInfo.m_minDist);
    }

    // Store cdInfo and minimum distance
    auto minVal = std::min_element(tmpMinDistVec.begin(), tmpMinDistVec.end());

    if (!passed)
      break;

    // std::cout << "Group Robot Single Boundary: " << *minVal << std::endl;

    // Set a criteria for pass
    double envRes = m_nShellsFree*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());

    // Compare them 
    if (*minVal > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
      break;
    }
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
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {
  
  // Call MPLibrary
  auto mpl = this->GetMPLibrary();
  
  // Define necessary variables
  bool passed = true;
  std::string message = "";

  std::map<Robot*,const Boundary*> boundaryMap;
  std::vector<GroupCfg> valids;
  std::vector<GroupCfg> invalids;
  std::vector<double> minDistInfoVec;

  // Call the Sampler
  this->GroupCfgSampleIndividualBoundaries(boundaryMap, valids, invalids);
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker("rapid");

  // Initialize the Medial Axis Utility
  m_medialAxisUtility.SetMPLibrary(mpl);
  m_medialAxisUtility.Initialize();

  auto groupTask = this->GetGroupTask();
  m_medialAxisUtility.GetMPLibrary()->SetGroupTask(groupTask);
  m_medialAxisUtility.GetMPLibrary()->SetTask(nullptr);

  // Iterate through valid cfgs
  for (auto groupCfg : valids) {
    if(!vc->IsValid(groupCfg, callee)) {
      passed = false;
      message = message + "\n\tA configuration was incorrectly labeled "
                "valid for the given boundary. n";
      break;
    }

    std::vector<double> tmpMinDistVec;

    for (auto task : *groupTask) {
      CDInfo _cdInfo;
      CfgType _cfgClr;
      bool valid = false;

      // Set an individual task for the Medial Axis Utility
      m_medialAxisUtility.GetMPLibrary()->SetTask(&task);
      
      // Get the individual robot and corresponding cfg
      auto robot = task.GetRobot();
      CfgType cfg = groupCfg.GetRobotCfg(robot);

      // Get a boundary of a robot
      auto boundary = boundaryMap.at(robot);

      // Find a nearest obstacle cfg
      valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

      // Cannot find a valid obstacle cfg    
      if (!valid) {
        passed = false;
        message = "Cannot find the closest obstacle from given cfg\n";
        std::cout << message << std::endl;
        break;
      }

      tmpMinDistVec.push_back(_cdInfo.m_minDist);
    }

    // Find the minimum distacne and store it in the vector
    auto minVal = std::min_element(tmpMinDistVec.begin(), tmpMinDistVec.end());
    
    if (!passed)
      break;

    // std::cout << "Group Robot Individual Boundary: " << *minVal << std::endl;

    // Set a criteria for pass
    double envRes = m_nShellsFree*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());

    // Compare them 
    if (*minVal > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
      break;
    }
  }
  
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
