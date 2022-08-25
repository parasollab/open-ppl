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
    typedef typename MPTraits::GroupCfgType   GroupCfg;
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

    int m_nShellsFree, m_nShellsColl; ///< Number of free and collision shells
    bool m_useBBX; ///< Is the bounding box an obstacle?
    string m_pointSelection; ///< Needed for the WOBPRM
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest() : ObstacleBasedSampler<MPTraits>() {
  m_medialAxisUtility = MedialAxisUtility<MPTraits>("rapid", "euclidean",
                                false, false, 30, 30, false, true, false, 0.1, 5);
}

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           ObstacleBasedSampler<MPTraits>(_node) {

  m_useBBX = _node.Read("useBBX", true, false, "Use bounding box as obstacle");

  m_medialAxisUtility = MedialAxisUtility<MPTraits>("rapid", "euclidean",
                                false, false, 30, 30, m_useBBX, true, false, 0.1, 5);

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
  std::vector<double> minDistVec;
  double avgDist;
  double envRes;

  // Call the Sampler
  this->IndividualCfgSample(boundary, valids, invalids);
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker("rapid");

  // Set MPLibrary and initialize the Medial Axis Utilitiy
  m_medialAxisUtility.SetMPLibrary(mpl);
  m_medialAxisUtility.Initialize();

  // Iterate through valid cfgs
  // First check the number of free shell
  if (m_nShellsFree > 0) {
    for (auto cfg : valids){
      // Check the validity
      if(!vc->IsValid(cfg, callee)) {   
        passed = false;
        message = message + "\n\tA configuration was incorrectly labeled "
                  "valid for the given boundary.\n";
        break;
      }

      // Initilize necessary variables
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
      minDistVec.push_back(_cdInfo.m_minDist);

      if (!passed)
        break;
    }

    // Set a criteria to pass
    envRes = m_nShellsFree*max(env->GetPositionRes(), env->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();
    
    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from valid cfgs to obstacles is too large \n";
    }
  }

  // Iterate through invalid cfgs
  // First check the number of collision shell
  if (m_nShellsColl > 0) {  
    minDistVec.empty();
    for (auto cfg : invalids) {
      // Check the validity
      if(vc->IsValid(cfg, callee)) {   
        passed = false;
        message = message + "\n\tA configuration was incorrectly labeled "
                  "invalid for the given boundary.\n";
        break;
      }

      // Initialize necessary variables
      CDInfo _cdInfo;
      CfgType _cfgClr;
      bool valid = false;

      // Find a nearest obstacle cfg
      valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

      // Cannot find a nearby obstacle
      if (!valid) {
        passed = false;
        message = "Cannot find the closest obstacle from given cfg\n";
        break;
      }

      // Store cdInfo and minimum distance
      minDistVec.push_back(abs(_cdInfo.m_minDist));

      if (!passed) 
        break;
    }


    // Set a criteria to pass
    envRes = m_nShellsColl*max(env->GetPositionRes(), env->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();

    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
    }
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
  std::vector<double> tmpMinDistVec;
  std::vector<double> minDistVec;
  double envRes;
  double avgDist;

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
  if (m_nShellsFree > 0) {
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
    envRes = m_nShellsFree*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();

    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
    }
  }


  // Iterate through invalid cfgs
  // First check the number of free shell
  if (m_nShellsColl > 0) {
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
    double envRes = m_nShellsColl*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();

    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
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
  std::vector<double> tmpMinDistVec;
  std::vector<double> minDistVec;
  double envRes;
  double avgDist;

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
  // First check the number of free shell
  if (m_nShellsFree > 0) {
    // Check the validity
    for (auto groupCfg : valids) {
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

        // Store the minimum distance information of each individual robot
        tmpMinDistVec.push_back(_cdInfo.m_minDist);
      }

      // Find the shortest distacne and store it in the vector
      auto minVal = std::min_element(tmpMinDistVec.begin(), tmpMinDistVec.end());
      minDistVec.push_back(*minVal);
      
      if (!passed)
        break;
    }

    // Set a criteria to pass
    envRes = m_nShellsFree*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();
    
    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
    }
  }

  // Iterate through valid cfgs
  // First check the number of free shell  
  if (m_nShellsColl > 0) {
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

        // Store the minimum distance information of each individual robot
        tmpMinDistVec.push_back(_cdInfo.m_minDist);
      }

      // Find the minimum distacne and store it in the vector
      auto minVal = std::min_element(tmpMinDistVec.begin(), tmpMinDistVec.end());
      minDistVec.push_back(abs(*minVal));

      if (!passed)
        break;
    }

    // set a criteria to pass
    envRes = m_nShellsColl*max(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    avgDist = std::accumulate(minDistVec.begin(), minDistVec.end(), 0.0) / minDistVec.size();

    // Compare them 
    if (avgDist > envRes) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
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
