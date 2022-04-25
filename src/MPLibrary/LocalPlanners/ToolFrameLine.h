#ifndef PPL_TOOL_FRAME_LINE_H_
#define PPL_TOOL_FRAME_LINE_H_

#include "LocalPlannerMethod.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/KDLModel.h"

template <typename MPTraits>
class ToolFrameLine : public LocalPlannerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::WeightType       WeightType;

    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef std::vector<size_t>                 RobotFormation;

    ///@}
    ///@name Construction
    ///@{

    ToolFrameLine(const std::string& _vcLabel = "", bool _binary = false,
        bool _saveIntermediates = false);

    ToolFrameLine(XMLNode& _node);

    virtual ~ToolFrameLine();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    ///@}
    ///@name LocalPlannerMethod Override
    ///@{

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false) override;

    virtual bool IsConnected(
        const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
        GroupLPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        const RobotFormation& _robotIndexes = RobotFormation()) override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    /// Default for non closed chains
    bool IsConnectedFunc(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    bool ComputeIncrement(const std::vector<double>& _posIncrements,
                 const std::vector<double>& _oriIncrements,
                 CfgType& _current);

    GroupCfgType ComputeIncrement(const std::unordered_map<Robot*,std::vector<double>>& _posIncrements,
                 const std::unordered_map<Robot*,std::vector<double>>& _oriIncrements,
                 GroupCfgType& _current);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_dmLabel;          ///< The metric for measuring edge length.
    std::string m_vcLabel;          ///< The validity checker.
    std::string m_smLabel;          ///< Sampler Method.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ToolFrameLine<MPTraits>::
ToolFrameLine(const std::string& _vcLabel, bool _binary, bool _saveIntermediates)
  : LocalPlannerMethod<MPTraits>(_saveIntermediates), m_vcLabel(_vcLabel) {
  this->SetName("ToolFrameLine");
}


template <typename MPTraits>
ToolFrameLine<MPTraits>::
ToolFrameLine(XMLNode& _node) : LocalPlannerMethod<MPTraits>(_node) {
  this->SetName("ToolFrameLine");

  m_dmLabel = _node.Read("dmLabel", false, "euclidean",
      "The distance metric for computing edge length.");

  m_vcLabel = _node.Read("vcLabel", true, "", "The validity checker to use.");

  m_smLabel = _node.Read("smLabel", true, "", "The sampler method to use.");

}

template <typename MPTraits>
ToolFrameLine<MPTraits>::
~ToolFrameLine() { }

/*-------------------------- MPBaseObject Overrides --------------------------*/
/*----------------------- LocalPlannerMethod Overrides -----------------------*/

template <typename MPTraits>
bool
ToolFrameLine<MPTraits>::
IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  const std::string id = this->GetNameAndLabel();
  MethodTimer mt(this->GetStatClass(), id + "::IsConnected");

  if(this->m_debug) {
    std::cout << id
              << "\n\tChecking line from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << std::endl;
  }

  // Initialize the LPOutput object.
  _lpOutput->Clear();
  _lpOutput->SetLPLabel(this->GetLabel());

  auto env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = _c1.GetRobot();

  // Get number of steps
  int numSteps;
  Cfg stepsDummy(robot);
  stepsDummy.FindIncrement(_c1,_c2, &numSteps, env->GetPositionRes(), env->GetOrientationRes());
  
  // Compute increment
  std::vector<double> posIncrements;
  std::vector<double> oriIncrements;

  auto kdl = robot->GetKDLModel();
  if(!kdl)
    throw RunTimeException(WHERE) << "No KDL model for robot: " << robot->GetLabel();

  auto frame1 = kdl->ForwardKinematics(_c1.GetData());

  auto frame2 = kdl->ForwardKinematics(_c2.GetData());

  // Compute Pose Increment
  auto pos1 = frame1.pos;
  auto pos2 = frame2.pos;

  std::vector<double> posInc;

  for(size_t i = 0; i < pos1.size(); i++) {
    auto d = pos2[i] - pos1[i];
    posIncrements.push_back(d/numSteps);
  }

  // Compute Orientation Increment
  auto ori1 = frame1.ori;
  auto ori2 = frame2.ori;

  std::vector<double> oriInc;

  for(size_t i = 0; i < 9; i++) {
    auto d = ori2[i] - ori1[i];
    oriIncrements.push_back(d/numSteps);
  }

  // Build and check local plan
  int cdCounter = 0;
  double distance = 0;
  CfgType currentStep(_c1),
          previousStep(robot);

  for(int i = 1; i < numSteps; ++i) {
    previousStep = currentStep;
    if(!ComputeIncrement(posIncrements,oriIncrements,currentStep)) {
      _col = previousStep;
      return false;
    }

    distance += dm->Distance(previousStep,currentStep);

    // Check collision if requested.
    if(_checkCollision) {
      ++cdCounter;
      const bool inBounds = currentStep.InBounds(env->GetBoundary());
      if(!inBounds or !vc->IsValid(currentStep, id)) {
        _col = currentStep;
        return false;
      }
    }

    // Save the resolution-level path if requested.
    if(_savePath)
      _lpOutput->m_path.push_back(currentStep);
  }

  // The edge is valid. Add the distance to final cfg
  distance += dm->Distance(currentStep,_c2);

  auto& edge1 = _lpOutput->m_edge.first,
      & edge2 = _lpOutput->m_edge.second;

  edge1.SetWeight(edge1.GetWeight() + distance);
  edge2.SetWeight(edge2.GetWeight() + distance);
  edge1.SetTimeSteps(edge1.GetTimeSteps() + numSteps);
  edge2.SetTimeSteps(edge2.GetTimeSteps() + numSteps);

  return true;
}

template <typename MPTraits>
bool
ToolFrameLine<MPTraits>::
IsConnected(const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
    GroupLPOutput<MPTraits>* _lpOutput, double _positionRes,
    double _orientationRes, bool _checkCollision, bool _savePath,
    const RobotFormation& _robotIndexes) {
  const std::string id = this->GetNameAndLabel();
  auto stats = this->GetStatClass();
  MethodTimer(stats, id + "::IsConnectedFunc");

  if(this->m_debug) {
    std::cout << id
              << "\n\tChecking line from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << std::endl;
    if(!_robotIndexes.empty())
      std::cout << "\tUsing formation: " << _robotIndexes << std::endl;
  }

  // Initialize the LPOutput object.
  _lpOutput->Clear();
  _lpOutput->SetLPLabel(this->GetLabel());

  auto env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto groupMap = _c1.GetGroupRoadmap();
  auto group = groupMap->GetGroup();

  // Find tool frame increment
  std::unordered_map<Robot*,std::vector<double>> posIncrements;
  std::unordered_map<Robot*,std::vector<double>> oriIncrements;

  std::set<Robot*> accountedFor;
  std::vector<Robot*> robots;
 
  for(auto f : _c1.GetFormations()) {
    auto leader = f->GetLeader();
    robots.push_back(leader);

    for(auto r : f->GetRobots()) {
      accountedFor.insert(r);
    }
  }

  for(auto robot : group->GetRobots()) {
    if(accountedFor.count(robot))
      continue;

    robots.push_back(robot);
  }

  int numSteps;
  // Currently assume only one leader
  auto leader1 = _c1.GetRobotCfg(robots[0]);
  auto leader2 = _c2.GetRobotCfg(robots[0]);
  Cfg leaderDummy(robots[0]);
  leaderDummy.FindIncrement(leader1,leader2, &numSteps, env->GetPositionRes(), env->GetOrientationRes());

  for(auto robot : robots) {
    auto kdl = robot->GetKDLModel();
    if(!kdl)
      throw RunTimeException(WHERE) << "No KDL model for robot: " << robot->GetLabel();

    auto cfg1 = _c1.GetRobotCfg(robot);
    auto frame1 = kdl->ForwardKinematics(cfg1.GetData());

    auto cfg2 = _c2.GetRobotCfg(robot);
    auto frame2 = kdl->ForwardKinematics(cfg2.GetData());

    // Compute Pose Increment
    auto pos1 = frame1.pos;
    auto pos2 = frame2.pos;

    std::vector<double> posInc;

    for(size_t i = 0; i < pos1.size(); i++) {
      auto d = pos2[i] - pos1[i];
      posInc.push_back(d/numSteps);
    }

    posIncrements[robot] = posInc;

    // Compute Orientation Increment
    auto ori1 = frame1.ori;
    auto ori2 = frame2.ori;

    std::vector<double> oriInc;

    for(size_t i = 0; i < 9; i++) {
      auto d = ori2[i] - ori1[i];
      oriInc.push_back(d/numSteps);
    }

    oriIncrements[robot] = oriInc;
  }
 

  bool connected = true;
  int cdCounter = 0;
  GroupCfgType currentStep(_c1),
               previousStep(groupMap);
  for(int i = 1; i < numSteps; ++i) {
    previousStep = currentStep;
    currentStep = ComputeIncrement(posIncrements,oriIncrements,currentStep);

    if(!currentStep.GetGroupRoadmap()) {
      connected = false;
      break;
    }

    // Check collision if requested.
    if(_checkCollision) {
      ++cdCounter;
      const bool inBounds = currentStep.InBounds(env->GetBoundary());
      if(!inBounds or !vc->IsValid(currentStep, id)) {
        _col = currentStep;
        connected = false;
        break;
      }
    }

    // Save the resolution-level path if requested.
    if(_savePath or this->m_saveIntermediates)
      _lpOutput->m_path.push_back(currentStep);
  }

  // Set data in the LPOutput object.
  _lpOutput->m_edge.first.SetWeight(numSteps);
  _lpOutput->m_edge.second.SetWeight(numSteps);
  _lpOutput->m_edge.first.SetTimeSteps(numSteps);
  _lpOutput->m_edge.second.SetTimeSteps(numSteps);
  _lpOutput->SetIndividualEdges(_robotIndexes);
  _lpOutput->SetActiveRobots(_robotIndexes);

  if(connected)
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);

  // Track usage stats.
  stats->IncLPAttempts(id);
  stats->IncLPConnections(id, connected);
  stats->IncLPCollDetCalls(id, cdCounter);

  if(this->m_debug)
    std::cout << "\n\tLocal Plan is "
              << (connected ? "valid" : "invalid at " + _col.PrettyPrint())
              << std::endl;
  return connected;

  return false;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
ToolFrameLine<MPTraits>::
IsConnectedFunc(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  auto robot = _c1.GetRobot();
  // If there is no robot pointer, this is assumed to be an interaction edge.
  /// @todo Why are we running a local planner on an interaction edge? That
  ///       shouldn't be necessary since we need neither intermediates nor CD
  ///       information in that case?
  if(!robot)
    return true;

  Environment* env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  const std::string id = this->GetNameAndLabel() + "::IsConnectedSLSequential";

  /*

  // Compute the number of resolution-level steps required to transition from
  // _c1 to _c2.
  int numSteps;
  CfgType increment(robot);
  increment.FindIncrement(_c1, _c2, &numSteps, _positionRes, _orientationRes);
  if(this->m_debug)
    std::cout << "\n\tComputed increment for " << numSteps << " steps: "
              << increment.PrettyPrint() << std::endl;

  // Step from _c1 by a distance increment, either numSteps - 1 times or until a
  // collision is detected (-1 because we the final step will land at _c2, which
  // we don't need to check).
  CfgType currentStep = _c1,
          previousStep;
  double distance = 0;
  for(int i = 1; i < numSteps; ++i) {
    // Update the current step.
    previousStep = currentStep;
    currentStep += increment;

    // Add the step distance to the total distance.
    distance += dm->Distance(previousStep, currentStep);

    // // Check for collisions.
    // if(_checkCollision) {
    //   _cdCounter++;
    //   if(this->m_debug)
    //     std::cout << "\n\t\tChecking step " << i << " at "
    //               << currentStep.PrettyPrint()
    //               << std::endl;

    //   const bool inBounds = currentStep.InBounds(env);
    //   if(!inBounds || !vc->IsValid(currentStep, id)) {
    //     _col = currentStep;

    //     if(this->m_debug)
    //       std::cout << "\n\t\t\tINVALID" << std::endl;
    //     return false;
    //   }
    //   else if(this->m_debug)
    //     std::cout << "\n\t\t\tOK" << std::endl;
    // }


    // Check for collisions. 
    // TODO: out of bounds and obst-space are treated the same.
    // This might be problematic.

    if(_checkCollision) {
      _cdCounter++;
      if(this->m_debug)
        std::cout << "\n\t\tChecking step " << i << " at "
                  << currentStep.PrettyPrint()
                  << std::endl;

      const bool inBounds = currentStep.InBounds(env);
      bool valid = false;

      //in case the validity is switched for toggle operations.
      const bool validity = vc->GetValidity();
      
      if (inBounds)
        valid = vc->IsValid(currentStep, id);
      
      if (!valid) {
        _col = currentStep;
        _col.SetLabel("VALID", valid and validity);
        return false;
      }
      
      if(this->m_debug)
          std::cout << "\n\t\t\t" << (valid ? "VALID" : "INVALID" ) << std::endl;
    }


    // Save the resolution-level path if requested.
    if(_savePath)
      _lpOutput->m_path.push_back(currentStep);
  }

  // The edge is valid. Add the distance to the final configuration.
  distance += dm->Distance(currentStep, _c2);

  auto& edge1 = _lpOutput->m_edge.first,
      & edge2 = _lpOutput->m_edge.second;

  edge1.SetWeight(edge1.GetWeight() + distance);
  edge2.SetWeight(edge2.GetWeight() + distance);
  edge1.SetTimeSteps(edge1.GetTimeSteps() + numSteps);
  edge2.SetTimeSteps(edge2.GetTimeSteps() + numSteps);

  */

  return true;
}

template <typename MPTraits>
bool
ToolFrameLine<MPTraits>::
ComputeIncrement(const std::vector<double>& _posIncrements,
                 const std::vector<double>& _oriIncrements,
                 CfgType& _current) { 

  auto robot = _current.GetRobot();
  auto kdl = robot->GetKDLModel();

  auto frame = kdl->ForwardKinematics(_current.GetData());

  for(size_t i = 0; i < _posIncrements.size(); i++) {
    frame.pos[i] = frame.pos[i] + _posIncrements[i];
  }

  for(size_t i = 0; i < _oriIncrements.size(); i++) {
    frame.ori[i] = frame.ori[i] + _oriIncrements[i];
  }

  auto jointValues = kdl->InverseKinematics(frame.pos, frame.ori, _current.GetData());

  if(jointValues.empty()) {
    if(this->m_debug) {
      std::cout << "No joint angles found for (TODO::TYPE FRAME INFO)" << std::endl;
    }
    return false;
  }

  _current.SetData(jointValues);
  return true;
}

template <typename MPTraits>
typename ToolFrameLine<MPTraits>::GroupCfgType
ToolFrameLine<MPTraits>::
ComputeIncrement(const std::unordered_map<Robot*,std::vector<double>>& _posIncrements,
                 const std::unordered_map<Robot*,std::vector<double>>& _oriIncrements,
                 GroupCfgType& _current) { 

  std::unordered_map<Robot*,std::unique_ptr<CSpaceConstraint>> constraintMap;

  for(auto kv : _posIncrements) {
    auto robot = kv.first;
    auto kdl = robot->GetKDLModel();
    auto cfg = _current.GetRobotCfg(robot);

    auto posInc = kv.second;
    auto oriInc = _oriIncrements.at(robot);
    auto frame = kdl->ForwardKinematics(cfg.GetData());

    for(size_t i = 0; i < posInc.size(); i++) {
      frame.pos[i] = frame.pos[i] + posInc[i];
    }

    for(size_t i = 0; i < 9; i++) {
      frame.ori[i] = frame.ori[i] + oriInc[i];
    }

    auto jointValues = kdl->InverseKinematics(frame.pos, frame.ori, cfg.GetData());

    if(jointValues.empty()) {
      if(this->m_debug) {
        std::cout << "No joint angles found for (TODO::TYPE FRAME INFO)" << std::endl;
      }
      return GroupCfgType(nullptr);
    }

    cfg.SetData(jointValues);
    constraintMap[robot] = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
  }

  std::map<Robot*,const Boundary*> boundaryMap;
  for(auto& kv : constraintMap) {
    boundaryMap[kv.first] = kv.second->GetBoundary();
  }

  auto sm = this->GetSampler(m_smLabel);
  std::vector<GroupCfgType> valid;
  sm->Sample(1,100,boundaryMap,std::back_inserter(valid));

  if(valid.empty())
    GroupCfgType(nullptr);

  return valid[0];
}

#endif
