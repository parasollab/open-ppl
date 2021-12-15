#include "HandoffStrategy.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"

/*----------------------- Construction -----------------------*/

HandoffStrategy::
HandoffStrategy() {
  this->SetName("HandoffStrategy");
}

HandoffStrategy::
HandoffStrategy(XMLNode& _node) : GraspStrategy(_node) {
  this->SetName("HandoffStrategy");
}

HandoffStrategy::
~HandoffStrategy() {}

/*------------------------ Interface -------------------------*/

bool
HandoffStrategy::
operator()(Interaction* _interaction, State& _start) {

  _interaction->Initialize();
  auto stages = _interaction->GetStages();

  // Assign initial roles
  auto initialConditions = _interaction->GetStageConditions(stages[0]);
  AssignRoles(_start,initialConditions);

  _start = GenerateInitialState(_interaction,_start,1);

  // Making assumptions that first and last stage are used to specify 
  // entry/exit modes only
  size_t finalStage = stages.size() - 1;
  for(size_t i = 2; i < finalStage; i++) {

    auto current = stages[i-1];
    auto next = stages[i];

    // Get start constraints
    auto startConstraintMap = GenerateConstraints(_start);
    if(startConstraintMap.empty())
      return false;

    // Get goal constraints
    auto nextState = GenerateTransitionState(_interaction,_start,i);
    auto goalConstraintMap = GenerateConstraints(nextState);
    if(goalConstraintMap.empty())
      return false;

    // Get static robots
    auto& startConditions = _interaction->GetStageConditions(current);
    auto staticRobots = GetStaticRobots(startConditions);

    // Solve tasks
    // Set the active formations for the planning problem.
    SetActiveFormations(startConditions,_interaction->GetToStageSolution(next));

    // Construct toNextStage tasks with current stage robot groups.
    auto toNextStageTasks = GenerateTasks(startConditions,
                                          startConstraintMap,
                                          goalConstraintMap);
    _interaction->SetToStageTasks(next,toNextStageTasks);

    // Configure static robots as obstacles
    ConfigureStaticRobots(staticRobots,_start);

    // Compute motions from pre to interim conditions.
    auto toNextStagePaths = PlanMotions(toNextStageTasks,_interaction->GetToStageSolution(next),
                  "PlanInteraction::"+_interaction->GetLabel()+"::To"+next,staticRobots,_start);

    if(toNextStagePaths.empty())
      return false;

    ResetStaticRobots();

    // Check if a valid solution was found.
    if(toNextStagePaths.empty())
      break;

    // Save plan information.
    _interaction->SetToStagePaths(next,toNextStagePaths);

    if(i+1 < stages.size())
      _start = InterimState(_interaction,next,stages[i+1],toNextStagePaths);
    else {
      _start = InterimState(_interaction,next,next,toNextStagePaths);
    }
  }

  // Clear information from this planning run.
  m_roleMap.clear();
  m_interimCfgMap.clear();
  m_individualPaths.clear();
  m_finalState.clear();

  return true;
}

/*--------------------- Helper Functions ---------------------*/

HandoffStrategy::State
HandoffStrategy::
GenerateInitialState(Interaction* _interaction, const State& _previous, const size_t _next) {

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto lib = this->GetMPLibrary();
  lib->SetTask(nullptr);
  auto sampler = lib->GetSampler(m_smLabel);
  auto stages = _interaction->GetStages();
  auto solution = _interaction->GetToStageSolution(stages[_next]);
  lib->SetMPSolution(solution);

  // Initialize boundary map from interaction conditions
  std::map<Robot*,const Boundary*> boundaryMap;
  for(const auto label : _interaction->GetStageConditions(stages[_next])) {
    auto constraint = as->GetCondition(label);

    // Check if the condition is a motion condition.
    auto m = dynamic_cast<MotionCondition*>(constraint);
    if(!m)
      continue;
    // Match constraints to robots.
    const auto& constraints = m->GetConstraints();
    for(const auto c : constraints) {

      // Check if the constraint role has been assigned.
      auto role = m->GetRole(c.second);
      auto iter = m_roleMap.find(role);
      if(iter == m_roleMap.end())
        throw RunTimeException(WHERE) << "Role "
                                      << role
                                      << " is unassigned.";
      
      auto robot = m_roleMap[role];
      boundaryMap[robot] = c.second->GetBoundary();
    }
  }

  // Identify who are the deliverers and who are the receivers
  std::vector<RobotGroup*> deliverers;
  std::vector<RobotGroup*> receivers;
  for(auto kv : _previous) {
    auto group = kv.first;
    bool deliver;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        deliverers.push_back(group);
        deliver = true;
        break;
      }
    }
    if(!deliver)
      receivers.push_back(group);
  }

  // Try for m_maxAttempts to generate a start state
  for(size_t i = 0; i < m_maxAttempts; i++) {
    State start;

    std::map<Robot*,Cfg> objectPoses;

    // Sample cfgs for deliverers
    for(auto group : deliverers) {
      // Configure library 
      GroupTask task(group);
      lib->SetGroupTask(&task);
      auto rm = solution->GetGroupRoadmap(group);
      // TODO::Check formation

      std::vector<GroupCfg> samples;
      sampler->Sample(1,1,boundaryMap,std::back_inserter(samples));
      if(samples.empty())
        break;

      auto cfg = samples[0];
      auto vid = rm->AddVertex(cfg);
      start[group] = std::make_pair(rm,vid);

      // Save object poses
      for(auto robot : group->GetRobots()) {
        if(!robot->GetMultiBody()->IsPassive())
          continue;
        
        objectPoses[robot] = cfg.GetRobotCfg(robot);
      }
    }

    // Make sure all deliverers are given a start state
    if(start.size() != deliverers.size())
      continue;

    // Sample cfgs for receivers

    bool failed = false;
    // Assuming only a single object in handoff
    auto eeFrames = ComputeEEFrames(_interaction,objectPoses,_next);
    for(auto group : receivers) {
      auto rm = solution->GetGroupRoadmap(group);
      GroupCfg gcfg(rm);

      for(auto robot : group->GetRobots()) {
       auto cfg = ComputeManipulatorCfg(robot,eeFrames[robot]);
       if(!cfg.GetRobot()) {
         failed = true;
         break;
       }
       gcfg.SetRobotCfg(robot,std::move(cfg));
      }
      if(failed)
        break;

      auto vid = rm->AddVertex(gcfg);
      start[group] = std::make_pair(rm,vid);
    }

    if(start.size() == _previous.size())
      return start;
  }

  return State();
}

HandoffStrategy::State
HandoffStrategy::
GenerateTransitionState(Interaction* _interaction, const State& _previous, const size_t _next) {



  return State();
}
/*------------------------------------------------------------*/
