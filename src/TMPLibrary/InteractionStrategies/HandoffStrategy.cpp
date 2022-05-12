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

  // Set the active formations for the planning problem.
  auto& startConditions = _interaction->GetStageConditions(stages[1]);
  SetActiveFormations(startConditions,_interaction->GetToStageSolution(stages[2]));
  _start = GenerateInitialState(_interaction,_start,2);

  // Making assumptions that first and last stage are used to specify 
  // entry/exit modes only
  size_t finalStage = stages.size() - 1;
  for(size_t i = 2; i < finalStage; i++) {

    auto current = stages[i-1];
    auto next = stages[i];

    // Set the active formations for the planning problem.
    auto& startConditions = _interaction->GetStageConditions(current);
    SetActiveFormations(startConditions,_interaction->GetToStageSolution(next));

    // Get start constraints
    auto startConstraintMap = GenerateConstraints(_start);
    if(startConstraintMap.empty())
      return false;

    // Get path constraints
    GeneratePathConstraints(startConditions,_interaction->GetStageConditions(next));

    // Get goal constraints
    auto solution = _interaction->GetToStageSolution(next);
    auto nextState = GenerateTransitionState(_interaction,_start,i,solution);
    auto goalConstraintMap = GenerateConstraints(nextState);
    if(goalConstraintMap.empty())
      return false;

    // Get static robots
    auto staticRobots = GetStaticRobots(startConditions);

    // Solve tasks

    // Construct toNextStage tasks with current stage robot groups.
    auto toNextStageTasks = GenerateTasks(startConditions,
                                          startConstraintMap,
                                          goalConstraintMap);
    _interaction->SetToStageTasks(next,toNextStageTasks);

    // Ensure that all groups are in solution
    for(auto task : toNextStageTasks) {
      auto group = task->GetRobotGroup();
      solution->AddRobotGroup(group);
    }

    // Configure static robots as obstacles
    ConfigureStaticRobots(staticRobots,_start);

    // Compute motions from pre to interim conditions.
    auto toNextStagePaths = PlanMotions(toNextStageTasks,solution,
                  "PlanInteraction::"+_interaction->GetLabel()+"::To"+next,staticRobots,_start);

    if(toNextStagePaths.empty())
      return false;

    size_t delay = _interaction->GetDelay(next);
    if(delay > 0) {
      for(auto path : toNextStagePaths) {
        auto pair = path->VIDsWaiting();
        auto vids = pair.first;
        auto wait = pair.second;
        if(wait.empty()) {
          auto last = vids.back();
          std::vector<size_t> add(delay,last);
          *path += add;
          path->SetTimeSteps(path->TimeSteps() + delay);
        }
        else {
          wait = std::vector<size_t>(vids.size(),0);
          wait[wait.size()-1] += delay;
          path->SetWaitTimes(wait);
        }
      }
    }

    ResetStaticRobots();

    // Check if a valid solution was found.
    if(toNextStagePaths.empty())
      break;

    // Save plan information.
    _interaction->SetToStagePaths(next,toNextStagePaths);

    if(i+1 < finalStage)
      _start = InterimState(_interaction,next,stages[i+1],toNextStagePaths);
    else {
    //  _start = InterimState(_interaction,next,next,toNextStagePaths);
      _start = InterimState(_interaction,stages[i+1],next,toNextStagePaths);
    }
  }

  // Clear information from this planning run.
  m_roleMap.clear();
  m_interimCfgMap.clear();
  m_individualPaths.clear();
  m_finalState.clear();

  if(m_debug) {
    std::cout << "Final interaction state:" << std::endl;
    for(auto kv : _start) {
      std::cout << kv.first->GetLabel() << " at "
                << kv.second.first->GetVertex(kv.second.second).PrettyPrint()
                << std::endl;
    }
  }

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
    bool deliver = false;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        deliverers.push_back(group);
        deliver = true;
        break;
      }
    }
    if(!deliver)
      receivers.push_back(group);

    // Make sure all groups are in the solution
    solution->AddRobotGroup(group);
  }

  // Try for m_maxAttempts to generate a start state
  for(size_t i = 0; i < m_maxAttempts; i++) {
    State start;

    std::vector<std::pair<GroupRoadmapType*,size_t>> vidsToDelete;

    // Sample a cfg for each deliverer
    for(auto group : deliverers) {
      // Configure library 
      GroupTask task(group);
      lib->SetGroupTask(&task);
      solution->AddRobotGroup(group);
      auto rm = solution->GetGroupRoadmap(group);
      // Check formation

      GroupCfg cfg(rm);

      if(!m_physicalDemo) {
        std::vector<GroupCfg> samples;
        sampler->Sample(1,1,boundaryMap,std::back_inserter(samples));
        if(samples.empty())
          break;

        cfg = samples[0];
      }
      else {
        // Hack to get clean physical demo handoffs
        
        // Get mean distance between robots
        double x = 0;
        double y = 0;
        double z = 0;
        double count = 0;
        for(auto group : deliverers) {
          for(auto robot : group->GetRobots()) {
            if(robot->GetMultiBody()->IsPassive())
              continue;
            auto transform = robot->GetMultiBody()->GetBase()->GetWorldTransformation();
            auto translation = transform.translation();
            x += translation[0];
            y += translation[1];
            z += translation[2];
            count += 1;
          }
        }
        for(auto group : receivers) {
          for(auto robot : group->GetRobots()) {
            if(robot->GetMultiBody()->IsPassive())
              continue;
            auto transform = robot->GetMultiBody()->GetBase()->GetWorldTransformation();
            auto translation = transform.translation();
            x += translation[0];
            y += translation[1];
            z += translation[2];
            count += 1;
          }
        }

        x = x/count;
        y = y/count;
        z = z/count;

        // Get passive robot  
        Robot* object = nullptr;
        for(auto robot : group->GetRobots()) {
          if(robot->GetMultiBody()->IsPassive()) {
            object = robot;
            break;
          }
        }

        Cfg objectCfg(object);

        objectCfg[0] = x;
        //objectCfg[0] = x+.015;
        objectCfg[1] = y;
        objectCfg[2] = z+.35;
        objectCfg[3] = 0;
        objectCfg[4] = 0;
        objectCfg[5] = 0;

        cfg.SetRobotCfg(object,std::move(objectCfg));
        if(m_debug) {
          std::cout << "Physical Demo object position:  " 
                    << cfg.GetRobotCfg(object).PrettyPrint() 
                    << std::endl;
        }

      }

      auto vid = rm->AddVertex(cfg);
      vidsToDelete.emplace_back(rm,vid);
      start[group] = std::make_pair(rm,vid);
    }

    // Make sure all deliverers are given a start state
    if(start.size() != deliverers.size())
      continue;

    // Add arbitrary nodes for receivers
    for(auto group : receivers) {
      start[group] = std::make_pair(nullptr,MAX_INT);
    }

    // Attempt to create full state
    auto state = GenerateTransitionState(_interaction,start,_next-1,solution);

    if(m_physicalDemo) {
      for(auto pair : vidsToDelete) {
        pair.first->DeleteVertex(pair.second);
      }
    }
  
    if(!state.empty())
      return state;
  }

  return State();

  /*
  // Try for m_maxAttempts to generate a start state
  for(size_t i = 0; i < m_maxAttempts; i++) {
    State start;

    std::map<Robot*,Cfg> objectPoses;

    // Sample cfgs for deliverers
    for(auto group : deliverers) {
      // Configure library 
      GroupTask task(group);
      lib->SetGroupTask(&task);
      solution->AddRobotGroup(group);
      auto rm = solution->GetGroupRoadmap(group);
      // Check formation
      

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
      solution->AddRobotGroup(group);
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

    if(start.size() == _previous.size()) {
      if(m_debug) {
        std::cout << "Generating start state as follows:" << std::endl;
        for(auto kv : start) {
          std::cout << kv.first->GetLabel() 
                    << kv.second.first->GetVertex(kv.second.second).PrettyPrint()
                    << std::endl;
        }
      }
      return start;
    }
  }

  return State();
  */
}

HandoffStrategy::State
HandoffStrategy::
GenerateTransitionState(Interaction* _interaction, const State& _previous, const size_t _next, MPSolution* _solution) {

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto stages = _interaction->GetStages();

  // Seprate deliverers and receivers
  std::vector<Robot*> deliverers;
  std::vector<Robot*> receivers;
  for(auto kv : _previous) {
    auto group = kv.first;
    bool deliverer = false;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        deliverer = true;
        break;
      }
    }
    if(!deliverer) {
      for(auto robot : group->GetRobots()) {
        receivers.push_back(robot);
      }
    }
    else {
      for(auto robot : group->GetRobots()) {
        deliverers.push_back(robot);
      }
    }
  }

  // Collect constraint boundaries
  std::map<Robot*,const Boundary*> boundaryMap;
  auto conditions = _interaction->GetStageConditions(stages[_next]);
  for(auto c : conditions) {
    auto m = dynamic_cast<MotionCondition*>(as->GetCondition(c));
    if(!m)
      continue;

    for(auto role : m->GetRoles()) {
      auto robot = m_roleMap[role];
      auto constraint = m->GetRoleConstraint(role);
      auto boundary = constraint->GetBoundary();
      boundaryMap[robot] = boundary;
    }
  }

  // Grab object poses from previous state
  std::map<Robot*,Cfg> objectPoses;
  for(auto kv : _previous) {
    auto group = kv.first;
    auto grm = kv.second.first;
    if(!grm)
      continue;
    auto gcfg = grm->GetVertex(kv.second.second);
    gcfg.ConfigureRobot();
    for(auto robot : group->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive())
        continue;
      
      objectPoses[robot] = gcfg.GetRobotCfg(robot);
    }
  }

  //for(size_t i = 0; i < m_maxAttempts; i++) {
  for(size_t i = 0; i < 1; i++) {

    std::map<Robot*,Cfg> sampleMap;
    bool failed = false;

    // Sample pregrasp joint angles
    auto eeFrames = ComputeEEFrames(_interaction,objectPoses,_next);

    for(auto kv : eeFrames) {

      auto cfg = ComputeManipulatorCfg(kv.first,kv.second);
      if(!cfg.GetRobot()) {
        if(m_debug) {
          std::cout << "Failed to find a valid grasp pose for " << kv.first->GetLabel() << std::endl;
        }
      	failed = true;
        break;
      }

      SetEEDOF(_interaction,cfg,stages[_next]);
      sampleMap[kv.first] = cfg;
    }

    if(failed)
      continue;

    // If !failed, create return state
    // Convert samples cfgs into group cfgs
    State transition;
    for(auto kv : _previous) {
      auto group = kv.first;
      auto grm = _solution->GetGroupRoadmap(group);
      GroupCfg gcfg(grm);
      for(auto robot : group->GetRobots()) {
        Cfg cfg(robot);
        if(robot->GetMultiBody()->IsPassive())
          cfg = objectPoses[robot];
        else 
          cfg = sampleMap[robot];
        gcfg.SetRobotCfg(robot,std::move(cfg));
      }
      auto vid = grm->AddVertex(gcfg);
      transition[group] = std::make_pair(grm,vid);
    }

    return transition;
  }

  return State();
}
    
std::vector<std::shared_ptr<GroupTask>> 
HandoffStrategy::
GenerateTasks(std::vector<std::string> _conditions, 
              std::unordered_map<Robot*,Constraint*> _startConstraints,
              std::unordered_map<Robot*,Constraint*> _goalConstraints) {

  // As all robots are specified with a formation involving the object
  // as a child node, the underlying generate tasks will create overlapping
  // groups with varied motions for the object.
  // In this function, all static robots will be put into one group, and
  // all active robots will be given their own task.

  // TODO::Decide if active robots should be one composite task or individual tasks
  
  auto staticRobots = GetStaticRobots(_conditions);
  
  // Collect robot groups and labels
  std::vector<Robot*> sr; // static robots
  std::vector<Robot*> ar; // active robots

  std::string staticLabel;
  std::string activeLabel;

  for(auto kv : _startConstraints) {
    auto robot = kv.first; 
    if(staticRobots.count(robot)) {
      sr.push_back(robot);
      staticLabel = staticLabel + robot->GetLabel() + "--";
    }
    else {
      ar.push_back(robot);
      activeLabel = activeLabel + robot->GetLabel() + "--";
    }
  }

  // Add robot groups to problem
  auto problem = this->GetMPProblem();
  auto activeGroup = problem->AddRobotGroup(sr,staticLabel);
  auto staticGroup = problem->AddRobotGroup(ar,activeLabel);

  // Create GroupTask for each group
  auto staticTask = std::shared_ptr<GroupTask>(new GroupTask(staticGroup));
  
  for(auto robot : staticGroup->GetRobots()) {
    MPTask task(robot);
    task.SetStartConstraint(std::move(_startConstraints[robot]->Clone()));
    task.AddGoalConstraint(std::move(_goalConstraints[robot]->Clone()));

    auto iter = m_pathConstraintMap.find(robot);
    if(iter != m_pathConstraintMap.end()) {
      task.AddPathConstraint(std::move(m_pathConstraintMap[robot]->Clone()));
    }

    staticTask->AddTask(task);
  }

  auto activeTask = std::shared_ptr<GroupTask>(new GroupTask(activeGroup));
  
  for(auto robot : activeGroup->GetRobots()) {
    MPTask task(robot);
    task.SetStartConstraint(std::move(_startConstraints[robot]->Clone()));
    task.AddGoalConstraint(std::move(_goalConstraints[robot]->Clone()));

    auto iter = m_pathConstraintMap.find(robot);
    if(iter != m_pathConstraintMap.end()) {
      task.AddPathConstraint(std::move(m_pathConstraintMap[robot]->Clone()));
    }

    activeTask->AddTask(task);
  }

  m_pathConstraintMap.clear();

  return {staticTask,activeTask};
}

void
HandoffStrategy::
GeneratePathConstraints(std::vector<std::string> _startConditions, 
                        std::vector<std::string> _endConditions) {

  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Check if any motion conditions exist in both condition sets
  for(auto label1 : _startConditions) {
    auto c = as->GetCondition(label1);
    auto m = dynamic_cast<MotionCondition*>(c);
    if(!m)
      continue;

    for(auto label2 : _endConditions) {
      if(label1 != label2)
        continue;

      auto constraints = m->GetConstraints();
      for(auto pair : constraints) {
        auto constraint = pair.second;
        auto role = m->GetRole(constraint);

        auto robot = m_roleMap[role];
        if(!robot)
          throw RunTimeException(WHERE) << "Generating path constraint for role without a robot.";

        m_pathConstraintMap[robot] = constraint;
        m_pathConstraintMap[robot]->SetRobot(robot);
      }

      break;
    }
  }
}
/*------------------------------------------------------------*/
