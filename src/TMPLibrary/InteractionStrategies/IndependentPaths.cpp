#include "IndependentPaths.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"

/*---------------------------- Construction --------------------------*/

IndependentPaths::
IndependentPaths() {
  this->SetName("IndependentPaths");
}

IndependentPaths::
IndependentPaths(XMLNode& _node) : InteractionStrategyMethod(_node) {
  this->SetName("IndependentPaths");
  m_mpStrategyLabel = _node.Read("mpStrategy", true, "",
                 "MPStrategy to use to plan interactions.");

  m_sequential = _node.Read("sequential",false,m_sequential,
                 "Flag indiciating if the paths should be sequentially planned to avoid each other.");
}

IndependentPaths::
~IndependentPaths() {}

/*------------------------------ Interface ----------------------------*/

bool
IndependentPaths::
operator()(Interaction* _interaction, State& _state) {

  if(m_debug) {
    std::cout << "Planning interaction: " 
              << _interaction->GetLabel() 
              << std::endl;
  }

  bool foundSolution = false;

  _interaction->Initialize();

  bool validStart = true;
  for(auto kv : _state) {
    if(!kv.second.first or kv.second.second == MAX_INT) {
      validStart = false;
      break;
    }
  }

  if(validStart) {
    MoveStateToLocalSolution(_interaction,_state);
  }
  else {
    SampleStartState(_interaction,_state);
  }

  SetInteractionBoundary(_interaction,_state);

  // Assign interaction roles.
  const auto& stages = _interaction->GetStages();
  auto& preconditions = _interaction->GetStageConditions(stages[0]);
  AssignRoles(_state,preconditions);

  // Iterate through stages and plan motions between them.
  for(size_t i = 1; i < stages.size(); i++) {
    const auto& current = stages[i-1];
    const auto& next = stages[i];

    // Collect robot groups
    std::vector<RobotGroup*> groups;
    for(auto kv : _state) {
      groups.push_back(kv.first);
    }

    // Construct start constraint map from current state.
    auto startConstraintMap = GenerateConstraints(_state);

    // Construct goal constraint map from next state.
    auto& startConditions = _interaction->GetStageConditions(current);
    auto staticRobots = GetStaticRobots(startConditions);
    auto& goalConditions = _interaction->GetStageConditions(next);
    auto goalConstraintMap = GenerateConstraints(goalConditions,groups,
                                                 _state,staticRobots);

    // Check that there are was goal constraint created.
    if(goalConstraintMap.empty())
      break;

    // Set the active formations for the planning problem.
    SetActiveFormations(startConditions,_interaction->GetToStageSolution(next));

    // Construct toNextStage tasks with current stage robot groups.
    auto toNextStageTasks = GenerateTasks(startConditions,
                                          startConstraintMap,
                                          goalConstraintMap);
    _interaction->SetToStageTasks(next,toNextStageTasks);

    // Configure static robots as obstacles
    ConfigureStaticRobots(staticRobots,_state);

    // Compute motions from pre to interim conditions.
    auto toNextStagePaths = PlanMotions(toNextStageTasks,_interaction->GetToStageSolution(next),
                  "PlanInteraction::"+_interaction->GetLabel()+"::To"+next,staticRobots,_state);

    ResetStaticRobots(staticRobots);

    // Check if a valid solution was found.
    if(toNextStagePaths.empty())
      break;

    // Save plan information.
    _interaction->SetToStagePaths(next,toNextStagePaths);

    if(i+1 < stages.size())
      _state = InterimState(_interaction,next,stages[i+1],toNextStagePaths);
    else {
      _state = InterimState(_interaction,next,next,toNextStagePaths);
      foundSolution = true;
    }
  }

  // Thomas - I commented this out bc it will make a mess on my problems. 
  // It looks like some kind of debug output. Please wrap it in a if(m_deug)
  // if that is the case.
  /*
  //Creating a roadmap for each stage in the interaction.
  //How to combine this into one roadmap?
  for (size_t i = 0; i < stages.size(); i++) {
        auto sol = _interaction->GetToStageSolution(stages[i]);
        auto& startConditions = _interaction->GetStageConditions(stages[i]);
        auto Robots = GetRobots(startConditions);   
        for (auto Robot : Robots){  
          if (Robot!=0){
                auto rmp = sol->GetRoadmap(Robot);
		auto lbl = Robot->GetLabel();
		if (rmp!=0) rmp->Write(stages[i]+"_"+Robot->GetLabel()+".map", this->GetMPProblem()->GetEnvironment());
          }
        }
  }
  */
  //auto sg = dynamic_cast<CombinedRoadmap*> this->GetStateGraph(m_sgLabel).get();
  //sg->Write("handoffTemplateTest2"+".map", this->GetMPProblem()->GetEnvironment());
  
  // Clear information from this planning run.
  m_roleMap.clear();
  m_interimCfgMap.clear();
  m_individualPaths.clear();
  m_finalState.clear();
           
  return foundSolution;


  /*
  // Construct start constraints.
  //auto startConstraintMap = GenerateConstraints(preconditions);

  // Construct start constraints from initial state.
  auto startConstraintMap = GenerateConstraints(_state);

  // Construct interim constraints.
  auto& interimConditions = _interaction->GetInterimConditions();
  SetActiveFormations(preconditions,_interaction->GetToInterimSolution());
  auto interimConstraintMap = GenerateConstraints(interimConditions,groups);

  if(interimConstraintMap.empty())
    return false;

  // Construct toInterim tasks with precondition robot groups.
  auto toInterim = GenerateTasks(preconditions,
                                 startConstraintMap,
                                 interimConstraintMap);

  // Compute motions from pre to interim conditions.
  //auto toInterimPath = PlanMotions(toInterim,_interaction->GetToInterimSolution(),
  //                "PlanInteraction::"+_interaction->GetLabel()+"::ToInterim");
  auto toInterimPaths = PlanMotions(toInterim,_interaction->GetToInterimSolution(),
                  "PlanInteraction::"+_interaction->GetLabel()+"::ToInterim");

  //if(!toInterimPath)
  if(toInterimPaths.empty())
    return false;

  auto& postconditions = _interaction->GetPostConditions();
  SetActiveFormations(postconditions,_interaction->GetToPostSolution());

  _interaction->SetToInterimPaths(toInterimPaths);

  // Extract last cfg from path and use to start post task.
  // Currently, the state is both computed and added in this function.
  auto state = InterimState(_interaction);

  // Construct goal constraints.
  auto goalConstraintMap = GenerateConstraints(postconditions,groups);

  if(goalConstraintMap.empty())
    return false;

  // Construct toPost tasks with postcondition robot groups.
  auto toPost = GenerateTasks(postconditions,
                              interimConstraintMap,
                              goalConstraintMap);

  // Compute motions from interim to post conditions.
  //auto toPostPath = PlanMotions(toPost,_interaction->GetToPostSolution(),
  //                   "PlanInteraction::"+_interaction->GetLabel()+"::ToPost"); 
  auto toPostPaths = PlanMotions(toPost,_interaction->GetToPostSolution(),
                     "PlanInteraction::"+_interaction->GetLabel()+"::ToPost"); 

  //if(!toPostPath)
  if(toPostPaths.empty())
    return false;

  _interaction->SetToPostPaths(toPostPaths);

  _state = m_finalState;

  m_roleMap.clear();
  m_interimCfgMap.clear();
  m_individualPaths.clear();
  m_finalState.clear();

  return true;
  */
}

/*--------------------------- Helper Functions ------------------------*/

std::vector<std::shared_ptr<GroupTask>>
IndependentPaths::
GenerateTasks(std::vector<std::string> _conditions, 
              std::unordered_map<Robot*,Constraint*> _startConstraints,
              std::unordered_map<Robot*,Constraint*> _goalConstraints) {

  //auto hcr = dynamic_cast<CombinedRoadmap*>(this->GetStateGraph(m_sgLabel).get());
  auto as = this->GetTMPLibrary()->GetActionSpace();
  std::vector<std::shared_ptr<GroupTask>> groupTasks;

  // Create a task for each required robot group
  for(auto label : _conditions) {
    auto condition = as->GetCondition(label);

    // Check that condition is a formation condition
    auto f = dynamic_cast<FormationCondition*>(condition);
    if(!f)
      continue;

    auto roles = f->GetRoles();
    std::vector<Robot*> robots;
    std::string groupLabel = "";
    for(auto role : roles) {
      auto robot = m_roleMap[role];
      robots.push_back(robot);
      groupLabel += (role + ":" + robot->GetLabel() + "--");
    }

    RobotGroup* group = this->GetMPProblem()->AddRobotGroup(robots,groupLabel);
    // Check if group is truly new and update MPSolution
    //if(group->GetLabel() == groupLabel) {
    //  hcr->AddRobotGroup(group);
    //}
   
    auto groupTask = shared_ptr<GroupTask>(new GroupTask(group));

    // Generate individual robot tasks
    for(auto robot : robots) {
      MPTask task(robot);

      auto start = _startConstraints[robot]->Clone();
      start->SetRobot(robot);
      task.SetStartConstraint(std::move(start));

      auto goal = _goalConstraints[robot]->Clone();
      goal->SetRobot(robot);
      task.AddGoalConstraint(std::move(goal));

      groupTask->AddTask(task);
    }
 
    groupTasks.push_back(groupTask);
  }

  return groupTasks;
}

std::vector<IndependentPaths::Path*>
IndependentPaths::
PlanMotions(std::vector<std::shared_ptr<GroupTask>> _tasks, MPSolution* _solution, std::string _label,
            const std::set<Robot*>& _staticRobots, const State& _state) {

  // Grab MPSolution, MPLibrary, and MPProblem.
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();

  // Clear previous final state.
  m_finalState.clear();

  // TODO::Refactor DO code to effeciently save copy of DOs while working in here
  if(!prob->GetDynamicObstacles().empty())
    throw RunTimeException(WHERE) << "Making assumption that there are no existing dynamic obstacles.";

  std::vector<Path*> paths;

  for(auto task : _tasks) {

    // TODO::Ensure that static groups always have a path length of 1.

    auto group = task->GetRobotGroup();
    bool isStatic = false;
    for(auto robot : group->GetRobots()) {
      if(_staticRobots.count(robot)) {
        isStatic = true;
        break;
      }
    }

    if(isStatic) {
      std::vector<size_t> vids = {_state.at(group).second};
      auto groupPath = _solution->GetGroupPath(group);
      groupPath->Clear();
      *groupPath += vids;
    }
    else {
      // Plan normal path for all other robots

      // Set group non-virtual
      for(auto robot : group->GetRobots()) {
        //_solution->AddRobot(robot);
        robot->SetVirtual(false);
      }
      //_solution->AddRobotGroup(group);
      
      // Call the MPLibrary solve function to expand the roadmap
      for(auto robot : task.get()->GetRobotGroup()->GetRobots()) {
        robot->SetVirtual(false);
      }
      lib->SetTask(nullptr);
      lib->Solve(prob,task.get(),_solution,m_mpStrategyLabel, LRand(),_label);
    }

    // Check for solution
    // Check composite path
    auto grm = _solution->GetGroupRoadmap(group);
    auto groupPath = _solution->GetGroupPath(group);
    if(groupPath and !groupPath->Empty()) {
      if(m_debug) {
        std::cout << "Cfg Path for " << _label << ": " << group->GetLabel() << std::endl;
        for(auto cfg : groupPath->FullCfgs(lib)) {
          std::cout << cfg.PrettyPrint() << std::endl;
        }
      }

      // Save group cfg at the end of the path.
      auto lastVID = groupPath->VIDs().back();
      auto last = grm->GetVertex(lastVID);

      m_finalState[group] = std::make_pair(grm,lastVID);

      // Save individual robot cfgs at the end of the path.
      for(auto robot : group->GetRobots()) {
        auto cfg = last.GetRobotCfg(robot);
        m_interimCfgMap[robot] = cfg;

      }

      // Extract into individual robot paths
      auto decoupledPaths = DecouplePath(_solution,groupPath);
      for(auto p : decoupledPaths) {
        paths.push_back(p);

        if(m_sequential) {
          DynamicObstacle dynamic(p->GetRobot(),p->Cfgs());
          prob->AddDynamicObstacle(std::move(dynamic));
        }

      }
      continue;
    }
    else if(groupPath and groupPath->Empty()) {
      return {};
    }

    // Check decoupled paths.
    GroupCfgType gcfg(grm);

    for(auto robot : group->GetRobots()) {
      auto path = _solution->GetPath(robot);
      if(path->Empty()) {
        //return nullptr;
        prob->ClearDynamicObstacles();
        return {};
      }

      // Save last cfg in path.
      auto lastVID = path->VIDs().back();
      auto rm = _solution->GetRoadmap(robot);
      auto cfg = rm->GetVertex(lastVID);
      m_interimCfgMap[robot] = cfg;

      gcfg.SetRobotCfg(robot,lastVID);

      paths.push_back(path);
    }

    // Save group cfg at the end of the path.
    auto groupVID = grm->AddVertex(gcfg);
    m_finalState[group] = std::make_pair(grm,groupVID);

    // Reset nonstatic robots as virtual
    for(auto robot : group->GetRobots()) {
      if(!_staticRobots.count(robot) and !m_sequential)
        robot->SetVirtual(true);
    }
  }

  prob->ClearDynamicObstacles();
  return paths;
}

IndependentPaths::State
IndependentPaths::
InterimState(Interaction* _interaction, const std::string& _current,
             const std::string& _next, const std::vector<Path*> _paths) {

  State interimState;

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto prob = this->GetMPProblem();
  auto solution = _interaction->GetToStageSolution(_next);

  auto& conditions = _interaction->GetStageConditions(_current);

  for(auto conditionLabel : conditions) {
    auto condition = as->GetCondition(conditionLabel);

    // Check that this is a formation condition
    auto f = dynamic_cast<FormationCondition*>(condition);
    if(!f)
      continue;

    // Collect robots from assigned roles
    std::vector<Robot*> robots;
    std::string groupLabel = "";

    std::unordered_map<std::string,Robot*> roleMap;
    for(auto role : f->GetRoles()) {
      auto robot = m_roleMap[role];
      robots.push_back(robot);
      groupLabel += (role + ":" + robot->GetLabel() + "--");
      roleMap[role] = robot;
    }

    // Intialize group
    RobotGroup* group = prob->AddRobotGroup(robots,groupLabel);
    for(auto robot : group->GetRobots()) {
      solution->AddRobot(robot);
    }
    solution->AddRobotGroup(group);

    // Set formations
    auto grm = solution->GetGroupRoadmap(group);
    grm->SetAllFormationsInactive();
    auto formation = f->GenerateFormation(roleMap);
    if(formation)
      grm->AddFormation(formation,true);

    // Create initial group vertex
    auto gcfg = GroupCfgType(grm);

    // Add initial cfg to individual roadmaps
    for(auto& robot : group->GetRobots()) {
      auto rm = solution->GetRoadmap(robot);
      auto cfg = m_interimCfgMap[robot];
      auto vid = rm->AddVertex(cfg);
     
      // Update group vertex 
      gcfg.SetRobotCfg(robot,vid);
    }

    // Add intial group vertex and add to state
    auto gvid = grm->AddVertex(gcfg);
    interimState[group] = std::make_pair(grm,gvid);
  }
 
  return interimState; 
}
