#include "InteractionStrategyExample.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"

/*----------------------- Construction -----------------------*/

InteractionStrategyExample::
InteractionStrategyExample() {
  this->SetName("InteractionStrategyExample");
}

InteractionStrategyExample::
InteractionStrategyExample(XMLNode& _node) 
            : InteractionStrategyMethod(_node) {
  this->SetName("InteractionStrategyExample");

  m_mpStrategyLabel = _node.Read("mpStrategy", true, "",
                 "MPStrategy to use to plan interactions.");
}

InteractionStrategyExample::
~InteractionStrategyExample() {}

/*------------------------ Interface -------------------------*/

bool 
InteractionStrategyExample::
operator()(Interaction* _interaction, const State& _state) {

  // Assign interaction roles.
  auto& preconditions = _interaction->GetPreConditions();
  AssignRoles(_state,preconditions);

  // Construct start constraints.
  auto startConstraintMap = GenerateConstraints(preconditions);

  // Construct interim constraints.
  auto& interimConditions = _interaction->GetInterimConditions();
  auto interimConstraintMap = GenerateConstraints(interimConditions);

  // Construct toInterim tasks with precondition robot groups.
  auto toInterim = GenerateTasks(preconditions,
                                 startConstraintMap,
                                 interimConstraintMap);

  // Compute motions from pre to interim conditions.
  if(!PlanMotions(toInterim,_interaction->GetToInterimSolution(),
                  "PlanInteraction::"+_interaction->GetLabel()+"::ToInterim"))
    return false;

  // Extract last cfg from path and use to start post task.
  // Currently, the state is both computed and added in this function.
  auto state = InterimState(_interaction);

  // Construct goal constraints.
  auto& postconditions = _interaction->GetPostConditions();
  auto goalConstraintMap = GenerateConstraints(postconditions);

  // Construct toPost tasks with postcondition robot groups.
  auto toPost = GenerateTasks(postconditions,
                              interimConstraintMap,
                              goalConstraintMap);

  // Compute motions from interim to post conditions.
  return PlanMotions(toPost,_interaction->GetToPostSolution(),
                     "PlanInteraction::"+_interaction->GetLabel()+"::ToPost"); 
}

/*--------------------- Helper Functions ---------------------*/

void
InteractionStrategyExample::
AssignRoles(const State& _state, std::vector<std::string> _conditions) {

  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Find relevant conditions
  std::unordered_map<RobotGroup*,std::vector<Condition*>> conditionMap;
 
  for(auto label : _conditions) {
    auto condition = as->GetCondition(label);

    auto group = condition->Satisfied(_state);
    if(!group)
      throw RunTimeException(WHERE) << "Interaction conditions "
                                    << "not satisfied by input state.";

    conditionMap[group].push_back(condition);
  }

  // Identify roles
  for(auto kv : conditionMap) {
    auto group = kv.first;
    auto robots = group->GetRobots();
    auto conditions = kv.second;

    auto groupRoadmap = _state.at(group).first;
    auto groupVID = _state.at(group).second;

    auto groupCfg = groupRoadmap->GetVertex(groupVID);

    for(auto condition : conditions) {
      // Check if condition is a motion condition
      auto m = dynamic_cast<MotionCondition*>(condition);
      if(!m) 
        continue;

      // Match constraints to robots
      const auto& constraints = m->GetConstraints();
      for(const auto& c : constraints) {

        auto role = m->GetRole(c.second.get());
        // Find the right robot for the role
        auto type = c.first;
        for(auto robot : robots) {
          // Check that robot is of the right type
          if(robot->GetCapability() != type) 
            continue;

          // Check that robot cfg matches the constraint
          auto cfg = groupCfg.GetRobotCfg(robot);
          if(!c.second->Satisfied(cfg)) 
            continue;

          m_roleMap[role] = robot;
        }
      }
    }
  }
}

std::unordered_map<Robot*,Constraint*>
InteractionStrategyExample::
GenerateConstraints(std::vector<std::string> _conditions) {

  auto as = this->GetTMPLibrary()->GetActionSpace();
/*
  // Find relevant conditions
  std::unordered_map<RobotGroup*,std::vector<Condition*>> conditionMap;
 
  for(auto label : _conditions) {
    auto condition = as->GetCondition(label);

    auto group = condition->Satisfied(_state);
    if(!group)
      throw RunTimeException(WHERE) << "Interaction conditions "
                                    << "not satisfied by input state.";

    conditionMap[group].push_back(condition);
  }*/

  // Convert conditions to constraints
  std::unordered_map<Robot*,Constraint*> constraintMap;

  /*for(auto kv : conditionMap) {
    auto group = kv.first;
    auto robots = group->GetRobots();
    auto conditions = kv.second;

    auto groupRoadmap = _state.at(group).first;
    auto groupVID = _state.at(group).second;

    auto groupCfg = groupRoadmap->GetVertex(groupVID);
*/
    for(auto conditionLabel : _conditions) {
      auto condition = as->GetCondition(conditionLabel);
      // Check if condition is a motion condition
      auto m = dynamic_cast<MotionCondition*>(condition);
      if(!m) 
        continue;

      // Match constraints to robots
      const auto& constraints = m->GetConstraints();
      for(const auto& c : constraints) {

        // Check if condition role has been assigned
        auto role = m->GetRole(c.second.get());
        auto iter = m_roleMap.find(role);
        if(iter != m_roleMap.end()) {
          auto robot = m_roleMap[role];
          constraintMap[robot] = c.second.get();
          continue;
        }
        throw RunTimeException(WHERE) << "Role "
                                      << role 
                                      << " is unassigned."
                                      << std::endl;
/*
        // Find the right robot for the role
        auto type = c.first;
        for(auto robot : robots) {
          // Check that robot is of the right type
          if(robot->GetCapability() != type) 
            continue;

          // Check that robot cfg matches the constraint
          auto cfg = groupCfg.GetRobotCfg(robot);
          if(!c.second->Satisfied(cfg)) 
            continue;

          m_roleMap[role] = robot;
          constraintMap[robot] = c.second.get();
        }*/
      }
    }

  //}

  return constraintMap;
}

std::vector<GroupTask*>
InteractionStrategyExample::
GenerateTasks(std::vector<std::string> _conditions, 
              std::unordered_map<Robot*,Constraint*> _startConstraints,
              std::unordered_map<Robot*,Constraint*> _goalConstraints) {

  auto hcr = dynamic_cast<CombinedRoadmap*>(this->GetStateGraph(m_sgLabel).get());
  auto as = this->GetTMPLibrary()->GetActionSpace();
  std::vector<GroupTask*> groupTasks;

  // Create a task for each required robot group
  for(auto label : _conditions) {
    auto condition = as->GetCondition(label);

    // Check that condition is a formation condition
    auto a = dynamic_cast<FormationCondition*>(condition);
    if(!a)
      continue;

    auto roles = a->GetRoles();
    std::vector<Robot*> robots;
    std::string groupLabel = "";
    for(auto role : roles) {
      auto robot = m_roleMap[role];
      robots.push_back(robot);
      groupLabel += (role + ":" + robot->GetLabel() + "--");
    }

    RobotGroup* group = this->GetMPProblem()->AddRobotGroup(robots,groupLabel);
    // Check if group is truly new and update MPSolution
    if(group->GetLabel() == groupLabel) {
      hcr->AddRobotGroup(group);
    }
   
    GroupTask* groupTask = new GroupTask(group);

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

bool
InteractionStrategyExample::
PlanMotions(std::vector<GroupTask*> _tasks, MPSolution* _solution, std::string _label) {

  // Grab MPSolution, MPLibrary, and MPProblem
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();

  for(auto task : _tasks) {

    // Add group to MPSolution
    auto group = task->GetRobotGroup();
    for(auto robot : group->GetRobots()) {
      _solution->AddRobot(robot);
    }
    _solution->AddRobotGroup(group);
    
    // Call the MPLibrary solve function to expand the roadmap
    lib->Solve(prob,task,_solution,m_mpStrategyLabel, LRand(),_label);

    // Check for solution
    // Check composite path
    auto groupPath = _solution->GetGroupPath(group);
    if(groupPath and !groupPath->Empty()) {
      if(m_debug) {
        std::cout << "Cfg Path for " << _label << std::endl;
        for(auto cfg : groupPath->Cfgs()) {
          std::cout << cfg.PrettyPrint() << std::endl;
        }
      }

      // Save individual robot cfgs at the end of the path.
      auto lastVID = groupPath->VIDs().back();
      auto rm = _solution->GetGroupRoadmap(group);
      auto last = rm->GetVertex(lastVID);

      for(auto robot : group->GetRobots()) {
        auto cfg = last.GetRobotCfg(robot);
        m_interimCfgMap[robot] = cfg;
      }

      continue;
    }

    // Check decoupled paths.
    for(auto robot : group->GetRobots()) {
      auto path = _solution->GetPath(robot);
      if(path->Empty())
        return false;

      // Save last cfg in path.
      auto lastVID = path->VIDs().back();
      auto rm = _solution->GetRoadmap(robot);
      auto cfg = rm->GetVertex(lastVID);
      m_interimCfgMap[robot] = cfg;
    }
  }

  return true;
}

InteractionStrategyExample::State
InteractionStrategyExample::
InterimState(Interaction* _interaction) {

  State interimState;

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto prob = this->GetMPProblem();
  auto solution = _interaction->GetToPostSolution();

  // Collect post condition robot groups
  std::vector<RobotGroup*> postGroups;
  auto& postConditions = _interaction->GetPostConditions();
  
  for(auto conditionLabel : postConditions) {
    auto condition = as->GetCondition(conditionLabel);

    // Check that this is a formation condition
    auto f = dynamic_cast<FormationCondition*>(condition);
    if(!f)
      continue;

    // Collect robots from assigned roles
    std::vector<Robot*> robots;
    std::string groupLabel = "";

    for(auto role : f->GetRoles()) {
      auto robot = m_roleMap[role];
      robots.push_back(robot);
      groupLabel += (role + ":" + robot->GetLabel() + "--");
    }

    // Intialize group
    RobotGroup* group = prob->AddRobotGroup(robots,groupLabel);
    for(auto robot : group->GetRobots()) {
      solution->AddRobot(robot);
    }
    solution->AddRobotGroup(group);
    
    // Create initial group vertex
    auto grm = solution->GetGroupRoadmap(group);
    auto gcfg = GroupCfg(grm);

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
/*------------------------------------------------------------*/
