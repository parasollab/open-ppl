#include "OCMG.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/Cfg.h"

#include "MPProblem/MPTask.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/Solution/Plan.h"

/*------------------------------- Construction -------------------------------*/
OCMG::
OCMG() {
  this->SetName("OCMG");
}

OCMG::
OCMG(XMLNode& _node) : StateGraph(_node) { 
  this->SetName("OCMG");
  m_roadmapStrategy = _node.Read("roadmapStrategy",true,"",
          "Strategy to use to build individual robot roadmaps.");
}

/*---------------------------- Initialization --------------------------------*/

void
OCMG::
Initialize() {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Initialize");

  auto coordinator = plan->GetCoordinator();

  // Initialize solution object.
  m_solution = std::unique_ptr<MPSolution>(
        new MPSolution(coordinator->GetRobot()));

  // Build roadmaps for each robot
  for(auto& kv : coordinator->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        continue;

      ConstructRobotRoadmap(robot);
    }
  }

  SampleInteractions();

  BuildIndividualObjectModeGraph();
}

/*------------------------------- Accessors ----------------------------------*/

OCMG::GroupRoadmapType*
OCMG::
GetGroupRoadmap(RobotGroup* _group) {
  return nullptr;
}

/*-------------------------------- Helpers -----------------------------------*/

void
OCMG::
ConstructRobotRoadmap(Robot* _robot) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConstructRobotRoadmap");

  // TODO::Track associated motion conditions - for now, we can safely assume
  //       the open and closed constraints without and with objects.

  // Collect all formation conditions that could applu to this robot.
  std::set<FormationCondition*> formationConditions;

  auto as = this->GetTMPLibrary()->GetActionSpace();
  for(auto& interaction : as->GetActions()) {
    auto stages = interaction.second->GetStages();
    for(auto c : interaction.second->GetStageConditions(stages.front())) {
      // Check if this is a formation condition
      auto f = dynamic_cast<FormationCondition*>(as->GetCondition(c));
      if(!f)
        continue;

      // Check if the robot can be used here
      for(auto type : f->GetTypes()) {
        if(type == _robot->GetCapability()) {
          formationConditions.insert(f);
          break;
        }
      }
    }

    for(auto c : interaction.second->GetStageConditions(stages.back())) {
      // Check if this is a formation condition
      auto f = dynamic_cast<FormationCondition*>(as->GetCondition(c));
      if(!f)
        continue;

      // Check if the robot can be used here
      for(auto type : f->GetTypes()) {
        if(type == _robot->GetCapability()) {
          formationConditions.insert(f);
          break;
        }
      }
    }
  }

  // Convert to formations to plan for.
  auto coordinator = plan->GetCoordinator();
  auto problem = this->GetMPProblem();
  std::set<std::pair<RobotGroup*,Formation*>> formations;
  std::map<Formation*,FormationCondition*> formationMap;

  for(auto f : formationConditions) {

    // Check if it's just this robot by itself
    if(f->GetTypes().size() == 1) {
      auto group = problem->AddRobotGroup({_robot},_robot->GetLabel());
      formations.insert(std::make_pair(group,nullptr));
    }

    // Find a robot that matches the formation conditions
    std::vector<Robot*> robots = {_robot};
    std::string label = _robot->GetLabel();

    for(auto type : f->GetTypes()) {
      if(type == _robot->GetCapability())
        continue;

      bool found = false;
      for(auto& kv : coordinator->GetInitialRobotGroups()) {
        auto group = kv.first;
        for(auto robot : group->GetRobots()) {
          if(type != robot->GetCapability())
            continue;

          if(std::find(robots.begin(),robots.end(),robot) != robots.end())
            continue;

          found = true;
          robots.push_back(robot);
          label += ("::" + robot->GetLabel());
          break;          
        }

        if(found)
          break;
      }

      if(!found)
        throw RunTimeException(WHERE) << "Failed to build full group for " 
                                      << f->GetLabel()
                                      << ".";
    }

    // Construct group and formation
    auto group = problem->AddRobotGroup(robots,label);
    State state;
    state[group] = std::make_pair(nullptr,MAX_INT);

    std::unordered_map<std::string,Robot*> roleMap;
    f->AssignRoles(roleMap,state);
    auto formation = f->GenerateFormation(roleMap);
    formationMap[formation] = f;
    formations.insert(std::make_pair(group,formation));
  }

  // Run roadmap strategy on all formations
  auto lib = this->GetMPLibrary();

  for(auto f : formations) {
    auto group = f.first;
    auto formation = f.second;

    // Configure formation in roadmap
    m_solution->AddRobotGroup(group);
    auto rm = m_solution->GetGroupRoadmap(group);
    rm->SetAllFormationsInactive();
    if(formation)
      rm->AddFormation(formation);

    // Configure Group Task
    GroupTask gt(group);
    for(auto robot : group->GetRobots()) {
      MPTask mt(robot);
      
      // TODO:: Add path constraint

      gt.AddTask(mt);
    }

    // Run MPSolution to generate roadmap
    lib->Solve(problem,&gt,m_solution.get(),m_roadmapStrategy, LRand(), 
            this->GetNameAndLabel());

    // Copy to all other duplicate pairs of robot - object
    if(group->Size() == 1)
      continue;

    // Iterate through all passive robots
    // TODO::Assuming each group is of size 2 and that only one is passive.
    auto passive = group->GetRobots()[0]->GetMultiBody()->IsPassive() ?
                   group->GetRobots()[0] :
                   group->GetRobots()[1];

    for(auto& kv : coordinator->GetInitialRobotGroups()) {
      auto group = kv.first;
      for(auto robot : group->GetRobots()) {
        if(!robot->GetMultiBody()->IsPassive() or robot == passive)
          continue;

        CopyRoadmap(rm,robot,formationMap[formation]);
      }
    }
  }

}

void
OCMG::
CopyRoadmap(GroupRoadmapType* _rm, Robot* _passive, FormationCondition* _condition) {

  auto originalGroup = _rm->GetGroup();
  auto problem = this->GetMPProblem();

  // Create new group
  std::vector<Robot*> robots = {_passive};
  std::string label = _passive->GetLabel();

  Robot* swapped = nullptr;

  for(auto robot : originalGroup->GetRobots()) {
    // Assuming only one robot of the passive type in the group
    if(robot->GetCapability() == _passive->GetCapability()) {
      swapped = robot;
      continue;
    }
    
    robots.push_back(robot);  
    label += ("::" + robot->GetLabel());
  }

  auto group = problem->AddRobotGroup(robots,label);
  m_solution->AddRobotGroup(group);
  auto rm = m_solution->GetGroupRoadmap(group);

  // Copy Formation
  State state;
  state[group] = std::make_pair(nullptr,MAX_INT);
  std::unordered_map<std::string,Robot*> roleMap;
  _condition->AssignRoles(roleMap,state);
  auto formation = _condition->GenerateFormation(roleMap);

  rm->SetAllFormationsInactive();
  if(formation)
    rm->AddFormation(formation);

  // Copy Roadmap

  // Copy Vertices
  std::map<size_t,size_t> oldToNew;
  for(auto vit = _rm->begin(); vit != _rm->end(); vit++) {
    auto oldGcfg = vit->property();

    GroupCfg newGcfg(rm);

    for(auto robot : originalGroup->GetRobots()) {
      Cfg cfg = oldGcfg.GetRobotCfg(robot);
      if(robot == swapped) {
        cfg.SetRobot(_passive);
      }

      newGcfg.SetRobotCfg(cfg.GetRobot(),std::move(cfg));
    }

    auto vid = rm->AddVertex(newGcfg);
    oldToNew[vit->descriptor()] = vid;
  }

  // Copy Edges
  for(auto vit = _rm->begin(); vit != _rm->end(); vit++) {
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      auto sourceGcfg = _rm->GetVertex(eit->source());
      auto targetGcfg = _rm->GetVertex(eit->target());

      auto source = oldToNew[eit->source()];
      auto target = oldToNew[eit->target()];

      // Copy over minimum info to query paths later
      auto property = eit->property();

      GroupLocalPlanType glp(rm);
      glp.SetWeight(property.GetWeight());
      glp.SetTimeSteps(property.GetTimeSteps());

      for(auto robot : originalGroup->GetRobots()) {
        auto edge = _rm->GetRoadmap(originalGroup->GetGroupIndex(robot))->GetEdge(
              sourceGcfg.GetVID(robot),targetGcfg.GetVID(robot));
        if(robot == swapped) {
          glp.SetEdge(_passive,std::move(edge));
        }
        else {
          glp.SetEdge(robot,std::move(edge));
        }
      }

      // Save edge
      rm->AddEdge(source,target,glp);
    }
  }
}

void
OCMG::
SampleInteractions() {

}

bool
OCMG::
SampleInteraction(Interaction* _interaction, State _state) {
  return false;
}


void
OCMG::
BuildIndividualObjectModeGraph() {

}

/*----------------------------------------------------------------------------*/
