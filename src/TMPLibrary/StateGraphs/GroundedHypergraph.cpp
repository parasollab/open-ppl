#include "GroundedHypergraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/Solution/Plan.h"

/*------------------------------ Construction --------------------------------*/

GroundedHypergraph::
GroundedHypergraph() {
  this->SetName("GroundedHypergraph");
}

GroundedHypergraph::
GroundedHypergraph(XMLNode& _node) : StateGraph(_node) {
  this->SetName("GroundedHypergraph");

}

GroundedHypergraph::
~GroundedHypergraph() {}

/*-------------------------------- Interface ---------------------------------*/

void
GroundedHypergraph::
Initialize() {

}

void
GroundedHypergraph::
SetStartSet(const std::set<VID>& _startSet) {

}

bool
GroundedHypergraph::
ConnectTransition(const VID& _tail, const VID& _head, const PathConstraints& _pathConstraints,
                  bool _bidirectional) {

  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();

  auto vertex1 = m_hypergraph->GetVertex(_tail);
  auto vertex2 = m_hypergraph->GetVertex(_head);

  // Create start constraint from vertex
  auto startGcfg = vertex1.property.first->GetVertex(vertex1.property.second);
  std::vector<CSpaceConstraint> startConstraints;

  auto grm = startGcfg.GetGroupRoadmap();
  auto group = grm->GetGroup();

  for(auto robot : group->GetRobots()) {
    CSpaceConstraint startConstraint(robot,startGcfg.GetRobotCfg(robot));
    startConstraints.push_back(startConstraint);
  }

  // Create goal constraint from vertex 2
  auto goalGcfg = vertex2.property.first->GetVertex(vertex2.property.second);
  std::vector<CSpaceConstraint> goalConstraints;

  for(auto robot : group->GetRobots()) {
    CSpaceConstraint goalConstraint(robot,goalGcfg.GetRobotCfg(robot));
    goalConstraints.push_back(goalConstraint);
  }

  // Create group task
  auto groupTask = std::shared_ptr<GroupTask>(new GroupTask(group));

  for(size_t i = 0; i < goalConstraints.size(); i++) {
    // Create individual robot task

    const auto& startConstraint = startConstraints[i];
    const auto& goalConstraint = goalConstraints[i];

    auto robot = startConstraint.GetRobot();
    if(robot != goalConstraint.GetRobot())
      throw RunTimeException(WHERE) << "Mismatching robots.";

    MPTask task(robot);
    task.SetStartConstraint(std::move(startConstraint.Clone()));
    task.AddGoalConstraint(std::move(goalConstraint.Clone()));

    for(const auto& c : _pathConstraints) {
      if(c->GetRobot() == robot)
        task.AddPathConstraint(std::move(c->Clone()));
    }

    groupTask->AddTask(task);
  }

  // Set active formation constraints
  //auto formations = mode->formations;
  //auto grm = m_solution->GetGroupRoadmap(group);
  //grm->SetAllFormationsInactive();
  //for(auto f : formations) {
  //  grm->SetFormationActive(f);
  //}

  // Set robots not virtual
  //for(auto robot : grm->GetGroup()->GetRobots()) {
  //  robot->SetVirtual(false);
  //}

  if(m_debug) {
    auto cfg1 = vertex1.property.first->GetVertex(vertex1.property.second);
    auto cfg2 = vertex2.property.first->GetVertex(vertex2.property.second);
    std::cout << "Querying path for " << cfg1.GetGroupRoadmap()->GetGroup()->GetLabel() << std::endl;
    std::cout << "\tFrom: " << cfg1.PrettyPrint() << std::endl;
    std::cout << "\tTo: " << cfg2.PrettyPrint() << std::endl;
  }

  // Query path for task
  lib->SetPreserveHooks(true);
  lib->Solve(prob,groupTask.get(),m_solution.get(),m_queryStrategy, LRand(), 
      "Query transition path");
  lib->SetPreserveHooks(false);

  grm->SetAllFormationsInactive();

  // Set robots back to virtual
  //for(auto robot : grm->GetGroup()->GetRobots()) {
  //  robot->SetVirtual(true);
  //}

  // Extract cost of path from solution
  auto path = m_solution->GetGroupPath(groupTask->GetRobotGroup());

  if(m_debug and !path->Empty()) {
    std::cout << "Path for transition: " << _tail << " -> " << _head << std::endl;
    for(const auto& cfg : path->Cfgs()) {
      std::cout << "\t" << cfg.PrettyPrint() << std::endl;
    }
    std::cout << std::endl;
  }

  if(path->Empty()) {
    if(m_debug) {
      std::cout << "Failed to find a path for: " 
        << _tail 
        << " -> " 
        << _head
        << std::endl;
    }
    return false;
  }

  Transition transition;
  transition.taskSet.push_back({groupTask});
  transition.cost = path->TimeSteps();
  //transition.taskFormations[groupTask.get()] = formations;

  // Add arc to hypergraph
  AddTransition({_tail},{_head},transition,true);

  if(_bidirectional) {
    AddTransition({_head},{_tail},transition,true);
  }

  return true;
}

void
GroundedHypergraph::
ConnectAllTransitions(const std::vector<VID>& _vertices, const PathConstraints& _pathConstraints,
                      const bool& _bidirectional) {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::ConnectTransitions");
  auto prob = this->GetMPProblem();

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  // For each actuated mode in the mode hypergraph, attempt to connect grounded transition samples
  for(size_t i = 0; i < _vertices.size(); i++) {
    for(size_t j = (_bidirectional) ? i : 0; i < _vertices.size(); j++) {

      auto vid1 = _vertices[i];
      auto vid2 = _vertices[j];

      // Make sure vertices are unique
      if(vid1 == vid2)
        continue;

      // Set robots non-virtual
      auto vertex = GetVertex(vid1);
      auto group = vertex.first->GetGroup();
      for(auto robot : group->GetRobots()) {
        robot->SetVirtual(false);
      }

      ConnectTransition(vid1,vid2,_pathConstraints,_bidirectional);

      // Set robots back to virtual
      for(auto robot : group->GetRobots()) {
        robot->SetVirtual(true);
      }
    }
  }

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    if(this->GetPlan()->GetCoordinator()->GetRobot() == robot.get())
      continue;
    robot->SetVirtual(false);
  }

  delete mt;
  // Strange behavior to avoid issues with roadmaps being improved after costs are saved
  //if(m_queryStrategy != m_queryStrategyStatic) {
  //  auto stash = m_queryStrategy;
  //  m_queryStrategy = m_queryStrategyStatic;
  //  ConnectTransitions();
  //  m_queryStrategy = stash;
  //}
}

bool
GroundedHypergraph::
ContainsSolution() {
  return false;
}

/*---------------------------- Vertex Accessors ------------------------------*/

GroundedHypergraph::VID
GroundedHypergraph::
AddVertex(const Vertex& _vertex) {
  return m_hypergraph->AddVertex(_vertex);
}

GroundedHypergraph::Vertex
GroundedHypergraph::
GetVertex(const VID& _vid) {
  return m_hypergraph->GetVertex(_vid).property;
}

/*--------------------------- Hyperarc Accessors ------------------------------*/

GroundedHypergraph::HID
GroundedHypergraph::
AddTransition(const std::set<VID>& _tail, const std::set<VID>& _head,
          const Transition& _transition, const bool& _override) {
  // TODO::Flip head/tail order in Hypergraph class
  return m_hypergraph->AddHyperarc(_head,_tail,_transition,_override);
}

GroundedHypergraph::GH::Hyperarc
GroundedHypergraph::
GetHyperarc(const HID& _hid) {
  return m_hypergraph->GetHyperarc(_hid);
}

GroundedHypergraph::Transition
GroundedHypergraph::
GetTransition(const HID& _hid) {
  return m_hypergraph->GetHyperarc(_hid).property;
}

GroundedHypergraph::Transition
GroundedHypergraph::
GetTransition(const std::set<VID>& _tail, const std::set<VID>& _head) {
  // TODO::Flip head/tail order in Hypergraph class
  auto hid = m_hypergraph->GetHID(_head,_tail);
  return GetTransition(hid);
}

const std::set<GroundedHypergraph::HID>
GroundedHypergraph::
GetOutgoingHyperarcs(const VID& _vid) {
  return m_hypergraph->GetOutgoingHyperarcs(_vid);
}

/*------------------------- Miscellaneous Accessors --------------------------*/

GroundedHypergraph::GH::GraphType*
GroundedHypergraph::
GetReverseGraph() {
  return m_hypergraph->GetReverseGraph();
}

/*---------------------------- Helper Functions ------------------------------*/


/*----------------------------------------------------------------------------*/

istream&
operator>>(istream& _is, const GroundedHypergraph::Vertex _vertex) {
  return _is;
}

ostream&
operator<<(ostream& _os, const GroundedHypergraph::Vertex _vertex) {
  return _os;
}

istream&
operator>>(istream& _is, const GroundedHypergraph::Transition _t) {
  return _is;
}

ostream&
operator<<(ostream& _os, const GroundedHypergraph::Transition _t) {
  return _os;
}
