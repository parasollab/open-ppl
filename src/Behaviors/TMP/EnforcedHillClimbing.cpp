#include "EnforcedHillClimbing.h"
#include "MPProblem/Constraints/Constraint.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Simulator/BulletModel.h"

EnforcedHillClimbing::
EnforcedHillClimbing(const CapabilityMap& _capabilityMap,
                     std::vector<Robot*> _robots,
                     MPTask* _task,
                     std::vector<std::unique_ptr<InteractionTemplate>>& _handoffs,
                     MPLibrary* _library,
                     bool _manipulator)
                     : m_capabilityMap(_capabilityMap){
  m_robots = _robots;
  m_library = _library;
  m_manipulator = _manipulator;

  //Extract initial and goal locations from MPTask
  auto startConstraint = _task->GetStartConstraint();
  m_initialLocation = startConstraint->GetBoundary();

  auto& goalConstraints = _task->GetGoalConstraints();
  m_goalLocation = goalConstraints[0]->GetBoundary();

  //Extract Handoff Locations from Templates
  for(auto& temp : _handoffs){
    //TODO::Find better way to do this
    auto pos = temp->GetPositions();
    auto first = CfgToBoundary(pos[0]);
    auto second = CfgToBoundary(pos[1]);

    std::pair<const Boundary*, const Boundary*> p(first,second);
    m_handoffLocations.push_back(p);
  }


  m_start.m_objectLocations.push_back(m_initialLocation);
  for(auto robot : m_robots){
    m_start.m_robotLocations[robot] = CfgToBoundary(robot->GetSimulationModel()->GetState());
  }

  m_goal.m_objectLocations.push_back(m_goalLocation);

}

EnforcedHillClimbing::
~EnforcedHillClimbing(){
  for(auto& boundary : m_handoffLocations){
    delete boundary.first;
    delete boundary.second;
  }
}

Boundary*
EnforcedHillClimbing::
CfgToBoundary(Cfg _cfg){
  //Provides error range for the manipulator to reach
  if(m_manipulator){
    auto boundary = new CSpaceBoundingBox(8);
    boundary->ShrinkToPoint(_cfg);
    auto ranges = boundary->GetRanges();
    for(size_t i = 3; i < ranges.size(); i++){
      auto range = ranges[i];
      auto center = range.Center();
      boundary->SetRange(i, center-.05, center+.05);
    }

    std::cout << "Ranges for boundary" << std::endl;
    for(auto r : boundary->GetRanges()){
      std::cout << r << std::endl;
    }

    return boundary;
  }
  //Provides error range to the non-manipulator to reach
  else{
    CSpaceBoundingSphere* boundary = new CSpaceBoundingSphere(_cfg.GetPosition(),
        1.2*_cfg.GetRobot()->GetMultiBody()->GetBoundingSphereRadius());
    return boundary;
  }
}

std::vector<std::shared_ptr<Action>>
EnforcedHillClimbing::
Solve(){
  State current = m_start;

  // Find intial possible actions and heuristic score
  std::vector<std::shared_ptr<Action>> plan;
  RelaxedGraphPlan rgp0(current, m_goal, m_robots, m_handoffLocations, m_initialLocation,
                        m_goalLocation, m_capabilityMap, m_library, m_manipulator);
  auto actions = rgp0.Heuristic();
  auto options = rgp0.RecommendedActions();
  if(m_debug){
    for(auto& action : actions){
      std::cout << action->PrintAction() << std::endl;
    }
  }

  // Score is representative of cost of moves not just one
  // "point" per action
  double score = 0;
  for(auto& action : actions){
    score += action->GetCost();
  }

  // Add reachable states/performable actions to queue
  std::queue<std::pair<State,std::vector<std::shared_ptr<Action>>>> queue;
  for(auto action : options){
    State resultState = current.ApplyAction(action);
    std::pair<State,std::vector<std::shared_ptr<Action>>> newEntry(resultState,{action});
    queue.push(newEntry);
  }

  std::list<std::shared_ptr<Action>> possibleActions;
  size_t count = 0;
  while(queue.size() > 0){
    if(m_debug){
      std::cout << "Inside EHC While loop " << count << std::endl;
    }
    count++;
    auto entry = queue.front();
    queue.pop();
    std::vector<std::shared_ptr<Action>> acts = entry.second;
    State check = entry.first;

    if(m_debug){
      std::cout << "Applied Actions: " << std::endl;
      for(auto& a : acts){
        std::cout << a->PrintAction() << std::endl;
      }
    }

    // Find the heuristic score on the possibloe new state
    RelaxedGraphPlan rgp(check, m_goal, m_robots, m_handoffLocations, m_initialLocation,
                         m_goalLocation, m_capabilityMap, m_library, m_manipulator);
    possibleActions = rgp.Heuristic();

    if(m_debug){
      std::cout << std::endl << std::endl << "POSSIBLE ACTIONS FROM RGP" << std::endl;
      for(auto& action : possibleActions){
        std::cout << action->PrintAction() << std::endl;
      }
      std::cout << std::endl;
    }

    //Check if action produces better state
    double newScore = 0;
    for(auto& action : possibleActions){
      newScore += action->GetCost();
    }
    // Update to new state
    if(newScore < score){
      for(auto act : acts){
        if(m_debug){
          std::cout << "ADDING : " << std::endl << act->PrintAction() << std::endl;
        }
        plan.push_back(act);
      }
      current = check;
      score = newScore;
      queue = std::queue<std::pair<State,std::vector<std::shared_ptr<Action>>>>();
      auto options = rgp.RecommendedActions();
      for(auto action : options){
        State resultState = current.ApplyAction(action);
        std::pair<State,std::vector<std::shared_ptr<Action>>> newEntry(resultState,{action});
        queue.push(newEntry);
      }
      m_visitedStates = {};
    }
    // Add child states of action to the queue
    else {
      auto options = rgp.RecommendedActions();
      for(auto action : options){
        State newState = check;
        std::vector<std::shared_ptr<Action>> newActs;
        newState.ApplyAction(action);
        if(!DuplicateState(newState)){
          newActs.push_back(action);
          std::pair<State, std::vector<std::shared_ptr<Action>>> newEntry(newState,newActs);
          queue.push(newEntry);
        }
      }
    }
    // Reached the goal state
    if(score == 0){
      if(m_debug){
        std::cout << "SCORE AT 0" << std::endl;
      }
      if(plan.back()->GetResultState().m_taskOwners.size() < 1){
        for(auto& a : possibleActions)
          plan.push_back(a);
      }
      return plan;
    }
  }
  if(m_debug){
    std::cout << "RAN OUT OF OPTIONS" << std::endl;
  }
  //TODO::HACK - Find actual bug and handle this case - think this is solved
  if(plan.back()->GetResultState().m_taskOwners.size() < 1){
    for(auto& a : possibleActions)
      plan.push_back(a);
  }
  return plan;
}

bool
EnforcedHillClimbing::
DuplicateState(State _state){
  for(auto exist : m_visitedStates){
    for(auto robot : m_robots){
      if(_state.m_robotLocations[robot] != exist.m_robotLocations[robot]){
        return false;
      }
    }
    //Should only run once for now, bc there's only one object/task right now
    for(size_t i = 0; i < exist.m_objectLocations.size(); i++){
      if(_state.m_objectLocations[i] != exist.m_objectLocations[i]){
        return false;
      }
    }
    for(size_t i = 0; i < exist.m_taskOwners.size(); i++){
      if(_state.m_taskOwners[i] != exist.m_taskOwners[i]){
        return false;
      }
    }
  }
  m_visitedStates.push_back(_state);
  return true;
}
