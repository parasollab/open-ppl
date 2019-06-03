#include "RelaxedGraphPlan.h"

#include "Behaviors/Agents/Agent.h"

#include "TMPLibrary/Actions/MoveRobot.h"
#include "TMPLibrary/Actions/HandoffTask.h"
#include "TMPLibrary/Actions/StartTask.h"


#include <list>
#include <math.h>


RelaxedGraphPlan::
RelaxedGraphPlan(State _start,
                 State _goal,
                 std::vector<Robot*> _robots,
                 std::set<std::pair<const Boundary*, const Boundary*>> _handoffLocations,
                 const Boundary* _initialLocation,
                 const Boundary* _goalLocation,
                 const CapabilityMap& _capabilityMap,
                 MPLibrary* _library,
                 bool _manipulator)
                 : m_capabilityMap(_capabilityMap){

  m_start = _start;
  m_goal  = _goal;
  m_library = _library;
  m_manipulator = _manipulator;
  m_robots = _robots;

  FactLayer* facts = new FactLayer();

  // Add initial robot locations to first fact layer and all location list
  for(size_t i = 0; i < _robots.size(); i++){
    auto robot = _robots[i];
    auto location = _start.m_robotLocations[robot];
    facts->m_possibleRobotLocations[robot] = {location};
    m_allLocations.push_back(location);
  }

  // Add handoff locations to the list of all locations
  for(auto& location : _handoffLocations){
    m_allLocations.push_back(location.first);
    m_allLocations.push_back(location.second);
  }

  // Add object initial locations to list of all locations
  // Needs to be updated for multiple objects
  m_allLocations.push_back(_start.m_objectLocations[0]);
  m_allLocations.push_back(_goalLocation);

  m_finalGoalLayer = new FactLayer();
  m_finalGoalLayer->m_possibleObjectLocations.insert(_goalLocation);

  //Indicates that no robot has intiated the task yet
  if(m_start.m_taskOwners.size() == 0){
    m_allLocations.push_back(_initialLocation);
  }

  facts->m_possibleObjectLocations.insert(*(_start.m_objectLocations.begin()));

  if(_start.m_taskOwners.size() > 0){
    facts->m_possibleRobotPayloads.insert(*(_start.m_taskOwners.begin()));
  }


  ActionLayer* actions = new ActionLayer();

  std::pair<FactLayer*, ActionLayer*> layerPair(facts, actions);

  m_layerGraph.push_back(layerPair);

  m_allFacts = facts;

  // Create all move robot actions
  for(auto robot : _robots){
    for(auto start : m_allLocations){
      for(auto goal : m_allLocations){
        if(start == goal) continue;
        std::string capability = robot->GetAgent()->GetCapability();
        auto roadmapIter = m_capabilityMap.find(capability);
        auto roadmap = roadmapIter->second;
        std::shared_ptr<MoveRobot> move1 = std::shared_ptr<MoveRobot>(
            new MoveRobot(robot, start, goal, false, roadmap, m_library, m_manipulator));
        std::shared_ptr<MoveRobot> move2 = std::shared_ptr<MoveRobot>(
            new MoveRobot(robot, start, goal, true, roadmap, m_library, m_manipulator));

        auto it1 = std::find(m_allActions.begin(), m_allActions.end(), move1);
        if(it1 == m_allActions.end()){
          m_allActions.push_back(move1);
        }

        auto it2 = std::find(m_allActions.begin(), m_allActions.end(), move2);
        if(it2 == m_allActions.end()){
          m_allActions.push_back(move2);
        }
      }
    }
  }

  // Create all start task actions if task has not been started
  if(m_start.m_taskOwners.size() == 0){
    for(auto robot : _robots){
      std::shared_ptr<StartTask> startTask(new StartTask(robot, _initialLocation));
      m_allActions.push_back(startTask);
    }
  }

  // Create all handoff actions
  for(auto passer : _robots){
    for(auto receiver : _robots){
      if(passer == receiver) continue;
      for(auto location : _handoffLocations){
        std::shared_ptr<HandoffTask> handoff(new
            HandoffTask(passer, receiver, location.first, location.second, m_library));
        std::shared_ptr<HandoffTask> handoff2(new
            HandoffTask(receiver, passer, location.second, location.first, m_library));
      m_allActions.push_back(handoff);
      m_allActions.push_back(handoff2);
      }
    }
  }

}

RelaxedGraphPlan::
~RelaxedGraphPlan() = default;

std::list<std::shared_ptr<Action>>
RelaxedGraphPlan::
Heuristic(){
  CreateGraphLayers();

  BackTrace();

  return LinearizePlan();
}


bool
RelaxedGraphPlan::
IsAtGoal(){
  if(m_debug){
    std::cout << "Checking if at goal" << std::endl;
  }
  //TODO:: Extend past basic rudimentary implementation
  //This just checks if a single object is at the location
  //described in the goal state
  auto goalLocation = m_goal.m_objectLocations[0];
  auto it = std::find(m_allFacts->m_possibleObjectLocations.begin(),
                      m_allFacts->m_possibleObjectLocations.end(),
                      goalLocation);
  if(it == m_allFacts->m_possibleObjectLocations.end()){
    //Not at goal
    return false;
  }
  //Is at goal
  return true;
}


/*------------------------Helper Functions----------------------------*/


void RelaxedGraphPlan::
CreateGraphLayers(){
  size_t iter = 0;

  auto currentFactLayer   =  m_layerGraph[iter].first;
  while(!IsAtGoal()){
    currentFactLayer   =  m_layerGraph[iter].first;
    if(m_debug){
      std::cout << "FACT LAYER " << iter << std::endl << std::endl;
      for(auto& robotLocations : currentFactLayer->m_possibleRobotLocations){
        std::cout << robotLocations.first->GetLabel() << std::endl;
        for(auto location : robotLocations.second){
          for(auto d : location->GetCenter()){
            std::cout << d << ":";
          }
          std::cout << endl;
        }
        std::cout << std::endl;
      }
      std::cout << endl;

      std::cout << "Object/Task Locations" << std::endl;
      for(auto& location : currentFactLayer->m_possibleObjectLocations){
        for(auto d : location->GetCenter()){
          std::cout << d << ":";
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;

      std::cout << "Object/Task Owners" << std::endl;
      for(auto& robot : currentFactLayer->m_possibleRobotPayloads){
        std::cout << robot->GetLabel();
        std::cout << endl;
      }
    }

    ActionLayer* actions =  m_layerGraph[iter].second;
    FactLayer* newGoalLayer = new FactLayer();
    m_goalLayers.push_back(newGoalLayer);

    if(m_debug){
      std::cout << "STARTING ITERATION: " << iter << std::endl;
    }

    bool addedAction = false;
    //add all possible actions to this action layer
    for(auto& action : m_allActions){
      if(!action->Used() and action->CheckPreConditions(m_layerGraph[iter].first)){
        action->SetUsed(true);
        auto it = std::find(actions->m_actions.begin(), actions->m_actions.end(), action);
        if(it == actions->m_actions.end()){
          actions->m_actions.push_back(action);
          addedAction = true;
        }
      }
    }

    if(m_debug){
      std::cout << "Done adding actions" << std::endl;
    }

    if(!addedAction){
      std::cout << "DID NOT FIND A SOLUTION" << std::endl;
      m_layerGraph = {};
      return;
    }


    FactLayer* newFacts = m_layerGraph[iter].first;
    for(auto& action : actions->m_actions){
      auto state = action->GetResultState();

      for(size_t i = 0; i < state.m_objectLocations.size(); i++){
        auto objectLocation = state.m_objectLocations[i];
        AddObjectLocation(objectLocation, newFacts);

      }

      //Indicates MoveRobot was selected
      //Should only execute once with current action options
      for(auto& robotLocation : state.m_robotLocations){
        auto robot = robotLocation.first;
        auto location = robotLocation.second;
        AddRobotLocation(robot, location, newFacts);
      }

      //Indicates Handoff Occurred
      //Should only execute once with current action options
      for(size_t i = 0; i < state.m_taskOwners.size(); i++){
        auto taskOwner = state.m_taskOwners[i];
        AddObjectOwner(taskOwner, newFacts);
      }
    }
    //Create next layer in the graph
    ActionLayer* newActions = new ActionLayer();
    newActions->m_actions = actions->m_actions;

    if(iter == 0){
      for(auto& action : actions->m_actions){
        m_firstOptions.insert(action);
      }
    }

    std::pair<FactLayer*, ActionLayer*> newLayer(newFacts, newActions);
    m_layerGraph.push_back(newLayer);
    //TODO:: Add check for various goals along the way.
    //Simple for now bc it only consists of the single goal
    //of reaching the boundary. (Info added to layer in the
    //AddRobotLocation function
    iter++;
  }

  m_goalLayers.push_back(m_finalGoalLayer);
  if(m_debug){
    std::cout << "FACT LAYER " << iter << std::endl << std::endl;
    for(auto& robotLocations : currentFactLayer->m_possibleRobotLocations){
      std::cout << robotLocations.first->GetLabel() << std::endl;
      for(auto location : robotLocations.second){
        for(auto d : location->GetCenter()){
          std::cout << d << ":";
        }
        std::cout << endl;
      }
      std::cout << std::endl;
    }
    std::cout << endl;

    std::cout << "Object/Task Locations" << std::endl;
    for(auto& location : currentFactLayer->m_possibleObjectLocations){
      for(auto d : location->GetCenter()){
        std::cout << d << ":";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Object/Task Owners" << std::endl;
    for(auto& robot : currentFactLayer->m_possibleRobotPayloads){
      std::cout << robot->GetLabel();
      std::cout << endl;
    }
  }

}

void
RelaxedGraphPlan::
AddRobotLocation(Robot* _robot, const Boundary* _location, FactLayer* _newFacts){
  //Check if the location is already possible for the robot to be at
  auto& priorLocations = _newFacts->m_possibleRobotLocations[_robot];
  auto it = std::find(priorLocations.begin(), priorLocations.end(), _location);

  //If its a new location, add it to the set of all possible facts and
  //this fact layer
  if(it == priorLocations.end()){
    priorLocations.insert(_location);
    auto& layerLocations = _newFacts->m_possibleRobotLocations[_robot];
    layerLocations.insert(_location);

    //Check if it was possible that the robot had the object at this point
    //and moved it
    auto& priorOwners = _newFacts->m_possibleRobotPayloads;
    auto it2 = std::find(priorOwners.begin(), priorOwners.end(), _robot);
    if(it2 != priorOwners.end()){
      AddObjectLocation(_location, _newFacts);
      //Checks if moving to this location achieved the goal state
      if(_location == m_goal.m_objectLocations[0]){
        m_goalLayers[m_goalLayers.size()-1]->m_possibleRobotLocations[_robot] = {_location};
        m_goalLayers[m_goalLayers.size()-1]->m_possibleObjectLocations.insert(_location);
        m_goalLayers[m_goalLayers.size()-1]->m_possibleRobotPayloads.insert(_robot);
      }
    }
  }
}

void
RelaxedGraphPlan::
AddObjectLocation(const Boundary* _location, FactLayer* _newFacts){
  //Check if the object location is already possible
  auto& priorObjectLocations = _newFacts->m_possibleObjectLocations;
  auto it = std::find(priorObjectLocations.begin(), priorObjectLocations.end(), _location);

  //If its a new location, add it to the set of all possible facts and
  //this fact layer
  if(it == priorObjectLocations.end()){
    priorObjectLocations.insert(_location);
    auto& layerObjectLocations = _newFacts->m_possibleObjectLocations;
    layerObjectLocations.insert(_location);
  }
}

void
RelaxedGraphPlan::
AddObjectOwner(Robot* _robot, FactLayer* _newFacts){
  //Check if the object ownership is already possible
  auto& priorOwners = _newFacts->m_possibleRobotPayloads;
  auto it = std::find(priorOwners.begin(), priorOwners.end(), _robot);

  //If it a new ownership possibility add it to all possible facts and
  //this fact layer
  if(it == priorOwners.end()){
    priorOwners.insert(_robot);
    auto& layerOwners = _newFacts->m_possibleRobotPayloads;
    layerOwners.insert(_robot);
  }
}

//TODO::Currently implemented with simplistic goal defintion
void RelaxedGraphPlan::
BackTrace(){
  if(m_debug){
    std::cout << "Starting Back Trace" << std::endl;
  }
  m_actionPlan = std::vector<ActionLayer*>(m_layerGraph.size());
  m_trueFacts  = std::vector<FactLayer*>(m_layerGraph.size());
  m_trueFacts[0] = m_layerGraph[0].first;

  //Shuffle Order of actions to produce different plans and not have it be
  //decided by order of initialized actions
  /*for(auto& layer : m_layerGraph){
    std::random_shuffle(layer.second->m_actions.begin(), layer.second->m_actions.end());
  }*/
  for(auto& layer : m_layerGraph){
    layer.second->m_actions = SortActions(layer.second->m_actions);
  }


  for(size_t i = 0; i < m_actionPlan.size(); i++){
    m_actionPlan[i] = new ActionLayer();
  }

  for(auto& robot : m_robots){
    m_goalLayers[m_goalLayers.size()-1]->m_possibleRobotPayloads.insert(robot);
  }
  for(size_t i = m_actionPlan.size()-1; i > 0; i--){
    auto& goalLayer = m_goalLayers[i];

    if(m_debug){
    std::cout << "Inside backtrace for loop: " << i << std::endl;

      //TODO::Turn into a print fact layer function
      std::cout << "GOAL LAYER " << i << std::endl << std::endl;
      for(auto& robotLocations : goalLayer->m_possibleRobotLocations){
        std::cout << robotLocations.first->GetLabel() << std::endl;
        for(auto location : robotLocations.second){
          for(auto d : location->GetCenter()){
            std::cout << d << ":";
          }
          std::cout << endl;
        }
        std::cout << std::endl;
      }
      std::cout << endl;

      std::cout << "Object/Task Locations" << std::endl;
      for(auto& location : goalLayer->m_possibleObjectLocations){
        for(auto d : location->GetCenter()){
          std::cout << d << ":";
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;

      std::cout << "Object/Task Owners" << std::endl;
      for(auto& robot : goalLayer->m_possibleRobotPayloads){
        std::cout << robot->GetLabel();
        std::cout << endl;
      }
    }


    bool found = false;
    for(auto& location : goalLayer->m_possibleObjectLocations){

      for(auto& action : m_layerGraph[i-1].second->m_actions){
        auto result = action->GetResultState();

        //Should execute only once
        for(auto& location2 : result.m_objectLocations){
          if(location != location2) continue;
          //found = true;
          auto it = std::find(m_actionPlan[i-1]->m_actions.begin(),
                  m_actionPlan[i-1]->m_actions.end(), action);
          if(it != m_actionPlan[i-1]->m_actions.end()){
            continue;
          }
          //m_actionPlan[i-1]->m_actions.push_back(action);
          auto state = action->GetStartState();
          //Check for Start Task
          if(state.m_taskOwners.size() < result.m_taskOwners.size()){
            //EDIT
            if(goalLayer->m_possibleRobotPayloads.count(result.m_taskOwners[0])) {
              m_actionPlan[i-1]->m_actions.push_back(action);
              found = true;
            }
            else{
              continue;
            }
            auto startRobot = result.m_taskOwners[0];
            if(m_start.m_robotLocations[startRobot] != location){
              AddRobotLocation(startRobot, location, m_goalLayers[i-1]);
            }
          }
          if(found){
            break;
          }

          //EDIT:: MOVING BELOW OTHER CHECKS
          /*for(size_t j = 0; j < state.m_objectLocations.size(); j++){
            auto objectLocation = state.m_objectLocations[j];
            //TODO::Add check to see if parallel action needs results of
            //this action to be performed.
            //Shouldn't be necessary in simplistic implementation

            AddObjectLocation(objectLocation, m_goalLayers[i-1]);
          }*/

          //Indicates MoveRobot was selected
          //Should only execute once with current action options
          for(auto& robotLocation : state.m_robotLocations){
            auto robot = robotLocation.first;
            if(state.m_robotLocations[robot] == result.m_robotLocations[robot]){
              continue;
            }
            //EDIT BEGIN
            if(result.m_taskOwners.size() > 0 and !goalLayer->m_possibleRobotPayloads.count(robot)){
              continue;
            }

            m_actionPlan[i-1]->m_actions.push_back(action);
            if(!found){//found indicates that a start task action was already added
              AddObjectOwner(robot, m_goalLayers[i-1]);
            }
            found = true;
            //EDIT END

            AddObjectLocation(state.m_objectLocations[0], m_goalLayers[i-1]);

            auto location = state.m_robotLocations[robot];
            //TODO::Add check to see if parallel action needs results of
            //this action to be performed.

            if(m_start.m_robotLocations[robot] != location){
              AddRobotLocation(robot, location, m_goalLayers[i-1]);
            }
          }

          //Indicates Handoff was selected
          //Should only execute once with current action options
          for(size_t j = 0; j < state.m_taskOwners.size(); j++){
            auto taskOwner = state.m_taskOwners[j];

            //TODO::Add check to see if parallel action needs results of
            //this action to be performed.
            //Shouldn't be necessary in simplistic implementation
            //EDIT
            //if(m_start.m_taskOwners.size()==0 or
            //if(m_start.m_taskOwners.size()>0 and
            //    m_start.m_taskOwners[0] != taskOwner){
              AddObjectOwner(taskOwner, m_goalLayers[i-1]);
              if(m_start.m_robotLocations[taskOwner] != state.m_robotLocations[taskOwner]){
                AddRobotLocation(taskOwner, state.m_robotLocations[taskOwner], m_goalLayers[i-1]);
              }
              auto newOwner = result.m_taskOwners[0];
              if(m_start.m_robotLocations[newOwner] != result.m_robotLocations[newOwner]){
                AddRobotLocation(newOwner,result.m_robotLocations[newOwner],m_goalLayers[i-1]);
              }
              m_actionPlan[i-1]->m_actions.push_back(action);
              found = true;
            //}
          }
          if(found){
            for(size_t j = 0; j < state.m_objectLocations.size(); j++){
              auto objectLocation = state.m_objectLocations[j];
              //TODO::Add check to see if parallel action needs results of
              //this action to be performed.
              //Shouldn't be necessary in simplistic implementation

              AddObjectLocation(objectLocation, m_goalLayers[i-1]);
            }
            break;
          }
        }
        if(found){
          break;
        }
      }
    }

    for(auto& robotLocations : goalLayer->m_possibleRobotLocations){
      auto robot = robotLocations.first;
      auto location = *(robotLocations.second.begin());

      if(goalLayer->m_possibleRobotPayloads.size() and
          *(goalLayer->m_possibleRobotPayloads.begin()) == robot) continue;

      if(!(m_actionPlan[i-1]->m_actions.empty()) and
          m_actionPlan[i-1]->m_actions[0]->GetRobots()[0] == robot) continue;


      found = false;
      for(auto& action : m_layerGraph[i-1].second->m_actions){
        if(found) continue;
        //Check if the robots match
        if(action->GetRobots()[0] != robot)
          continue;
        //Check if it moves it to the right location
        auto resultState = action->GetResultState();
        if(location != resultState.m_robotLocations[robot])
          continue;
        //Found action
        auto state = action->GetStartState();
        auto startLocation = state.m_robotLocations[robot];
        m_actionPlan[i-1]->m_actions.push_back(action);

        if(m_start.m_robotLocations[robot] != startLocation){
          AddRobotLocation(robot, startLocation, m_goalLayers[i-1]);
        }
        found = true;
      }
    }

    for(auto action : m_actionPlan[i-1]->m_actions){
      std::cout << action->PrintAction() << std::endl;
    }

  }
  if(m_debug){
    std::cout << "Finished Back Trace" << std::endl;
  }
}

std::list<std::shared_ptr<Action>>
RelaxedGraphPlan::
LinearizePlan(){
  if(m_debug){
    std::cout << "Starting Linearization of the plan" << std::endl;
  }
  //Iterate through each action layer in the action plan and see which
  //ones contain more than one action
  //Check the start conditions of these actions and see if they are true in the
  //layer below this one as this indicates it was produced by an action in the
  //prior layer or one prior to that.
  //May need to update facts all the way up when adding to the true fact layers
  //to make sure this works.
  //Otherwise, will need another way to compare the start and end states of the
  //parallel actions and ee if there is necessary order to them. Should produce
  //a linear set of actions that solves the task.

  //Print out the actions before linearization
  size_t l = 0;
  if(m_debug){
    for(auto& layer : m_actionPlan){
      std:: cout << "Layer: " << l << std::endl;
      for(auto action : layer->m_actions){
        std::cout <<action->PrintAction() << std::endl;
      }
      l++;
    }
    std::cout << std::endl;
    l = 0;

  }
  std::list<std::shared_ptr<Action>> linearActions;
  for(auto& layer : m_actionPlan){
    if(m_debug){
      std::cout << "Layer: " << l << "size: " << layer->m_actions.size() << std::endl;
    }
    if(layer->m_actions.size() == 1){
      linearActions.push_back(layer->m_actions[0]);
      continue;
    }
    std::unordered_map<std::shared_ptr<Action>, std::vector<std::shared_ptr<Action>>> dependencyMap;
    for(auto action1 : layer->m_actions){
      for(auto action2 : layer->m_actions){
        if(action1 == action2) continue;
        if(m_debug){
          std::cout << "Checking dependency of " << action1->PrintAction() << std::endl
                  << " on " << action2->PrintAction() << std::endl;
        }
        auto start1 = action1->GetStartState();
        auto result1 = action1->GetResultState();
        auto start2 = action2->GetStartState();
        auto result2 = action2->GetResultState();
        //TODO::Check if actions that prevent each other need to be checked for

        //Check for object locations
        for(auto location1 : start1.m_objectLocations){
          for(auto location2 : result2.m_objectLocations){
            for(auto location3 : start2.m_objectLocations){
              //Checks if action2 moves object location to location needed for
              //action1
              if(m_debug){
                std::cout << "Checking location equivalencies" << std::endl;
              }
              if(location1 == location2 and location1 != location3){
                //Check for uniqueness
                auto& actions = dependencyMap[action1];
                auto it = std::find(actions.begin(), actions.end(), action2);
                if(it == actions.end()){
                  dependencyMap[action1].push_back(action2);
                }
              }
            }
          }
        }
        if(m_debug){
          std::cout << "Checking for robot location dependency" << std::endl;
        }
        //Check if robot locations of action1 are dependent upon other actions
        for(auto robotLocation1 : start1.m_robotLocations){
          auto robot1 = robotLocation1.first;
          auto location1 = robotLocation1.second;
          for(auto robotLocation2 : result2.m_robotLocations){
            auto robot2 = robotLocation2.first;
            auto location2 = robotLocation2.second;
            if(robot1 == robot2){
              if(location1 == location2){
                for(auto robotLocation3 : start2.m_robotLocations){
                  auto robot3 = robotLocation3.first;
                  auto location3 = robotLocation3.second;
                  if(robot1 == robot3){
                    //Indicates that action2 moves robot to necessary location
                    //for action1
                    if(location2 != location3){
                      //Check for uniqueness
                      auto& actions = dependencyMap[action1];
                      auto it = std::find(actions.begin(), actions.end(), action2);
                      if(it == actions.end()){
                        dependencyMap[action1].push_back(action2);
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    //Turn dependency map into list of actions
    if(m_debug){
      std::cout << "Converting map into list" << std::endl;
    }
    std::list<std::shared_ptr<Action>> layerOrder;
    for(auto& action : layer->m_actions){
      auto dependencySet = dependencyMap[action];
      if(dependencySet.size() == 0){
        layerOrder.push_front(action);
      }
      for(auto it = layerOrder.rbegin(); it != layerOrder.rend(); it++){
        auto dependency = *it;
        auto f = std::find(dependencySet.begin(), dependencySet.end(), dependency);
        if(f == dependencySet.end()){
          continue;
        }
        it--;
        layerOrder.insert(it.base(),action);
        break;
      }
    }
    for(auto it = layerOrder.begin(); it != layerOrder.end(); it++){
      linearActions.push_back(*it);
    }
  }
  if(m_debug){
    std::cout << "Finished Linearization of the plan" << std::endl;
  }
  m_linearActions = linearActions;
  return linearActions;
}

std::vector<std::shared_ptr<Action>>
RelaxedGraphPlan::
RecommendedActions(){
  std::unordered_set<std::shared_ptr<Action>> linearSet;
  //if(m_debug){
  if(true){
    std::cout << "LINEAR PLAN" << std::endl;
    for(auto& action : m_linearActions){
      linearSet.insert(action);
      std::cout << action->PrintAction() << std::endl;
    }
  }


  if(m_debug){
    std::cout << "FIRST OPTIONS" << std::endl;
    for(auto& action : m_firstOptions){
      std::cout << action->PrintAction() << std::endl;
    }
  }

  if(m_debug){
    std::cout << "RecommendedActions" << std::endl;
  }
  std::vector<std::shared_ptr<Action>> ret;
  //for(auto action : m_layerGraph[0].second->m_actions){
  /*for(auto& action : m_linearActions){
    //auto it = std::find(m_linearActions.begin(), m_linearActions.end(), action);
    //if(it != m_linearActions.end()){
    if(m_firstOptions.find(action.get()) != m_firstOptions.end()){
      if(m_debug){
        std::cout << action->PrintAction() << std::endl;
      }
      ret.push_back(action);
    }
  }*/
  for(auto& action : m_firstOptions){
    if(linearSet.count(action)){
      ret.push_back(action);
      std::cout << action->PrintAction() << std::endl;
    }
  }
  //if(m_linearActions.size() > 0)
  //  ret.push_back(m_linearActions.front());
  //srand(m_library->m_solvers.front().seed);
  //std::random_shuffle(ret.begin(), ret.end());
  ret = SortActions(ret);
  return ret;
}

std::vector<std::shared_ptr<Action>>
RelaxedGraphPlan::
SortActions(std::vector<std::shared_ptr<Action>> _actions){
  if(_actions.size() <= 1)
    return _actions;
  size_t pivot = LRand()%_actions.size();
  auto pivotAction = _actions[pivot];
  std::vector<std::shared_ptr<Action>> lowerActions;
  std::vector<std::shared_ptr<Action>> upperActions;
  for(auto& action : _actions){
    if(action == pivotAction)
      continue;
    if(action->GetCost() > pivotAction->GetCost()){
      upperActions.push_back(action);
    }
    else if(action->GetCost() < pivotAction->GetCost()){
      lowerActions.push_back(action);
    }
    else{//tie breaker
      double distance1 = 0;
      for(auto robotLocation : action->GetStartState().m_robotLocations){
        auto initialLocation = m_start.m_robotLocations[robotLocation.first]->GetCenter();
        auto startLocation = robotLocation.second->GetCenter();
        distance1 = sqrt(pow((initialLocation[0]-startLocation[0]),2.0) +
                         pow((initialLocation[1]-startLocation[1]),2.0));
      }
      double distance2 = 0;
      for(auto robotLocation : pivotAction->GetStartState().m_robotLocations){
        auto initialLocation = m_start.m_robotLocations[robotLocation.first]->GetCenter();
        auto startLocation = robotLocation.second->GetCenter();
        distance2 = sqrt(((initialLocation[0]-startLocation[0])*(initialLocation[0]-startLocation[0])) +
                         ((initialLocation[1]-startLocation[1])*(initialLocation[1]-startLocation[1])));
      }
      std::cout << "Distance1: " << distance1 << std::endl;
      std::cout << "Distance2: " << distance2 << std::endl;
      if(distance1>distance2){
        upperActions.push_back(action);
      }
      else{
        lowerActions.push_back(action);
      }
    }
  }

  auto sorted = SortActions(lowerActions);
  sorted.push_back(pivotAction);
  for(auto& action : SortActions(upperActions)){
    sorted.push_back(action);
  }

  return sorted;
}
