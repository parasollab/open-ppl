#include "StartTask.h"
#include "../TMPHelperAlgorithms/RelaxedGraphPlan.h"

StartTask::
StartTask(Robot* _robot, const Boundary* _location){
  m_robot = _robot;
  m_location = _location;

  m_startState.m_robotLocations[m_robot] = _location;
  m_startState.m_objectLocations.push_back(_location);

  m_resultState.m_robotLocations[m_robot] = _location;
  m_resultState.m_objectLocations.push_back(_location);
  m_resultState.m_taskOwners.push_back(m_robot);
}

StartTask::
~StartTask() = default;

bool
StartTask::
CheckPreConditions(const FactLayer* _factLayer){
  std::cout << "CHECKING PRE CONDITIONS FOR START TASK FOR " << m_robot->GetLabel() << std::endl;
  //Checks that the location is the starting point of the task
  auto initialLocation = *(_factLayer->m_possibleObjectLocations.begin());
  if(initialLocation != m_location)
    return false;

  //Check that robot didn't already start task
  auto& owners = _factLayer->m_possibleRobotPayloads;
  auto ownerIt = std::find(owners.begin(), owners.end(), m_robot);
  if(ownerIt != owners.end())
    return false;

  //if(!CanBeInLocation(m_robot, m_location))
  //  return false;

  //Checks that robot is at the location
  auto possibleLocationsIt = _factLayer->m_possibleRobotLocations.find(m_robot);
  auto possibleLocations = possibleLocationsIt->second;
  auto it = std::find(possibleLocations.begin(), possibleLocations.end(), m_location);
  if(it == possibleLocations.end())
    return false;

  std::cout << "ADDING START TASK FOR " << m_robot->GetLabel() << std::endl;
  return true;
}

std::vector<Robot*>
StartTask::
GetRobots(){
  return {m_robot};
}

std::string
StartTask::
PrintAction(){
  std::string ret = "STARTTASK " + m_robot->GetLabel() + " at ";
  for(auto d : m_location->GetCenter()){
    ret += std::to_string(d) + " : ";
  }
  return ret;
}
