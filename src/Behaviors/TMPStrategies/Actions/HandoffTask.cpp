#include "HandoffTask.h"
#include "../TMPHelperAlgorithms/RelaxedGraphPlan.h"

HandoffTask::
HandoffTask(Robot* _passer, Robot* _receiver,
    const Boundary* _passingLocation,
    const Boundary* _receivingLocation,
    MPLibrary* _library){

  m_passer            = _passer;
  m_receiver          = _receiver;
  m_passingLocation   = _passingLocation;
  m_receivingLocation =  _receivingLocation;
  m_library = _library;

  m_startState.m_taskOwners.push_back(_passer);
  m_startState.m_objectLocations.push_back(_passingLocation);
  m_startState.m_robotLocations[_passer] = _passingLocation;
  m_startState.m_robotLocations[_receiver] = _receivingLocation;

  m_resultState.m_taskOwners.push_back(_receiver);
  m_resultState.m_objectLocations.push_back(_receivingLocation);
  m_resultState.m_robotLocations[_passer] = _passingLocation;
  m_resultState.m_robotLocations[_receiver] = _receivingLocation;
}

HandoffTask::
~HandoffTask() = default;

bool
HandoffTask::
CheckPreConditions(const FactLayer* _factLayer){

  std::cout << "CHECKING PRE CONDITIONS FOR HANDOFF TASK FOR " << m_passer->GetLabel() << " to " << m_receiver->GetLabel() << std::endl;

  std::cout << "AT: ";
  for(auto d : m_passingLocation->GetCenter()){
    std::cout << d << " : ";
  }
  std::cout << " to ";
  for(auto d : m_receivingLocation->GetCenter()){
    std::cout << d << " : ";
  }
  std::cout << std::endl << std::endl;

  //Check if passer is in the passing location and that the receiver is
  //in the receiving location and that the passer has the task

  auto possibleLocationsIter = _factLayer->m_possibleRobotLocations.find(m_passer);
  auto possibleLocations = possibleLocationsIter->second;
  auto it = std::find(possibleLocations.begin(), possibleLocations.end(), m_passingLocation);
  if(it == possibleLocations.end())
    return false;

  possibleLocationsIter = _factLayer->m_possibleRobotLocations.find(m_receiver);
  possibleLocations = possibleLocationsIter->second;
  it = std::find(possibleLocations.begin(), possibleLocations.end(), m_receivingLocation);
  if(it == possibleLocations.end())
    return false;


  // Check if the object/task can be at the IT location
  auto objectLocations = _factLayer->m_possibleObjectLocations;
  auto obIt = std::find(objectLocations.begin(), objectLocations.end(), m_passingLocation);
  if(obIt == objectLocations.end())
    return false;

  // Check if the object/task can be owned by the passing robot
  auto objectOwners = _factLayer->m_possibleRobotPayloads;
  auto owIt = std::find(objectOwners.begin(), objectOwners.end(), m_passer);
  if(owIt == objectOwners.end())
    return false;

  if(m_debug){
    std::cout << "ADDING HANDOFF TASK ACTION FROM "
              << m_passer->GetLabel()
              << " to "
              << m_receiver->GetLabel()
              << std::endl;
  }
  return true;
}

std::vector<Robot*>
HandoffTask::
GetRobots(){
  return {m_passer, m_receiver};
}


std::string
HandoffTask::
PrintAction(){
  std::string ret = "HANDOFF TASK " + m_passer->GetLabel() + " at ";
  for(auto d : m_passingLocation->GetCenter()){
    ret += std::to_string(d) + " : ";
  }

  ret += " to " + m_receiver->GetLabel();

  ret += " at ";
  for(auto d : m_receivingLocation->GetCenter()){
    ret += std::to_string(d) + " : ";
  }
  return ret;
}
