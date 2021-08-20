#include "Interaction.h"

#include "Behaviors/Agents/Coordinator.h"

//#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/Solution/Plan.h"

/*----------------------- Construction -----------------------*/

Interaction::
Interaction() {
  this->SetName("Interaction");
}

Interaction::
Interaction(XMLNode& _node) : Action(_node) {
  ParseXMLNode(_node);
}

Interaction::
~Interaction() {}

/*------------------------ Interface -------------------------*/

void
Interaction::
Initialize() {
  auto c = this->GetPlan()->GetCoordinator();
  for(auto stage : this->m_stages) {
    m_toStageSolutions[stage] = std::unique_ptr<MPSolution>(new MPSolution(c->GetRobot()));
  }
}

bool
Interaction::
Valid(const State& _state) {

  // Check if pre-conditions are satisfied.
  if(!Action::Valid(_state))
    return false;

  // TODO:: This is being done elsewhere - re-evaluate the decision.
  // Check if there is a valid motion from pre-conditions
  // to post-conditions.
  /*State s = _state;
  auto is = this->GetInteractionStrategyMethod(m_isLabel);
  if(!is->operator()(this,s))
    return false;
  */
  return true;
}

/*------------------------ Accessors -------------------------*/

Interaction::MPSolution*
Interaction::
GetToStageSolution(const std::string& _stage) const {
  return m_toStageSolutions.at(_stage).get();
}
    
std::unique_ptr<Interaction::MPSolution>&& 
Interaction::
ExtractToStageSolution(const std::string& _stage) {
  return std::move(m_toStageSolutions[_stage]);
}

const std::string 
Interaction::
GetInteractionStrategyLabel() const {
  return m_isLabel;
}
    
Interaction::GroupPathType*
Interaction::
GetToStageGroupPath(const std::string& _stage) const {
  return m_toStageGroupPaths.at(_stage);
}

void 
Interaction::
SetToStageGroupPath(const std::string& _stage, GroupPathType* _path) {
  m_toStageGroupPaths[_stage] = _path;
}

std::vector<Path*> 
Interaction::
GetToStagePaths(const std::string& _stage) const {
  return m_toStagePaths.at(_stage);
}
    
void 
Interaction::
SetToStagePaths(const std::string& _stage, std::vector<Path*> _paths) {
  m_toStagePaths[_stage] = _paths;
}
    
/*--------------------- Helper Functions ---------------------*/

void
Interaction::
SetClassName() {
  this->SetName("Interaction");
}

void
Interaction::
ParseXMLNode(XMLNode& _node) {
  //Action::ParseXMLNode(_node);

  m_isLabel = _node.Read("isLabel", true, "", 
                         "Interaction Strategy Label");

}

/*------------------------------------------------------------*/
