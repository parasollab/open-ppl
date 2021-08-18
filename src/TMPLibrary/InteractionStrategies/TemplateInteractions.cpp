#include "TemplateInteractions.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
/*--------------------------------------- Construction ---------------------------------*/

TemplateInteractions::
TemplateInteractions() {
  this->SetName("TemplateInteractions");
}

TemplateInteractions::
TemplateInteractions(XMLNode& _node) : InteractionStrategyMethod(_node) {
  this->SetName("TemplateInteractions");

  m_lazy = _node.Read("lazy", false, m_lazy, 
                      "Flag indiciating if templates are constructed lazily.");

  m_refine = _node.Read("refine", false, m_lazy, 
                        "Flag indiciating if plans are refined after pasting.");

  m_templateEnvironment = _node.Read("templateEnv", true, "",
                                     "File name for environment to plan templates in.");
}

TemplateInteractions::
~TemplateInteractions() { }

/*--------------------------------------- Interface ------------------------------------*/

void
TemplateInteractions::
Initialize() {

  // If not lazy, build initial templates for all interactions in the action space.
  if(!m_lazy) {
    auto as = this->GetTMPLibrary()->GetActionSpace();
    for(auto action : as->GetActions()) {
      auto interaction = dynamic_cast<Interaction*>(action.second);

      if(!CreateTemplate(interaction)) {
        throw RunTimeException(WHERE) << "Failed to build initial template for"
                                      << interaction->GetLabel();
      }
    }
  }
}

bool
TemplateInteractions::
operator()(Interaction* _interaction, State& _start) {

  

  // Transformation = Sample origin of transformation

  // TransformSolution(Transformation);
  
  // Connect input _start to solution start

  if(m_refine) {
    ;
    // Call MPStrategy with template generated mp solution
  }
  else {
    ;
    // Connect _start to template path
    // Check if valid
  }

  // return if path is valid
  return false;
}

/*----------------------------------- Helper Functions ---------------------------------*/

bool
TemplateInteractions::
CreateTemplate(Interaction* _interaction) {

  // Sample start state from _interaction->GetPreConditions()
  State start;

  // Plan interaction in empty environment.
  auto is = this->GetInteractionStrategyMethod(_interaction->GetInteractionStrategyLabel());
  if(!is->operator()(_interaction,start))
    return false;

  // Extract MPSolutions to the template
  InteractionTemplate it;
  const auto& stages = _interaction->GetStages();
  it.stages = stages;
  for(const auto& stage : stages) {
    it.toStageSolutions[stage] = _interaction->ExtractToStageSolution(stage);
  }
  //TODO::Convert start to it.start

  m_templateMap[_interaction] = std::move(it);

  return true;
}

void
TemplateInteractions::
TransformSolution(MPSolution* _old, MPSolution* _new) {

}

GroupCfg 
TemplateInteractions::
TransformGroupCfg(GroupCfg& _gcfg) {
  return _gcfg;
}

Cfg
TemplateInteractions::
TransformCfg(Cfg& _cfg) {
  return _cfg;
}

/*--------------------------------------------------------------------------------------*/
