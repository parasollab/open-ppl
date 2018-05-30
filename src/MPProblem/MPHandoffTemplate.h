#ifndef MP_HANDOFF_TEMPLATE_H_
#define MP_HANDOFF_TEMPLATE_H_

#include <memory>
#include <string>
#include <vector>

#include "MPTask.h"
#include "MPProblem.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"

////////////////////////////////////////////////////////////////////////////////
/// This represents a Handoff Template, which stores the tasks required for
/// robots to perform a handoff.
////////////////////////////////////////////////////////////////////////////////
//template <typename MPTraits>
class MPHandoffTemplate {

  public:

    //typedef typename MPTraits::RoadmapType        Roadmap;

    ///@name Construction
    ///@{

    MPHandoffTemplate(MPProblem* _problem, XMLNode& _node); 

    ///@}
    
    ///@name Accessors
    ///@{
    
    std::vector<std::shared_ptr<MPTask>> GetTasks() const;
  
    std::vector<RoadmapGraph<Cfg,DefaultWeight<Cfg>>> GetRoadmaps() const;

    std::string GetLabel() const;
    
    ///@}    
    ///@name Member Management
    ///@{
    
    void AddRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>> _roadmap);
    
    ///@}

  protected:

    MPProblem* m_problem{nullptr}; ///< The handoff template problem. 

    ///The set of tasks that must be performed to handoff.
    std::vector<std::shared_ptr<MPTask>> m_tasks;

    ///The set of paths created from solving each task.
    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>> m_roadmaps; 

    ///The handoff label
    std::string m_label;
};



/*------------------------------ Construction --------------------------------*/
/*
template <typename MPTraits>
MPHandoffTemplate<MPTraits>::
MPHandoffTemplate(MPProblem* _problem, XMLNode& _node) : m_problem(_problem){
  // Parse the tasks within the Handoff Template
  for(auto& child : _node) {
    if(child.Name() == "Task") {
      m_tasks.emplace_back(new MPTask(m_problem, child));
    }
  }
}

*/
/*------------------------------ Accessors --------------------------------*/
/*
template <typename MPTraits>
std::vector<std::shared_ptr<MPTask>> 
MPHandoffTemplate<MPTraits>::
GetTasks() const {
  return m_tasks;
}

template <typename MPTraits>
std::vector<std::vector<Cfg>> 
MPHandoffTemplate<MPTraits>::
GetRoadmaps() const {
  return m_roadmaps;
}

template <typename MPTraits>
std::string 
MPHandoffTemplate<MPTraits>::
GetLabel() const {
  return m_label;
}
*/
/*------------------------------ Member Management --------------------------------*/
/*
template <typename MPTraits>
void
MPHandoffTemplate::
AddRoadmap(Roadmap _roadmap) {
  m_roadmaps.push_back(_roadmap);
}
*/
#endif
