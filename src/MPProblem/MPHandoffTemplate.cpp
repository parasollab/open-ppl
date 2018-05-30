#include "MPHandoffTemplate.h"

/*------------------------------ Construction --------------------------------*/

MPHandoffTemplate::
MPHandoffTemplate(MPProblem* _problem, XMLNode& _node) : m_problem(_problem){
  // Parse the tasks within the Handoff Template
  for(auto& child : _node) {
    if(child.Name() == "Task") {
      m_tasks.emplace_back(new MPTask(m_problem, child));
    }
  }
}


/*------------------------------ Accessors --------------------------------*/

std::vector<std::shared_ptr<MPTask>> 
MPHandoffTemplate::
GetTasks() const {
  return m_tasks;
}

std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>> 
MPHandoffTemplate::
GetRoadmaps() const {
  return m_roadmaps;
}

std::string 
MPHandoffTemplate::
GetLabel() const {
  return m_label;
}

/*------------------------------ Member Management --------------------------------*/

void
MPHandoffTemplate::
AddRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>> _roadmap) {
  m_roadmaps.push_back(_roadmap);
}
