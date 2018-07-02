#include "MPHandoffTemplate.h"

/*------------------------------ Construction --------------------------------*/

MPHandoffTemplate::
MPHandoffTemplate(MPProblem* _problem, XMLNode& _node) : m_problem(_problem){
  // Parse the tasks within the Handoff Template
  
  m_label = _node.Read("label", true, "", "Label for the handoff template.");

  m_maxAttempts = _node.Read("maxAttempts", true, 0, 0, MAX_INT,
      "The number of attempts made to place the template in the real environment");

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

std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> 
MPHandoffTemplate::
GetRoadmaps() const {
  return m_roadmaps;
}

std::string 
MPHandoffTemplate::
GetLabel() const {
  return m_label;
}

GMSPolyhedron 
MPHandoffTemplate::
GetHandoffPolyhedron() const {
  return m_handoffPolyhedron;
}
    
size_t 
MPHandoffTemplate::
GetMaxAttempts() const {
  return m_maxAttempts;
}

std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*>
MPHandoffTemplate::
GetRoadmaps() {
  return m_roadmaps;
}

RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
MPHandoffTemplate::
GetConnectedRoadmap() const {
  return m_connectedRoadmap;
}

std::vector<std::vector<size_t>>&
MPHandoffTemplate::
GetDistinctRoadmaps() {
  return m_distinctRoadmaps;
}
/*------------------------------ Member Management --------------------------------*/

void
MPHandoffTemplate::
AddRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _roadmap) {
  //auto robot = _roadmap->begin()->property().GetRobot();
  //RoadmapGraph<Cfg, DefaultWeight<Cfg>>* roadmap = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(*_roadmap);
  RoadmapGraph<Cfg, DefaultWeight<Cfg>>* roadmap = 
    new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(_roadmap->GetRobot());
  *roadmap = *_roadmap;
  m_roadmaps.push_back(roadmap);
  /*for(auto vit = _roadmap->begin(); vit != _roadmap->end(); vit++){
    const size_t oldVID = vit->descriptor();
    const size_t newVID = 
  }*/
}


void 
MPHandoffTemplate::
AddHandoffCfg(Cfg _cfg, MPProblem* _problem) {
  _cfg.SetRobot(_problem->GetRobot(_cfg.GetRobot()->GetLabel()));
  m_handoffCfgs.push_back(_cfg);
}
    
void 
MPHandoffTemplate::
SetHandoffPolyhedron(const Boundary* _boundary) {
  m_handoffPolyhedron = _boundary->MakePolyhedron();
}

void
MPHandoffTemplate::
ConnectRoadmaps(Robot* _robot, MPProblem* _problem) {
  //Combine roadmaps into one graph
  m_connectedRoadmap = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(_robot);
  for(auto roadmap : m_roadmaps){
    vector<size_t> currentRoadmap;
    // Copy vertices and map the change in VIDs.
    std::unordered_map<size_t, size_t> oldToNew;
    for(auto vit = roadmap->begin(); vit != roadmap->end(); ++vit) {
      auto oldVID = vit->descriptor();
      //TODO: This cfg does not have a robot. Need to figure out if this matters
      auto newCfg = vit->property();
      newCfg.SetRobot(_problem->GetRobot(newCfg.GetRobot()->GetLabel()));
      auto newVID = m_connectedRoadmap->AddVertex(newCfg);
      oldToNew[oldVID] = newVID;
      currentRoadmap.push_back(newVID);
    }
    // Copy edges.
    for(auto vit = roadmap->begin(); vit != roadmap->end(); ++vit) {
      for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
        auto source = oldToNew[eit->source()];
        auto target = oldToNew[eit->target()];
        if(!m_connectedRoadmap->IsEdge(source, target)){
          m_connectedRoadmap->AddEdge(source, target, eit->property());
        }
      }
    }
    m_distinctRoadmaps.push_back(currentRoadmap);
  }
  //Connect the end configuration and add the edge to the roadmap
  for(auto start : m_handoffCfgs){
    for(auto end : m_handoffCfgs){
      if(start == end)
        continue;
      auto startVID = m_connectedRoadmap->GetVID(start);
      auto endVID = m_connectedRoadmap->GetVID(end);
      const DefaultWeight<Cfg> weight;
      m_connectedRoadmap->AddEdge(startVID, endVID, weight);
    }
  }
}

