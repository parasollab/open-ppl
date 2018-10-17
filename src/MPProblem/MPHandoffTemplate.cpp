#include "MPHandoffTemplate.h"

/*------------------------------ Construction --------------------------------*/

MPHandoffTemplate::
MPHandoffTemplate(MPProblem* _problem, XMLNode& _node) : m_problem(_problem){
  // Parse the tasks within the Handoff Template
  
  m_label = _node.Read("label", true, "", "Label for the handoff template.");

  m_maxAttempts = _node.Read("maxAttempts", true, 0, 0, MAX_INT,
      "The number of attempts made to place the template in the real environment");
  
  m_interactionWeight = _node.Read("interactionWeight", false, 0, 0, 0,
      "The weight of the connecting edge between interaction robot configurations");

  m_savePaths = _node.Read("savePaths", false, false, 
      "Indicates if the handoff requires explicit paths");

  for(auto& child : _node) {
    if(child.Name() == "Task") {
      m_tasks.emplace_back(new MPTask(m_problem, child));
    }
    if(child.Name() == "Location"){
      const std::string pointString = child.Read("point", false, "", 
          "The center point of the handoff template");
      if(!m_tasks[0])
        throw RunTimeException(WHERE, "Must declare the handoff task components before the locations");

      Cfg point(m_tasks[0]->GetRobot());
#ifdef VIZMO_MAP
      std::istringstream pointStream("0 " + pointString);
#else
      std::istringstream pointStream(pointString);
#endif
      point.Read(pointStream);

      m_handoffLocations.push_back(point);

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

std::vector<Cfg>
MPHandoffTemplate::
GetLocations(){
  return m_handoffLocations;
}

std::vector<Cfg>
MPHandoffTemplate::
GetPositions(){
  std::vector<Cfg> positions;
  for(auto center : m_handoffLocations){
    for(auto cfg : m_handoffCfgs){
      double x = cfg[0];
      double y = cfg[1];
      double theta = center[2];

      double newX = x*cos(theta) - y*sin(theta);
      double newY = y*sin(theta) + y*cos(theta);
      double oldTheta = cfg[2];

      Cfg newCfg = cfg;
      newCfg.SetLinearPosition({newX, newY, oldTheta});
      newCfg += center;
      positions.push_back(newCfg);
    }
  }
  return positions;
}

void
MPHandoffTemplate::
FindLocations(){
  //Assumes that manipulators are all in a problem with a fixed base.
  //Will need to adjust if experiments are changed
  if(m_tasks[0]->GetRobot()->GetLabel() == "coordinator_m"){
    //Place tamplate at the base of the fixed manipulator
    return;
  }
  bool sameCapability = true;
  for(auto& task : m_tasks){
    for(auto& task2 : m_tasks){
      if(task->GetCapability() != task2->GetCapability()){
        sameCapability = false;
        break;
      }
    }
  } 
  if(!sameCapability){
    //disjoint workspaces within current system
    return;
  }
  //conjunct workspaces within current system

  //potentially use Workspace Decomposition path to place

}

bool 
MPHandoffTemplate::
SavedPaths(){
  return m_savePaths;
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
AddPath(std::vector<Cfg> _path, double _cost){
  m_distinctPaths.push_back(_path);
  auto roadmap = m_roadmaps[m_roadmaps.size()-1];
  auto robot = roadmap->GetRobot();
  
  std::vector<double> pathVIDs;
  pathVIDs.push_back(roadmap->GetVID(_path[0]));

  RoadmapGraph<Cfg, DefaultWeight<Cfg>>* g = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(robot);
  *g = *roadmap;
  for(size_t i = 1; i < _path.size(); i++){
    auto cfg = _path[i];
    auto vid = g->GetVID(cfg);
    g->DeleteVertex(vid);
    pathVIDs.push_back(vid);
  }
  m_pathlessRoadmaps.push_back(g);
  
  RoadmapGraph<Cfg, DefaultWeight<Cfg>>* pg = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(robot);
  std::unordered_map<double, double> oldToNew;
  for(size_t i = 0; i < _path.size(); i++){
    auto newVID = pg->AddVertex(_path[i]);
    oldToNew[pathVIDs[i]] = newVID;
  }
  


  m_pathOnlyRoadmaps.push_back(pg);
  m_distinctPathCosts.push_back(_cost);
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
      DefaultWeight<Cfg> weight;
      if(m_savePaths){
        double cost = 0;
        for(auto c : m_distinctPathCosts){
          cost += c;
        }
        weight.SetWeight(cost);
      }
      weight.SetWeight(m_interactionWeight);
      m_connectedRoadmap->AddEdge(startVID, endVID, weight);
    }
  }
}

