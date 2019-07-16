#include "InteractionTemplate.h"
/*------------------------------ Construction --------------------------------*/

InteractionTemplate::
InteractionTemplate(InteractionInformation* _info) {
  m_information = _info;
}

/*------------------------------ Accessors --------------------------------*/

std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*>
InteractionTemplate::
GetRoadmaps() const {
  return m_roadmaps;
}

std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*>
InteractionTemplate::
GetRoadmaps() {
  return m_roadmaps;
}

InteractionInformation*
InteractionTemplate::
GetInformation() {
  return m_information;
}

RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
InteractionTemplate::
GetConnectedRoadmap() const {
  return m_connectedRoadmap;
}

std::vector<std::vector<size_t>>&
InteractionTemplate::
GetDistinctRoadmaps() {
  return m_distinctRoadmaps;
}

std::vector<Cfg>
InteractionTemplate::
GetPositions(){
  return m_handoffCfgs;
}

std::vector<std::vector<Cfg>>
InteractionTemplate::
GetPaths(){
  return m_interactionPaths;
}

std::vector<Cfg>&
InteractionTemplate::
GetTranslatedPositions(){
  if(!m_translatedInteractionPositions.empty()){
    return m_translatedInteractionPositions;
  }
  for(auto center : m_information->GetTemplateLocations()){
    for(auto cfg : m_handoffCfgs){
      double x = cfg[0];
      double y = cfg[1];
      double theta = center[2]*PI;

      double newX = x*cos(theta) - y*sin(theta);
      double newY = y*sin(theta) + y*cos(theta);
      double oldTheta = cfg[2];

      Cfg newCfg = cfg;
      newCfg.SetLinearPosition({newX, newY, oldTheta});
      newCfg += center;
      m_translatedInteractionPositions.push_back(newCfg);
    }
  }
  return m_translatedInteractionPositions;
}


std::vector<std::pair<Cfg,Cfg>> 
InteractionTemplate::
GetTransformedPositionPairs(){
  if(!m_transformedInteractionPositionPairs.empty()){
    return m_transformedInteractionPositionPairs;
  }
  for(auto center : m_information->GetTemplateLocations()){
    std::vector<Cfg> positions;
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
    m_transformedInteractionPositionPairs.push_back(std::pair<Cfg,Cfg>(positions[0],positions[1]));
  }
  return m_transformedInteractionPositionPairs;
}


std::vector<std::vector<Cfg>>&
InteractionTemplate::
GetTranslatedPaths(){
  if(!m_translatedInteractionPaths.empty()){
    return m_translatedInteractionPaths;
  }
  for(auto center : m_information->GetTemplateLocations()){
    for(auto& path : m_interactionPaths){
      std::vector<Cfg> translatedPath;
      for(auto cfg : path){
        double x = cfg[0];
        double y = cfg[1];
        double theta = center[2]*PI;

        double newX = x*cos(theta) - y*sin(theta);
        double newY = y*sin(theta) + y*cos(theta);
        double oldTheta = cfg[2];

        Cfg newCfg = cfg;
        newCfg.SetLinearPosition({newX, newY, oldTheta});
        newCfg += center;
        translatedPath.push_back(newCfg);
      }
      m_translatedInteractionPaths.push_back(translatedPath);
    }
  }
  return m_translatedInteractionPaths;
}

/*------------------------------ Member Management --------------------------------*/

void
InteractionTemplate::
AddRoadmap(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _roadmap) {
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
InteractionTemplate::
AddPath(std::vector<Cfg> _path/*, double _cost*/, MPProblem* _problem){
  for(auto& cfg : _path){
    cfg.SetRobot(_problem->GetRobot(cfg.GetRobot()->GetLabel()));
  }
  if(_path.size() == 1){
    for(size_t i = 0; i < 200; i ++){
      _path.push_back(_path[0]);
    }
  }
  if(m_information->SavedPaths()){
    m_interactionPaths.push_back(_path);
  }
  else{
    m_interactionPaths.push_back({_path[_path.size()-1]});
  }
  /*auto roadmap = m_roadmaps[m_roadmaps.size()-1];
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
  m_distinctPathCosts.push_back(_cost);*/
}


void
InteractionTemplate::
AddHandoffCfg(Cfg _cfg, MPProblem* _problem) {
  _cfg.SetRobot(_problem->GetRobot(_cfg.GetRobot()->GetLabel()));
  m_handoffCfgs.push_back(_cfg);
}

void
InteractionTemplate::
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
      /*if(m_information->SavedPaths()){
        double cost = 0;
        for(auto c : m_distinctPathCosts){
          cost += c;
        }
        weight.SetWeight(cost);
      }*/
      weight.SetWeight(m_information->GetInteractionWeight());
      m_connectedRoadmap->AddEdge(startVID, endVID, weight);
      m_connectingEdge = std::pair<size_t,size_t>(startVID, endVID);
    }
  }
}

std::pair<size_t,size_t>
InteractionTemplate::
GetConnectingEdge(){
  return m_connectingEdge;
}
