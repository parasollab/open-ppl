#include "ITConnector.h"

#include <limits>

ITConnector::
ITConnector(double _threshold, MPLibrary* _library) : m_threshold(_threshold){
  m_library = _library;
}


RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
ITConnector::
ConnectInteractionTemplates(std::vector<std::unique_ptr<InteractionTemplate>>& _ITs,
                            const std::string& _capability,
                            std::vector<Cfg>& _startAndGoal,
                            RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _megaRoadmap){

  RoadmapGraph<Cfg, DefaultWeight<Cfg>>* graph = new RoadmapGraph<
                                      Cfg,DefaultWeight<Cfg>>(_megaRoadmap->GetRobot());

  auto cfgs = CalculateBaseDistances(_ITs,_capability, _startAndGoal);

  if(cfgs.empty())
    return graph;

  graph->SetRobot(cfgs[0]->GetRobot());

  for(auto cfg1 : cfgs){
    for(auto cfg2 : cfgs){
      if(cfg1 == cfg2)
        continue;
      AdjustedInfo info;
      info.m_connection = cfg1;
      info.m_summedDistance = m_baseDistances[cfg1][cfg2];
      info.m_directConnection = false;
      m_adjDist[cfg1][cfg2] = info;
    }
  }

  FindAlternativeConnections(cfgs);
  if(true){
    std::cout << "Number of connections:" << m_connections.size() << std::endl;
  }

  CopyInTemplates(graph, _ITs, _capability, _startAndGoal);
  if(!cfgs[0]->GetRobot()->IsManipulator())
    ConnectTemplates(graph);

  return graph;
}

/******************************Helper Functions*************************/

std::vector<Cfg*>
ITConnector::
CalculateBaseDistances(std::vector<std::unique_ptr<InteractionTemplate>>& _ITs,
                       const std::string& _capability, std::vector<Cfg>& _startAndGoal){
  //TODO::Give dmLabel in an XML node constructor.
  //auto dm = m_library->GetDistanceMetric("connectedFreeSpace");
  auto dm = m_library->GetDistanceMetric("euclidean");
  std::vector<Cfg*> cfgs;
  for(auto& it1 : _ITs){
    for(auto& path1 : it1->GetTranslatedPaths()){
      Cfg& cfg1 = path1[0];
      if(cfg1.GetRobot()->GetCapability() != _capability)
        continue;
      cfgs.push_back(&cfg1);
      for(auto& it2 : _ITs){
        for(auto& path2 : it2->GetTranslatedPaths()){
          Cfg& cfg2 = path2[0];
          if(cfg2.GetRobot()->GetCapability() != _capability or cfg1 == cfg2)
            continue;
          auto distance = dm->Distance(cfg1,cfg2);
          m_baseDistances[&cfg1][&cfg2] = distance;
        }
      }
      for(auto& cfg2 : _startAndGoal){
        if(cfg2.GetRobot()->GetCapability() != _capability or cfg1 == cfg2)
          continue;
        auto distance = dm->Distance(cfg1,cfg2);
        m_baseDistances[&cfg1][&cfg2] = distance;
      }
    }
  }
  for(auto& cfg1 : _startAndGoal){
    if(cfg1.GetRobot()->GetCapability() != _capability)
      continue;
    cfgs.push_back(&cfg1);
    for(auto& it2 : _ITs){
      for(auto& path : it2->GetTranslatedPaths()){
        auto& cfg2 = path[0];
        if(cfg2.GetRobot()->GetCapability() != _capability or cfg1 == cfg2)
          continue;
        auto distance = dm->Distance(cfg1,cfg2);
        m_baseDistances[&cfg1][&cfg2] = distance;
      }
    }
    for(auto& cfg2 : _startAndGoal){
      if(cfg2.GetRobot()->GetCapability() != _capability or cfg1 == cfg2)
        continue;
      auto distance = dm->Distance(cfg1,cfg2);
      m_baseDistances[&cfg1][&cfg2] = distance;
    }
  }
  return cfgs;
}

void
ITConnector::
FindAlternativeConnections(std::vector<Cfg*>& _cfgs){
  bool newConnection = true;
  while(newConnection){
    newConnection = false;
    for(auto cfg1 : _cfgs){
      double minDistance = std::numeric_limits<double>::max();
      auto neighbor = cfg1;
      for(auto cfg2 : _cfgs){
        if(cfg1 == cfg2)
          continue;
        auto info = m_adjDist[cfg1][cfg2];
        if(info.m_connection == cfg1
           and info.m_summedDistance < minDistance
           and info.m_directConnection == false){
          neighbor = cfg2;
          minDistance = info.m_summedDistance;
        }
      }
      if(neighbor != cfg1){
        std::pair<Cfg*,Cfg*> oppositePath(neighbor,cfg1);
        auto iter = std::find(m_connections.begin(), m_connections.end(), oppositePath);
        if(iter == m_connections.end()){
          m_connections.push_back(std::pair<Cfg*,Cfg*>(cfg1,neighbor));
          UpdateAdjustedDistances(cfg1,neighbor,_cfgs);
          newConnection = true;
        }
      }
    }
  }
}

void
ITConnector::
UpdateAdjustedDistances(Cfg* _cfg1, Cfg* _cfg2, std::vector<Cfg*> _cfgs){
  for(auto cfg3 : _cfgs){
    size_t connectingDistance1 = m_adjDist[_cfg2][cfg3].m_summedDistance + m_adjDist[_cfg1][_cfg2].m_summedDistance;
    size_t connectingDistance2 = m_adjDist[_cfg1][cfg3].m_summedDistance + m_adjDist[_cfg2][_cfg1].m_summedDistance;
    if(m_adjDist[_cfg1][cfg3].m_summedDistance > connectingDistance1
        and (m_baseDistances[_cfg1][cfg3] * m_threshold) > connectingDistance1){
      m_adjDist[_cfg1][cfg3].m_directConnection = false;
      m_adjDist[_cfg1][cfg3].m_connection = _cfg2;
      m_adjDist[_cfg1][cfg3].m_summedDistance = connectingDistance1;
    }
    else if(m_adjDist[_cfg2][cfg3].m_summedDistance > connectingDistance2
        and (m_baseDistances[_cfg2][cfg3] * m_threshold) > connectingDistance2){
      m_adjDist[_cfg2][cfg3].m_directConnection = false;
      m_adjDist[_cfg2][cfg3].m_connection = _cfg1;
      m_adjDist[_cfg2][cfg3].m_summedDistance = connectingDistance2;
    }
  }
  m_adjDist[_cfg1][_cfg2].m_directConnection = true;
  m_adjDist[_cfg2][_cfg1].m_directConnection = true;
}

void
ITConnector::
CopyInTemplates(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _graph,
                std::vector<std::unique_ptr<InteractionTemplate>>& _ITs,
                const std::string& _capability,
                std::vector<Cfg>& _startAndGoal){
  for(auto& it : _ITs){
    for(auto roadmap : it->GetRoadmaps()){
      if(roadmap->begin()->property().GetRobot()->GetCapability() != _capability)
        continue;
      for(auto location : it->GetInformation()->GetTemplateLocations()){
        //Copy in vertices
        std::unordered_map<size_t,size_t> oldToNew;
        for(auto vit = roadmap->begin(); vit != roadmap->end(); vit++){
          const size_t oldVID = vit->descriptor();
          auto relativeCfg = vit->property();
          //relativeCfg.SetRobot(_graph->GetRobot());
          TranslateCfg(location, relativeCfg);

          const size_t newVID = _graph->AddVertex(relativeCfg);
          oldToNew[oldVID] = newVID;
        }
        //Copy in edges
        for(auto vit = roadmap->begin(); vit != roadmap->end(); vit++){
          for(auto eit = vit->begin(); eit != vit->end(); eit++){
            const size_t source = oldToNew[eit->source()];
            const size_t target = oldToNew[eit->target()];
            if(!_graph->IsEdge(source,target)){
              auto intermediates = eit->property().GetIntermediates();
              for(auto cfg : intermediates){
                TranslateCfg(location, cfg);
              }
              _graph->AddEdge(source,target,eit->property());
            }
          }
        }
      }
    }
  }
  for(auto cfg : _startAndGoal){
    if(cfg.GetRobot()->GetCapability() == _capability){
      _graph->AddVertex(cfg);
    }
  }
}

void
ITConnector::
TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg){
  double x = _relativeCfg[0];
  double y = _relativeCfg[1];
  double theta = _centerCfg[2];

  double newX = x*cos(theta) - y*sin(theta);
  double newY = y*sin(theta) + y*cos(theta);
  double oldTheta = _relativeCfg[2];

  _relativeCfg.SetLinearPosition({newX,newY,oldTheta});

  _relativeCfg += _centerCfg;
}

void
ITConnector::
ConnectTemplates(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _graph){
  auto robot = _graph->GetRobot();
  auto solution = new MPSolution(robot);
  // TODO:: Do not want to double copy, but we have to right now until MPSolution
  // API is more advanced.
  *solution->GetRoadmap(robot) = *_graph;

  _graph->Write("ConnectingTemplates-"+robot->GetCapability()+".map",
               robot->GetMPProblem()->GetEnvironment());

  for(auto connection : m_connections){
    auto start = *(connection.first);
    auto goal = *(connection.second);

    start.SetRobot(robot);
    goal.SetRobot(robot);
    if(!m_library->GetValidityChecker("terrain_solid")->IsValid(start,"start"))
      continue;
    if(!m_library->GetValidityChecker("terrain_solid")->IsValid(goal,"goal"))
      continue;

    MPTask* task = new MPTask(robot);
    std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(robot, start));
    std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(robot, goal));
    task->SetStartConstraint(std::move(startConstraint));
    task->AddGoalConstraint(std::move(goalConstraint));

    m_library->SetTask(task);

    if(robot->IsManipulator()){
      m_library->Solve(m_library->GetMPProblem(), task, solution, "EvaluateMapStrategy",
                       LRand(), "ConnectingDistinctRoadmaps");
    }
    else {
      m_library->Solve(m_library->GetMPProblem(), task, solution, "BasicPRM1",
                       LRand(), "ConnectingDistinctRoadmaps");
    }
    if(m_debug){
      if(solution->GetPath()->Cfgs().empty()){
        std::cout << "No path between templates." << std::endl;
      }
    }
    for(auto cfg : solution->GetPath()->Cfgs()){
      std::cout << cfg.PrettyPrint() << std::endl;
    }

  }
  *_graph = *solution->GetRoadmap(robot);
  _graph->Write("ConnectedTemplates-"+robot->GetCapability()+".map",
                robot->GetMPProblem()->GetEnvironment());
}
