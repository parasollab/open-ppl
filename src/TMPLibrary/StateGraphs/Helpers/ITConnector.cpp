#include "ITConnector.h"

#include "ConfigurationSpace/Cfg.h"

#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/TMPTools/InteractionTemplate.h"

#include "Utilities/SSSP.h"

#include "Workspace/WorkspaceSkeleton.h"

#include <limits>
#include <unordered_set>

ITConnector::
ITConnector(double _threshold, MPLibrary* _library) : m_threshold(_threshold){
  m_library = _library;
  //BuildSkeletons();
}

ITConnector::
ITConnector(XMLNode& _node) {
	//TODO::ParseXML
	//BuildSkeletons();
}


RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
ITConnector::
ConnectInteractionTemplates(std::vector<std::shared_ptr<InteractionTemplate>>& _ITs,
                            const std::string& _capability,
                            std::vector<Cfg>& _startAndGoal,
                            RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _megaRoadmap){

  RoadmapGraph<Cfg, DefaultWeight<Cfg>>* graph = new RoadmapGraph<
                                      Cfg,DefaultWeight<Cfg>>(_megaRoadmap->GetRobot());

  auto cfgs = CalculateBaseDistances(_ITs,_capability, _startAndGoal);

	auto robot = cfgs[0]->GetRobot();

	if(robot->GetCapability() != _capability)
		throw RunTimeException(WHERE,"Mismatched robot types.");

  //graph->SetRobot(cfgs[0]->GetRobot());
  graph->SetRobot(robot);

  auto vcm = m_library->GetValidityChecker("terrain_solid");
	std::vector<size_t> invalids;
	for(auto vit = graph->begin(); vit != graph->end(); vit++) {
		Cfg cfg = vit->property();
		cfg.SetRobot(robot);
		bool isValid = vcm->IsValid(cfg, "ValidateITCfg");
		if(!isValid) {
			invalids.push_back(vit->descriptor());
		}
	}
	for(auto vid : invalids) {
		graph->DeleteVertex(vid);
	}

  if(cfgs.empty())
    return graph;

  for(auto cfg1 : cfgs){
    for(auto cfg2 : cfgs){
      if(cfg1 == cfg2)
        continue;
      AdjustedInfo info;
      info.m_connection = cfg1;
      info.m_summedDistance = m_baseDistances[cfg1][cfg2];
      info.m_directConnection = false;
      info.m_changed = false;
      m_adjDist[cfg1][cfg2] = info;
    }
  }

  FindAlternativeConnections(cfgs);
  if(true){
    std::cout << "Number of connections:" << m_connections.size() << std::endl;
    for(auto connection : m_connections){
      std::cout << connection.first->PrettyPrint()
                << "->"
                << connection.second->PrettyPrint()
                << std::endl;
      std::cout << "Base: " << m_baseDistances[connection.first][connection.second] << std::endl;
      std::cout << "Adj: " << m_adjDist[connection.first][connection.second].m_summedDistance
                           << std::endl;

    }
  }

  CopyInTemplates(graph, _ITs, _capability, _startAndGoal);
  if(!cfgs[0]->GetRobot()->IsManipulator())
    ConnectTemplates(graph);

	DirectionConnections(graph, _capability, cfgs);

  m_connections = {};
  m_baseDistances = {};
  m_adjDist = {};
  return graph;
}

/******************************Helper Functions*************************/

std::vector<Cfg*>
ITConnector::
CalculateBaseDistances(std::vector<std::shared_ptr<InteractionTemplate>>& _ITs,
                       const std::string& _capability, std::vector<Cfg>& _startAndGoal){
  //TODO::Give dmLabel in an XML node constructor.
  //auto dm = m_library->GetDistanceMetric("connectedFreeSpace");
  auto dm = m_library->GetDistanceMetric("positionEuclidean");
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
          if(InConnectedWorkspace(cfg1,cfg2)){
            auto distance = dm->Distance(cfg1,cfg2);
            m_baseDistances[&cfg1][&cfg2] = distance;
          }
          else {
            //auto distance = std::numeric_limits<double>::max();
            double distance = -1;
            m_baseDistances[&cfg1][&cfg2] = distance;
          }
        }
      }
      for(auto& cfg2 : _startAndGoal){
        if(cfg2.GetRobot()->GetCapability() != _capability or cfg1 == cfg2)
          continue;
        if(InConnectedWorkspace(cfg1,cfg2)){
          auto distance = dm->Distance(cfg1,cfg2);
          m_baseDistances[&cfg1][&cfg2] = distance;
        }
        else {
          //auto distance = std::numeric_limits<double>::max();
          double distance = -1;
          m_baseDistances[&cfg1][&cfg2] = distance;
        }
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
        if(InConnectedWorkspace(cfg1,cfg2)){
          auto distance = dm->Distance(cfg1,cfg2);
          m_baseDistances[&cfg1][&cfg2] = distance;
        }
        else {
          //auto distance = std::numeric_limits<double>::max();
          double distance = -1;
          m_baseDistances[&cfg1][&cfg2] = distance;
        }
      }
    }
    for(auto& cfg2 : _startAndGoal){
      if(cfg2.GetRobot()->GetCapability() != _capability or cfg1 == cfg2)
        continue;
      if(InConnectedWorkspace(cfg1,cfg2)){
        auto distance = dm->Distance(cfg1,cfg2);
        m_baseDistances[&cfg1][&cfg2] = distance;
      }
      else {
        //auto distance = std::numeric_limits<double>::max();
        double distance = -1;
        m_baseDistances[&cfg1][&cfg2] = distance;
      }
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
        if(m_baseDistances[cfg1][cfg2] < 0){
          continue;
        }
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
    if(m_baseDistances[_cfg1][cfg3] < 0 or m_baseDistances[_cfg2][cfg3] < 0){
      continue;
    }
    //1->2->3
    double connectingDistance1 = m_adjDist[_cfg2][cfg3].m_summedDistance
                               + m_adjDist[_cfg1][_cfg2].m_summedDistance;
    //2->1->3
    double connectingDistance2 = m_adjDist[_cfg1][cfg3].m_summedDistance
                               + m_adjDist[_cfg2][_cfg1].m_summedDistance;

    auto& info1 = m_adjDist[_cfg1][cfg3];
    auto& info2 = m_adjDist[_cfg2][cfg3];
    double summedDistance1, summedDistance2;
    if(info1.m_changed){
      summedDistance1 = info1.m_summedDistance;
    }
    else{
      summedDistance1 = m_threshold * info1.m_summedDistance;
    }
    if(info2.m_changed){
      summedDistance2 = info2.m_summedDistance;
    }
    else{
      summedDistance2 = m_threshold * info2.m_summedDistance;
    }

    if(summedDistance1 > connectingDistance1
        and (m_baseDistances[_cfg1][cfg3] * m_threshold) > connectingDistance1){
      info1.m_directConnection = false;
      info1.m_connection = _cfg2;
      info1.m_summedDistance = connectingDistance1;
      info1.m_changed = true;
    }
    else if(summedDistance2 > connectingDistance2
        and (m_baseDistances[_cfg2][cfg3] * m_threshold) > connectingDistance2){
      info2.m_directConnection = false;
      info2.m_connection = _cfg1;
      info2.m_summedDistance = connectingDistance2;
      info2.m_changed = true;
    }
  }
  m_adjDist[_cfg1][_cfg2].m_directConnection = true;
  m_adjDist[_cfg2][_cfg1].m_changed = true;
  m_adjDist[_cfg2][_cfg1].m_directConnection = true;
  m_adjDist[_cfg1][_cfg2].m_changed = true;
}

void
ITConnector::
CopyInTemplates(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _graph,
                std::vector<std::shared_ptr<InteractionTemplate>>& _ITs,
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
          relativeCfg.TransformCfg(location.GetBaseTransformation());

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
                cfg.TransformCfg(location.GetBaseTransformation());
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
ConnectTemplates(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _graph){
  auto robot = _graph->GetRobot();
  auto solution = new MPSolution(robot);
  auto env = robot->GetMPProblem()->GetEnvironment();
  env->SaveBoundary();
  // TODO:: Do not want to double copy, but we have to right now until MPSolution
  // API is more advanced.
  *solution->GetRoadmap(robot) = *_graph;

  _graph->Write("ConnectingTemplates-"+robot->GetCapability()+".map",
               robot->GetMPProblem()->GetEnvironment());


  for(auto connection : m_connections){
    auto start = *(connection.first);
    auto goal = *(connection.second);

/*    std::unique_ptr<Boundary> terrainBoundary;
    for(auto& terrain : env->GetTerrains()){
      bool match = false;
      for(auto elem : terrain.second){
        if(elem.GetBoundary()->InBoundary(start) and elem.GetBoundary()->InBoundary(goal)){
          env->SetBoundary(std::move(elem.GetBoundary()->Clone()));
          match = true;
          break;
        }
      }
      if(match)
        break;
    }
    */

    //env->IsolateTerrain(start,goal);
		if(!env->SameTerrain(start,goal))	
			continue;

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

    typedef RoadmapGraph<Cfg,DefaultWeight<Cfg>> RoadmapType;
    _graph->InstallHook(RoadmapType::HookType::AddEdge, "debug",
        [](RoadmapType::EI _ei) {
        if(_ei->property().GetWeight()==0){
          std::cout << "Zero weight edge" << std::endl;
        }
        });

    if(robot->IsManipulator()){
      m_library->Solve(m_library->GetMPProblem(), task, solution, "EvaluateMapStrategy",
                       LRand(), "ConnectingDistinctRoadmaps");
    }
    else {
      m_library->Solve(m_library->GetMPProblem(), task, solution, "BasicPRM1",
                       LRand(), "ConnectingDistinctRoadmaps");
    }

    _graph->RemoveHook(RoadmapType::HookType::AddEdge, "debug");

    if(solution->GetPath()->Cfgs().empty()){
      //throw RunTimeException(WHERE, "Could not connect " + start.PrettyPrint() + " to "
      //                       + goal.PrettyPrint());
      std::cout << "COULDN'T CONNECT LOCATIONS." << std::endl;
    }
    for(auto cfg : solution->GetPath()->Cfgs()){
      std::cout << cfg.PrettyPrint() << std::endl;
    }

  }
  //env->RestoreBoundary();
  *_graph = *solution->GetRoadmap(robot);
  _graph->Write("ConnectedTemplates-"+robot->GetCapability()+".map",
                robot->GetMPProblem()->GetEnvironment());

}


void
ITConnector::
BuildSkeletons(){
  auto env = m_library->GetMPProblem()->GetEnvironment();

  for(auto& terrainType : env->GetTerrains()){
    std::vector<GMSPolyhedron> polyhedra;
    for(size_t i = 0; i < env->NumObstacles(); ++i) {
      MultiBody* const obstacle = env->GetObstacle(i);
      for(size_t j = 0; j < obstacle->GetNumBodies(); ++j)
        polyhedra.emplace_back(obstacle->GetBody(j)->GetWorldPolyhedron());
    }

    for(auto& otherTerrainType : env->GetTerrains()){
      if(otherTerrainType.first == terrainType.first){
        continue;
      }
      auto terrains = otherTerrainType.second;
      for(auto terrain : terrains){
				if(terrain.IsVirtual()){
					continue;
				}
        auto boundary = terrain.GetBoundary();
        auto polyhedron = boundary->MakePolyhedron();
        polyhedron.Invert();
        polyhedra.push_back(polyhedron);
      }
    }
    //MedialAxis2D ma(polyhedra, nullptr);

    auto fakeEnv = env->GetBoundary()->Clone();
    double minx, miny, minz, maxx, maxy, maxz;
    minx = miny = minz = std::numeric_limits<double>::max();
    maxx = maxy = maxz = std::numeric_limits<double>::lowest();
    for(auto terrain : terrainType.second){
      minx = std::min(minx, terrain.GetBoundary()->GetRange(0).min);
      maxx = std::max(maxx, terrain.GetBoundary()->GetRange(0).max);
      miny = std::min(miny, terrain.GetBoundary()->GetRange(1).min);
      maxy = std::max(maxy, terrain.GetBoundary()->GetRange(1).max);
      minz = std::min(minz, terrain.GetBoundary()->GetRange(2).min);
      maxz = std::max(maxz, terrain.GetBoundary()->GetRange(2).max);
    }
    for(size_t i = 0; i < env->NumObstacles(); i++){
      auto obstacle = env->GetObstacle(i);
      const double* tmp = obstacle->GetBoundingBox();
      minx = std::min(minx, tmp[0]);
      maxx = std::max(maxx, tmp[1]);
      miny = std::min(miny, tmp[2]);
      maxy = std::max(maxy, tmp[3]);
      minz = std::min(minz, tmp[4]);
      maxz = std::max(maxz, tmp[5]);
    }

    std::vector<std::pair<double,double>> obstBBX(3);
    obstBBX[0] = std::make_pair(minx, maxx);
    obstBBX[1] = std::make_pair(miny, maxy);
    obstBBX[2] = std::make_pair(minz, maxz);
    fakeEnv->ResetBoundary(obstBBX, .01);

    MedialAxis2D ma(polyhedra, fakeEnv.get());
    ma.BuildMedialAxis();
    m_capabilitySkeletons[terrainType.first] = std::shared_ptr<WorkspaceSkeleton>(
                        new WorkspaceSkeleton(get<0>(ma.GetSkeleton(1)))); // 1 for free space
    m_capabilitySkeletons[terrainType.first]->RefineEdges(0.5);//TODO set as xml paramater
    auto& g = m_capabilitySkeletons[terrainType.first]->GetGraph();
    std::cout << "Printing graph for: " << terrainType.first << std::endl;
    for(auto vi = g.begin(); vi != g.end(); vi++){
      std::cout << vi->property() << std::endl;
    }
    for(auto vi = g.begin(); vi != g.end(); vi++){
      for(auto ei = vi->begin(); ei != vi->end(); ei++){
        std::cout << g.find_vertex(ei->source())->property()
                  << ":"
                  << g.find_vertex(ei->target())->property()
                  << std::endl;
      }
    }
    std::vector<size_t> deletionVertices;
    for(auto vi = g.begin(); vi != g.end(); vi++){
      if(env->GetBoundary()->InBoundary(vi->property())){
          continue;
      }
      deletionVertices.push_back(vi->descriptor());
    }
    for(auto vd : deletionVertices){
      g.delete_vertex(vd);
    }
    std::vector<WorkspaceSkeleton::ED> deletionEdges;
    for(auto vi = g.begin(); vi != g.end(); vi++){
      for(auto ei = vi->begin(); ei != vi->end(); ei++){
        //if(!g.find_edge(ei->descriptor, ei->source(), ei->target())){
        //  continue;
        //}
        for(auto point : ei->property()){
          if(env->GetBoundary()->InBoundary(point)){
            continue;
          }
          deletionEdges.push_back(ei->descriptor());
          break;
        }
      }
    }
    for(auto ed : deletionEdges){
      g.delete_edge(ed);
    }
    std::cout << "Printing graph for: " << terrainType.first << std::endl;
    for(auto vi = g.begin(); vi != g.end(); vi++){
      std::cout << vi->property() << std::endl;
    }
    for(auto vi = g.begin(); vi != g.end(); vi++){
      for(auto ei = vi->begin(); ei != vi->end(); ei++){
        std::cout << g.find_vertex(ei->source())->property()
                  << ":"
                  << g.find_vertex(ei->target())->property()
                  << std::endl;
      }
    }
  }
}

bool
ITConnector::
InConnectedWorkspace(Cfg _cfg1, Cfg _cfg2){

	auto e = m_library->GetMPProblem()->GetEnvironment();
	if(e->SameTerrain(_cfg1,_cfg2))
		return true;
	else
		return false;

  if(_cfg1.GetRobot()->GetCapability() !=
      _cfg2.GetRobot()->GetCapability()){
    return false;
  }
  else if(_cfg1.GetRobot()->IsManipulator()){
    return true;
  }

	auto envTemp = m_library->GetMPProblem()->GetEnvironment();
	for(auto& terrain : envTemp->GetTerrains()){
		if(terrain.first != _cfg1.GetRobot()->GetCapability())
			continue;
		for(auto& t : terrain.second)
			if(t.InTerrain(_cfg1) and t.InTerrain(_cfg2))
				return true;	
	}
	return false;

  auto& g = m_capabilitySkeletons[_cfg1.GetRobot()->GetCapability()];

  if(!g) //indicates that no terrains are present to constrict the agent type
    return true;

  Point3d start = _cfg1.GetPoint();
  auto startVertex = g->FindNearestVertex(start);
  std::cout << "Start: " << startVertex->property() << std::endl;
  Point3d goal =  _cfg2.GetPoint();
  auto goalVertex = g->FindNearestVertex(goal);
  std::cout << "Goal: " << goalVertex->property() << std::endl;
  std::unordered_set<size_t> goalSet;
  goalSet.insert(goalVertex->descriptor());
  //std::unordered_set<WorkspaceSkeleton::vertex_iterator> goals = {goalVertex};
  SSSPTerminationCriterion<WorkspaceSkeleton> termination(
      [goalSet](typename WorkspaceSkeleton::vertex_iterator& _vi,
              const SSSPOutput<WorkspaceSkeleton>& _sssp) {
        //return (goalVertex->descriptor() == _vi->descriptor()) ? SSSPTermination::EndSearch
        return goalSet.count(_vi->descriptor()) ? SSSPTermination::EndSearch
                                              : SSSPTermination::Continue;
      }
  );

  SSSPPathWeightFunction<WorkspaceSkeleton> weight;
  weight = [this](typename WorkspaceSkeleton::adj_edge_iterator& _ei,
                   const double _sourceDistance,
                   const double _targetDistance) {
            return this->SkeletonPathWeight(_ei);
        };

  const SSSPOutput<WorkspaceSkeleton> sssp = DijkstraSSSP(g.get(),
          {startVertex->descriptor()}, weight, termination);

  auto last = sssp.ordering.back();
  if(!goalSet.count(last))
    return false;
  //else
  //  return true;
  auto env = m_library->GetMPProblem()->GetEnvironment();
  env->SaveBoundary();
  bool connected = env->IsolateTerrain(_cfg1,_cfg2);
  env->RestoreBoundary();
  return connected;
}



double
ITConnector::
SkeletonPathWeight(typename WorkspaceSkeleton::adj_edge_iterator& _ei) const {
  auto intermediates = _ei->property();
  auto dm = m_library->GetDistanceMetric("euclidean");
  double distance = 0.0;
  for(size_t i = 1; i < intermediates.size(); i++){
    double step = (intermediates[i-1]-intermediates[i]).norm();
    distance += step;
  }
  return distance;
}
	
void
ITConnector::
DirectionConnections(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _graph, std::string _capability, 
											std::vector<Cfg*> _cfgs) {
	std::vector<size_t> vids;
	std::unordered_set<size_t> vidSet;

	for(auto cfg : _cfgs) {
		if(cfg->GetRobot()->GetCapability() == _capability) {
			auto vid = _graph->GetVID(*cfg);
			if(vid != MAX_INT) {
				vids.push_back(vid);
				vidSet.insert(vid);
			}
		}
	}

	auto cm = m_library->GetConnector("Closest");
	cm->Connect(_graph, vids.begin(), vids.end(),
										 &vidSet); 
}
