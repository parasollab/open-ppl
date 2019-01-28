#include "TMPStrategyMethod.h"
#include "ITConstructor.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"
#include "Simulator/Simulation.h"


TMPStrategyMethod::
TMPStrategyMethod(XMLNode& _node){
  m_useITs = _node.Read("useITs", false, m_useITs,
                        "Indicate if the TMP Strategy should use ITs when planning.");
  m_debug = _node.Read("debug", false, m_debug,
                       "Indicate if the TMP Strategy should output debug information.");

}

TaskPlan
TMPStrategyMethod::
PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
          vector<std::shared_ptr<MPTask>> _tasks, Robot* _superRobot,
          std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>* _ITPlacementMethods){
  return TaskPlan();
}

void
TMPStrategyMethod::
ResetCapabilityRoadmaps(){
  m_capabilityRoadmaps.clear();
}


void
TMPStrategyMethod::
GenerateCapabilityRoadmaps(MPLibrary* _library, vector<HandoffAgent*> _agents,
                           vector<std::shared_ptr<MPTask>> _tasks, Robot* _superRobot,
                           std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                                              _ITPlacementMethods){
  if(m_useITs){
    GenerateInteractionTemplates(_library, _agents, _superRobot);
    PlaceInteractionTemplates(_library, _superRobot, _ITPlacementMethods);
  }

  for(auto& r : _superRobot->GetMPProblem()->GetRobots()){
    r->SetVirtual(true);
  }

  GenerateDummyAgents(_agents);

  for(auto const& elem : m_dummyMap){
    auto capability = elem.first;
    Simulation::GetStatClass()->StartClock("Construct CapabilityMap " + capability);
    HandoffAgent* dummyAgent = elem.second;
    dummyAgent->InitializeRoadmap();

    std::unordered_map<size_t, size_t> handoffVIDMap;
    std::unordered_map<size_t, size_t> inverseVIDMap;
    auto graph = dummyAgent->GetMPSolution()->GetRoadmap()->GetGraph();

    //TODO::Extract into own function where IT instances are copied to
    //capability graphs
    Simulation::GetStatClass()->StartClock("Copy Handoffs to CapabilityMap " + capability);
    // Copy cfgs of handoffs of same capability into corresponding capability map

    for(size_t i = 0; i < m_transformedRoadmaps.size(); i++){
      auto roadmap = m_transformedRoadmaps[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = 0; j < roadmap.size(); j++){
          auto vid = roadmap[j];
          const auto newVID = graph->AddVertex(m_megaRoadmap->GetVertex(vid));
          handoffVIDMap[newVID] = vid;
          inverseVIDMap[vid] = newVID;
        }
      }
    }
    //Adding edges of handoffs
    for(auto vit = graph->begin(); vit != graph->end(); vit++){
      auto vid1 = vit->descriptor();
      auto orig1 = handoffVIDMap[vid1];
      for(auto vit2 = graph->begin(); vit2 != graph->end(); vit2++){
        if(vit == vit2) continue;
        auto vid2 = vit2->descriptor();
        auto orig2 = handoffVIDMap[vid2];
        if(m_megaRoadmap->IsEdge(orig1, orig2)){
          auto edge = m_megaRoadmap->GetEdge(orig1, orig2);
          graph->AddEdge(vid1, vid2, edge);
        }

      }
    }

    Simulation::GetStatClass()->StopClock("Copy Handoffs to CapabilityMap " + capability);

    if(m_debug){
      std::cout << "Done initializing capability map with corresponding cfgs from megaRoadmap"
                << std::endl;
    }

    //TODO::Extract to separate function for connecting IT instances
    // Connect handoff templates of the same capability.
    for(size_t i = 0; i < m_transformedRoadmaps.size(); i++){
      auto roadmap = m_transformedRoadmaps[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = i+1; j < m_transformedRoadmaps.size(); j++){
          auto roadmap2 = m_transformedRoadmaps[j];
          if(capability == m_megaRoadmap->GetVertex(roadmap2[0]).GetRobot()->
              GetAgent()->GetCapability()){
            // Find point in both roadmaps and try to connect them
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent, _superRobot, _library);
          }
        }
      }
    }

    if(m_debug){
      std::cout << "Done connecting handoff templates of same capability" << std::endl;
    }

    // Attempt to connect the whole task start/end points to each
    // other in the capability maps
    for(size_t i = 0; i < m_wholeTaskStartEndPoints.size(); i++){
      auto roadmap = m_wholeTaskStartEndPoints[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
            GetAgent()->GetCapability()){
        for(size_t j = 0; j < roadmap.size(); j++){
          auto vid = roadmap[j];
          const auto newVID = graph->AddVertex(m_megaRoadmap->GetVertex(vid));
          handoffVIDMap[newVID] = vid;
          inverseVIDMap[vid] = newVID;
        }
      }
    }

    // Attempt to connect the whole task start/end points to the
    // ITs in the capability map
    for(size_t i = 0; i < m_wholeTaskStartEndPoints.size(); i++){
      auto roadmap = m_wholeTaskStartEndPoints[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = 0; j < m_transformedRoadmaps.size(); j++){
          auto roadmap2 = m_transformedRoadmaps[j];
          if(capability == m_megaRoadmap->GetVertex(roadmap2[0]).GetRobot()->
              GetAgent()->GetCapability()){
            // Find point in both roadmaps and try to connect them
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent, _superRobot, _library);
          }
        }
      }
    }

    // Attempt to connect the start/end points
    for(size_t i = 0; i < m_wholeTaskStartEndPoints.size(); i++){
      auto roadmap = m_wholeTaskStartEndPoints[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = 0; j < m_wholeTaskStartEndPoints.size(); j++){
          if(i == j) continue;
          auto roadmap2 = m_wholeTaskStartEndPoints[j];
          if(capability == m_megaRoadmap->GetVertex(roadmap2[0]).GetRobot()->
              GetAgent()->GetCapability()){
            // Find point in both roadmaps and try to connect them
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent, _superRobot, _library);
          }
        }
      }
    }

    m_capabilityRoadmaps[capability] = graph;

    Simulation::GetStatClass()->StopClock("Construct CapabilityMap " + capability);
    Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
    // Copy over newly found vertices
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      // Add vertices found in building the capability map to the mega roadmap
      // without copying over the handoff vertices that already exist
      if(handoffVIDMap.find(vit->descriptor()) == handoffVIDMap.end()){
        auto megaVID = m_megaRoadmap->AddVertex(vit->property());
        handoffVIDMap[vit->descriptor()] = megaVID;
      }
    }

    if(m_debug){
      std::cout << "Done copying over vertices" << std::endl;
    }

    // Copy over newly found edges in capability to mega roadmap
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      for(auto eit = vit->begin(); eit != vit->end(); ++eit){
        auto source = handoffVIDMap[eit->source()];
        auto target = handoffVIDMap[eit->target()];
        // Won't copy over existing edges to the mega roadmap
        if(!m_megaRoadmap->IsEdge(source, target)){
            m_megaRoadmap->AddEdge(source, target, eit->property());
        }
      }
    }

    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
    if(m_debug){
      std::cout << "Done copying over edges" << std::endl;
    }
  }

  //Printing megaroadmap
  if(m_debug){
    std::cout << "Printing m_megaRoadmap after constructing capability maps" << std::endl;
    for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
      std::cout << vit->property().PrettyPrint() << std::endl;
    }
  }

  for(auto& r : _superRobot->GetMPProblem()->GetRobots()){
    if(r.get() != _superRobot){
      r->SetVirtual(true);
    }
  }

}

void
TMPStrategyMethod::
GenerateInteractionTemplates(MPLibrary* _library, vector<HandoffAgent*> _agents, Robot* _superRobot){
  if(m_debug){
    std::cout << "Finding Handoff Locations" << std::endl;
  }
  for(auto& info : _library->GetMPProblem()->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    _library->GetMPSolution()->AddInteractionTemplate(it);
  }
  // Loop through interaction templates

  ITConstructor constructor(_library, _agents, _superRobot);
  for(auto& it : _library->GetMPSolution()->GetInteractionTemplates()){
    constructor.ConstructIT(it.get());
  }
}

void
TMPStrategyMethod::
PlaceInteractionTemplates(MPLibrary* _library, Robot* _superRobot,
                          std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                          _ITPlacementMethods){

  if(m_debug){
    std::cout << "Finding Handoff Locations" << std::endl;
  }
  auto originalProblem = _superRobot->GetMPProblem();
  _library->SetMPProblem(originalProblem);

  for(auto& it : _library->GetMPSolution()->GetInteractionTemplates()){
    FindITLocations(it.get(), _library, _superRobot, _ITPlacementMethods);
  }

  std::cout << "Found Handoff Locations" << std::endl;
  Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
  auto vcm = _library->GetValidityChecker("terrain_solid");
  for(auto& currentTemplate : _library->GetMPSolution()->GetInteractionTemplates()){

    if(m_debug){
      auto g = currentTemplate->GetConnectedRoadmap();
      std::cout << "Original handoff position" << std::endl;
      for(auto vit = g->begin(); vit!=g->end(); vit++){
        std::cout << vit->property().PrettyPrint() << std::endl;
      }
    }

    Simulation::GetStatClass()->StartClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
    for(auto centerCfg : currentTemplate->GetInformation()->GetTemplateLocations()){
      Simulation::GetStatClass()->StartClock("Placement InteractionTemplate "
                + currentTemplate->GetInformation()->GetLabel());

      RoadmapGraph<Cfg, DefaultWeight<Cfg>>* graph = currentTemplate->GetConnectedRoadmap();

      // Copy vertices and map the change in VIDs.
      std::unordered_map<VID, VID> oldToNew;
      for(auto vit = graph->begin(); vit != graph->end(); ++vit) {
        const VID oldVID = vit->descriptor();
        auto relativeCfg = vit->property();
        TranslateCfg(centerCfg, relativeCfg);

        //bool isValid = vcm->IsValid(relativeCfg, "ValidateITCfg");
        //if(isValid){
          const VID newVID = m_megaRoadmap->AddVertex(relativeCfg);
          oldToNew[oldVID] = newVID;
        //}
      }

      // Keep track of the distinct transformed handoff roadmaps
      for(auto distinctRoadmap : currentTemplate->GetDistinctRoadmaps()) {
        std::vector<size_t> transformedRoadmap;
        for(auto vid : distinctRoadmap) {
          transformedRoadmap.push_back(oldToNew[vid]);
        }
        m_transformedRoadmaps.push_back(transformedRoadmap);
      }


      if(m_debug){
        auto r = m_transformedRoadmaps.front();
        std::cout << "Transformed Postion of Roadmap" << std::endl;
        for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
          std::cout << vit->descriptor() << std::endl;
          std::cout << vit->property().PrettyPrint() << std::endl;
        }
        for(auto v : r){
          std::cout << m_megaRoadmap->GetVertex(v).PrettyPrint() << std::endl;
        }
      }

      // Copy edges into the mega roadmap
      for(auto vit = graph->begin(); vit != graph->end(); ++vit) {
        for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
          const VID source = oldToNew[eit->source()];
          const VID target = oldToNew[eit->target()];
          if(!m_megaRoadmap->IsEdge(source, target)){
            // Call translate cfg on the all the intermediates and built
            // up a new vector of intermediates to store in the edge property
            // before storing it in the megaRoadmap
            std::vector<Cfg> intermediates = eit->property().GetIntermediates();
            for(auto cfg : intermediates){
              TranslateCfg(centerCfg, cfg);
            }
            m_megaRoadmap->AddEdge(source, target, eit->property());
          }
        }
      }
    }
    Simulation::GetStatClass()->StopClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
  }
  //Get rid of cfgs that are invalid after placement
  /*for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
    auto cfg = vit->property();
    auto vcm = m_library->GetValidityChecker("terrain_solid");
    bool isValid = vcm->IsValid(cfg, "ValidateITCfg");
    if(!isValid){
      m_megaRoadmap->DeleteVertex(vit->descriptor());
    }
  }*/


  Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
}

void
TMPStrategyMethod::
CopyCapabilityRoadmaps(vector<HandoffAgent*> _agents, Robot* _superRobot){
  for(auto agent : _agents){

    //Copy corresponding capability roadmap into agent
    auto graph = m_capabilityRoadmaps[agent->GetCapability()];
    auto g = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(agent->GetRobot());
    *g = *graph;
    g->SetRobot(agent->GetRobot());
    agent->SetRoadmapGraph(g);
    if(m_debug){
      auto g2 = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(agent->GetRobot());
      *g2 = *graph;
      Roadmap<MPTraits<Cfg>> testRm(agent->GetRobot());
      testRm.SetGraph(g2);
      testRm.Write(agent->GetRobot()->GetLabel() + ".map", _superRobot->GetMPProblem()->GetEnvironment());
    }
  }
}

void
TMPStrategyMethod::
FindITLocations(InteractionTemplate* _it, MPLibrary* _library, Robot* _superRobot,
                std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>* _ITPlacementMethods){
  if(!_ITPlacementMethods or _ITPlacementMethods->size() == 0){
    throw RunTimeException(WHERE, "No IT Placement Methods provided to the TMP Strategy Method.");
  }
  for(auto& method : *_ITPlacementMethods){
    Simulation::GetStatClass()->StartClock("Placing Templates with: " + method.second->GetLabel());
    method.second->PlaceIT(_it, _library->GetMPSolution(), _library,
                           static_cast<Coordinator*>(_superRobot->GetAgent()));
    Simulation::GetStatClass()->StopClock("Placing Templates with: " + method.second->GetLabel());
  }
}

void
TMPStrategyMethod::
TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg){
  double x = _relativeCfg[0];
  double y = _relativeCfg[1];
  double theta = _centerCfg[2];

  double newX = x*cos(theta) - y*sin(theta);
  double newY = x*sin(theta) + y*cos(theta);
  double oldTheta = _relativeCfg[2];

  _relativeCfg.SetLinearPosition({newX, newY, oldTheta});

  _relativeCfg += _centerCfg;
}

void
TMPStrategyMethod::
GenerateDummyAgents(std::vector<HandoffAgent*> _agents){
  // Load the dummyMap, which stores a dummy agent for each agent capability.
  for(auto agent : _agents){
    std::string capability = agent->GetCapability();
    if(m_dummyMap.find(capability) == m_dummyMap.end()){
      m_dummyMap[capability] = agent;
    }
  }
}


void
TMPStrategyMethod::
ConnectDistinctRoadmaps(vector<size_t> _roadmap1, vector<size_t> _roadmap2,
    HandoffAgent* _agent, Robot* _superRobot, MPLibrary* _library) {
  std::cout << "Connecting Distinct Roadmaps" << std::endl;
  Robot* dummyRobot = _agent->GetRobot();
  auto originalProblem = _library->GetMPProblem();
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*dummyRobot->GetMPProblem()));
  // Loop through each roadmap until valid cfgs from both are found.

  //TODO: Check if the roadmaps are reachable from the other
  //For now, just select random cfgs from each and try to connect
  for(auto vid1 : _roadmap1) {
    Cfg cfg1 = m_megaRoadmap->GetVertex(vid1);
    auto cfg1Copy = cfg1;
    cfg1Copy.SetRobot(dummyRobot);
    if(!_library->GetValidityChecker("terrain_solid")->IsValid(cfg1Copy, "cfg1"))
      continue;

    for(auto vid2 : _roadmap2) {
      Cfg cfg2 = m_megaRoadmap->GetVertex(vid2);
      auto cfg2Copy = cfg2;
      cfg2Copy.SetRobot(dummyRobot);
      if(vid1 == vid2 || cfg1Copy == cfg2Copy) continue;
      if(!_library->GetValidityChecker("terrain_solid")->IsValid(cfg2Copy, "cfg2"))
        continue;

      // Attempt to plan between the two roadmaps
      MPTask* tempTask = new MPTask(dummyRobot);
      std::unique_ptr<CSpaceConstraint> start(new CSpaceConstraint(dummyRobot, cfg1Copy));
      std::unique_ptr<CSpaceConstraint> goal(new CSpaceConstraint(dummyRobot, cfg2Copy));
      tempTask->SetStartConstraint(std::move(start));
      tempTask->AddGoalConstraint(std::move(goal));

      if(m_debug){
        std::cout << "Calling solve" << std::endl;
      }
      // Solve for manipulator robot team
      if(_superRobot->IsManipulator()){
        _library->Solve(problemCopy.get(), tempTask, _agent->GetMPSolution(),
            "EvaluateMapStrategy", LRand(), "ConnectingDistinctRoadmaps");
      }
      // Solve for non-manipulator robot team
      else{
        _library->Solve(problemCopy.get(), tempTask, _agent->GetMPSolution(),
            "LazyPRM", LRand(), "ConnectingDistinctRoadmaps");
      }

      if(m_debug){
        std::cout << "Finish solving" << std::endl;
      }

      if(!_agent->GetMPSolution()->GetPath()->Cfgs().empty()) {
        if(m_debug){
          std::cout << "Finishing connecting distinct roadmaps" << std::endl;
        }
        _library->SetMPProblem(originalProblem);
        return;
      }
    }
  }
  _library->SetMPProblem(originalProblem);
}
