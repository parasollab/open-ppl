#include "TMPStrategyMethod.h"
#include "ITConstructor.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/TMPStrategies/ITConnector.h"
#include "Simulator/Simulation.h"


/*****************************************Constructor*****************************************************/
TMPStrategyMethod::
TMPStrategyMethod(XMLNode& _node){
  m_useITs = _node.Read("useITs", false, m_useITs,
                        "Indicate if the TMP Strategy should use ITs when planning.");
  m_debug = _node.Read("debug", false, m_debug,
                       "Indicate if the TMP Strategy should output debug information.");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric for checking "
      "nearest agents and charging locations.");
  m_connectionThreshold = _node.Read("connectionThreshold",true,1.2, 0., 1000.,
      "Acceptable variabliltiy in IT paths.");

}

/******************************************Configure*****************************************************/

void 
TMPStrategyMethod::
Initialize(Robot* m_robot){
  m_initialized = true;

  m_robot = m_robot;

  m_memberAgents.clear();

  GenerateDummyAgents(); 
  
  ResetCapabilityRoadmaps();

  m_wholeTasks.clear();
  m_wholeTaskStartEndPoints.clear();

  delete m_megaRoadmap;
  m_megaRoadmap = nullptr;

  m_library = nullptr;

  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));

  /*GenerateDummyAgents();
  if(m_debug){
    std::cout << "Done Generating Dummy Agents" << std::endl;
  }

  GenerateITs();
  if(m_debug){
    std::cout << "Done Generating Handoff Templates" << std::endl;
  }

  m_megaRoadmap = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(m_robot);
  // Setting library task to set robot
  auto task = m_library->GetMPProblem()->GetTasks(m_robot).front();
  m_library->SetTask(task.get());

  CreateCombinedRoadmap();*/
}

void
TMPStrategyMethod::
ResetCapabilityRoadmaps(){
  m_capabilityRoadmaps.clear();
  m_transformedRoadmaps.clear();;
}

/****************************************Call Method*****************************************************/

TaskPlan
TMPStrategyMethod::
PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
          vector<std::shared_ptr<MPTask>> _tasks){
  if(!m_initialized){
	throw RunTimeException(WHERE, "TMPStrategyMethod not initialized.");
  }

  m_memberAgents = _agents;

  CreateWholeTasks(_tasks);

  return TaskPlan();
}

/**************************************Combined Roadmap**************************************************/

void
TMPStrategyMethod::
CreateCombinedRoadmap(){
  for(auto agent : m_memberAgents){
    agent->GetRobot()->SetVirtual(true);
  }
  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  //m_library->InitializeMPProblem(originalProblem);
  m_library->SetMPProblem(originalProblem);

  for(auto& it : m_solution->GetInteractionTemplates()){
    //m_solution->AddInteractionTemplate(it);
    FindITLocations(it.get());
  }

  std::cout << "Found Handoff Locations" << std::endl;

  TransformITs();
  SetupWholeTasks();

  ITConnector connector(m_connectionThreshold,m_library);
  auto dm = m_library->GetDistanceMetric(m_dmLabel);

  if(true){
    for(auto agent : m_memberAgents){
      auto robot = agent->GetRobot();
      auto cfg = robot->GetSimulationModel()->GetState();
      auto vid = m_megaRoadmap->AddVertex(cfg);
	  std::vector<size_t> vids = {vid};
      m_wholeTaskStartEndPoints.push_back(vids);
    }
  }

  std::vector<Cfg> startAndGoal;
  for(auto vid : m_wholeTaskStartEndPoints){
    startAndGoal.push_back(m_megaRoadmap->GetVertex(vid[0]));
  }

  for(auto it = m_dummyMap.begin(); it != m_dummyMap.end(); it++){
    const std::string capability = it->first;
    auto graph = connector.ConnectInteractionTemplates(
                          m_solution->GetInteractionTemplates(),
                          capability,
                          startAndGoal,
                          m_megaRoadmap);

    auto robot = it->second->GetRobot();
    graph->Write("CoordinatorTemplates.map", robot->GetMPProblem()->GetEnvironment());

    std::priority_queue<std::pair<size_t,size_t>> pq;

    Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
    // Copy over newly found vertices
    std::unordered_map<size_t,size_t> handoffVIDMap;
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

    m_capabilityRoadmaps[capability] = std::shared_ptr<GraphType>(graph);
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

    m_megaRoadmap->Write("MegaTemplates.map", robot->GetMPProblem()->GetEnvironment());
    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
  }
}

void
TMPStrategyMethod::
GenerateITs(){
  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  m_library->SetMPProblem(originalProblem);

  for(auto& info : originalProblem->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    m_solution->AddInteractionTemplate(it);
    //FindITLocations(it);
  }

  std::cout << "Found Handoff Locations" << std::endl;

  // Loop through handoff templates, set start constraints for handoff, set
  // dummy robot for handoff task by capability, and solve handoff task.
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  problemCopy->SetEnvironment(std::move(m_handoffEnvironment));
  m_library->SetMPProblem(problemCopy.get());
  // Set robots to virtual so that planning handoffs does not cause collisions
  std::list<HandoffAgent*> unusedAgents;


  for(auto& currentTemplate : m_solution->GetInteractionTemplates()){
    Simulation::GetStatClass()->StartClock("Construct InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());

    unusedAgents.clear();
    std::copy(m_memberAgents.begin(), m_memberAgents.end(), std::back_inserter(unusedAgents));
    auto handoffTasks = currentTemplate->GetInformation()->GetInteractionTasks();
    std::unordered_map<std::shared_ptr<MPTask>, HandoffAgent*> agentTasks;
    // Loop through all tasks and assign a robot of matching capability to the
    // task, then configuring the robot at the goal constraint.
    for(auto task : handoffTasks){
      m_library->SetTask(task.get());
      for(auto agent : unusedAgents){
        if(agent->GetCapability() == task->GetCapability()){
          agentTasks[task] = agent;
          Robot* tempRobot = problemCopy->GetRobot(agent->GetRobot()->GetLabel());
          task->SetRobot(tempRobot);
          unusedAgents.remove(agent);
          // Confiure tempRobot at the goal constraint for the task
          //        - Sample at the point of the goal constraint
          //        - Get CFG from sample and place tempRobot there
          auto boundingBox = task->GetGoalConstraints().front()->
              GetBoundary();
          std::vector<Cfg> goalPoints;

          MPSolution* sol = new MPSolution(m_robot);
          m_library->SetMPSolution(sol);
          m_library->SetTask(task.get());
          auto sampler = m_library->GetSampler("UniformRandomFree");
          size_t numNodes = 1, numAttempts = 100;
          tempRobot->SetVirtual(true);
          sampler->Sample(numNodes, numAttempts, boundingBox,
              std::back_inserter(goalPoints));

          tempRobot->SetVirtual(false);
          if(goalPoints.empty())
            throw RunTimeException(WHERE, "No valid final handoff position for the robot.");

          goalPoints[0].ConfigureRobot();
          break;
        }
      }
    }
    // Set the unused agents to virtual before planning.
    for(auto agent : unusedAgents){
      auto robot = problemCopy->GetRobot(agent->GetRobot()->GetLabel());
      robot->SetVirtual(true);
    }
    int check = 0;
    for(auto task : handoffTasks){
      Robot* taskRobot = problemCopy->GetRobot(agentTasks[task]->GetRobot()->GetLabel());
      std::unique_ptr<MPSolution> handoffSolution(new MPSolution(taskRobot));
      // Store the current configuration of the robot, since the multibody
      // will be moved while solving.
      auto currentConfig = taskRobot->GetMultiBody()->GetCurrentDOFs();
      if(m_debug){
        for(auto& robot : problemCopy->GetRobots()){
          std::cout << robot->GetLabel() << " - " << robot.get()
                    << ": " << robot->GetMultiBody()->GetCurrentDOFs()
                    << " - " << robot->IsVirtual() << std::endl;
        }
      }

      task->SetRobot(taskRobot);
      // Solve for non-mainpulator robot teams
      if(!taskRobot->IsManipulator()){
        m_library->Solve(problemCopy.get(), task.get(), handoffSolution.get());
      }
      // Solve for manipulator robot teams
      else {
        std::vector<Cfg> startPoints;
        MPSolution* sol = new MPSolution(m_robot);
        m_library->SetMPSolution(sol);
        auto sampler = m_library->GetSampler("UniformRandomFree");
        size_t numNodes = 1, numAttempts = 100;
        auto boundingBox = task->GetStartConstraint()->GetBoundary();
        m_library->SetTask(task.get());
        sampler->Sample(numNodes, numAttempts, boundingBox,
            std::back_inserter(startPoints));

        if(startPoints.empty())
          throw RunTimeException(WHERE, "No valid start handoff position for the robot.");

        startPoints[0].ConfigureRobot();
        std::cout << startPoints[0].PrettyPrint() << std::endl;

        m_library->Solve(problemCopy.get(), task.get(),
            handoffSolution.get(), "FixedPRM", LRand(), "FixedPRM");
      }

      taskRobot->GetMultiBody()->Configure(currentConfig);

      if(m_debug){
        std::cout << "Size of path: " << handoffSolution->GetPath()->Cfgs().size() << std::endl;
        for(auto cfg : handoffSolution->GetPath()->Cfgs()){
          std::cout << cfg.PrettyPrint() << std::endl;
        }
      }


      handoffSolution->GetRoadmap()->Write("indHandoffTemplate" + std::to_string(check) + ".map", problemCopy->GetEnvironment());
      check++;

      // Store the roadmap for each task in the handoff
      auto rob = handoffSolution->GetRoadmap()->GetRobot();
      handoffSolution->GetRoadmap()->SetRobot(originalProblem->GetRobot(rob->GetLabel()));
      currentTemplate->AddRoadmap(handoffSolution->GetRoadmap());
      currentTemplate->AddPath(handoffSolution->GetPath()->Cfgs(), originalProblem);

      if(currentTemplate->GetInformation()->SavedPaths()){
        //currentTemplate->AddPath(handoffSolution->GetPath()->Cfgs()),
           // handoffSolution->GetPath()->Length());
        std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
        currentTemplate->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), originalProblem);
        std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().back() << std::endl;
      }
      else{
        // Add final configuration of path to template
        std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
        currentTemplate->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), originalProblem);
        std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().back() << std::endl;
      }
    }


    currentTemplate->ConnectRoadmaps(m_robot, originalProblem);

    Simulation::GetStatClass()->StopClock("Construct InteractionTemplate "
                + currentTemplate->GetInformation()->GetLabel());
    Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
                            +"::Vertices", currentTemplate->GetConnectedRoadmap()->get_num_vertices());
    Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
                            +"::Edges", currentTemplate->GetConnectedRoadmap()->get_num_vertices());

    size_t count = 0;
    for(auto rm : currentTemplate->GetRoadmaps()){
      count++;
      Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
          +"::"+std::to_string(count)
          +"::Vertices", rm->get_num_vertices());
      Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
          +"::"+std::to_string(count)
          +"::Edges", rm->get_num_vertices());
    }

    std::cout << "Trying to write handoffTemplate Map" << std::endl;
    currentTemplate->GetConnectedRoadmap()->Write("handoffTemplate.map",
        problemCopy->GetEnvironment());

    // Reset the agents to non-virtual, since they could be used in the next
    // template.
    for(auto agent : unusedAgents){
      problemCopy->GetRobot(agent->GetRobot()->GetLabel())->SetVirtual(false);
    }
  }


  m_library->SetMPProblem(originalProblem);
  m_library->SetMPSolution(m_solution.get());
  m_library->SetTask(originalProblem->GetTasks(m_robot)[0].get());
}

void
TMPStrategyMethod::
FindITLocations(InteractionTemplate* _it){
  for(auto& method : m_ITPlacementMethods){
    Simulation::GetStatClass()->StartClock("Placing Templates with: " + method.second->GetLabel());
    method.second->PlaceIT(_it, m_library->GetMPSolution(), m_library,
                           static_cast<Coordinator*>(m_robot->GetAgent()));
    Simulation::GetStatClass()->StopClock("Placing Templates with: " + method.second->GetLabel());
  }
}


void
TMPStrategyMethod::
TransformITs(){

  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  m_library->SetMPProblem(originalProblem);

  std::cout << "Found Handoff Locations" << std::endl;
  Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
  auto vcm = m_library->GetValidityChecker("terrain_solid");
  for(auto& currentTemplate : m_solution->GetInteractionTemplates()){

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
        relativeCfg.TransformCfg(centerCfg);

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

      m_megaRoadmap->InstallHook(RoadmapType::HookType::AddEdge, "debug",
          [](RoadmapType::EI _ei) {
          if(_ei->property().GetWeight()==0){
          std::cout << "Zero weight edge" << std::endl;
          }
          });
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
              cfg.TransformCfg(centerCfg);
            }
            m_megaRoadmap->AddEdge(source, target, eit->property());
          }
        }
      }
      m_megaRoadmap->RemoveHook(RoadmapType::HookType::AddEdge, "debug");
    }
    Simulation::GetStatClass()->StopClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
  }
  
  Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
}

void
TMPStrategyMethod::
SetupWholeTasks(){
  for(auto wholeTask : m_wholeTasks){
    // find a start and goal configuration for the coordinator
    auto task = wholeTask->m_task;
	m_library->SetTask(task.get());
    auto startBox = task->GetStartConstraint()->GetBoundary();
    std::vector<Cfg> startPoints;
    auto sampler = m_library->GetSampler("UniformRandomFree");
    size_t numNodes = 1, numAttempts = 100;
    sampler->Sample(numNodes, numAttempts, startBox,
        std::back_inserter(startPoints));

    if(startPoints.empty())
      throw RunTimeException(WHERE, "No valid initial position for the robot.");

    auto goalBox = task->GetGoalConstraints().front()->GetBoundary();
    std::vector<Cfg> goalPoints;
    sampler->Sample(numNodes, numAttempts, goalBox,
        std::back_inserter(goalPoints));

    if(goalPoints.empty())
      throw RunTimeException(WHERE, "No valid goal position for the robot.");

    wholeTask->m_startPoints[m_robot->GetLabel()] = {startPoints[0]};
    wholeTask->m_goalPoints[m_robot->GetLabel()] = {goalPoints[0]};

    // Loop through each type of capability then push start/goal constraints
    // into vectors in WholeTask
    for(auto const& elem : m_dummyMap) {
      // Set library robot to the corresponding capability
      task->SetRobot(elem.second->GetRobot());
      m_library->SetTask(task.get());
      // Sample to find valid start and goal points in the environment
      auto startBox = task->GetStartConstraint()->GetBoundary();
      std::vector<Cfg> startPoints;
      auto sampler = m_library->GetSampler("UniformRandomFreeTerrain");
      size_t numNodes = 1, numAttempts = 100;
      sampler->Sample(numNodes, numAttempts, startBox,
          std::back_inserter(startPoints));

      if(!startPoints.empty())
        wholeTask->m_startPoints[elem.second->GetCapability()].push_back(startPoints[0]);

      auto goalBox = task->GetGoalConstraints().front()->GetBoundary();
      std::vector<Cfg> goalPoints;
      sampler->Sample(numNodes, numAttempts, goalBox,
          std::back_inserter(goalPoints));

      if(!goalPoints.empty())
        wholeTask->m_goalPoints[elem.second->GetCapability()].push_back(goalPoints[0]);

    }

    task->SetRobot(m_robot);
    m_library->SetTask(task.get());

    // Create 0 weight edges between each capability and the coordinator
    // configuration.
    auto coordinatorStartVID = m_megaRoadmap->AddVertex(
                    wholeTask->m_startPoints[m_robot->GetLabel()][0]);

    wholeTask->m_startVIDs[m_robot->GetLabel()] = {coordinatorStartVID};

    auto coordinatorGoalVID = m_megaRoadmap->AddVertex(wholeTask->m_goalPoints[m_robot->GetLabel()][0]);

    wholeTask->m_goalVIDs[m_robot->GetLabel()] = {coordinatorGoalVID};

    const DefaultWeight<Cfg> weight;

    Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
    for(auto const& elem : wholeTask->m_startPoints){
      if(elem.first == m_robot->GetLabel())
        continue;

      for(auto start : elem.second) {
        auto agentStartVID = m_megaRoadmap->AddVertex(start);

        // Add the start points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentStartVID});
        wholeTask->m_startVIDs[elem.first].push_back(agentStartVID);
        m_megaRoadmap->AddEdge(coordinatorStartVID, agentStartVID, {weight,weight});
      }

    }

    for(auto const& elem : wholeTask->m_goalPoints){
      if(elem.first == m_robot->GetLabel())
        continue;

      for(auto goal : elem.second) {
        auto agentGoalVID = m_megaRoadmap->AddVertex(goal);

        // Add the end points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentGoalVID});
        wholeTask->m_goalVIDs[elem.first].push_back(agentGoalVID);
        m_megaRoadmap->AddEdge(coordinatorGoalVID, agentGoalVID, {weight,weight});
      }

    }
    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
  }
  m_library->SetTask(m_robot->GetMPProblem()->GetTasks(m_robot)[0].get());
}


/**************************************Helper Methods**************************************************/

void
TMPStrategyMethod::
GenerateDummyAgents(){
  m_dummyMap.clear();
  // Load the dummyMap, which stores a dummy agent for each agent capability.
  for(auto agent : m_memberAgents){
    std::string capability = agent->GetCapability();
    if(m_dummyMap.find(capability) == m_dummyMap.end()){
      m_dummyMap[capability] = agent;
    }
  }
}

void
TMPStrategyMethod::
CreateWholeTasks(std::vector<std::shared_ptr<MPTask>> _tasks){
  for(auto task : _tasks){
    WholeTask* wholeTask = new WholeTask();
    wholeTask->m_task = task;
    for(auto const& elem : m_dummyMap){
      wholeTask->m_startPoints[elem.first] = {};
      wholeTask->m_goalPoints[elem.first] = {};
      wholeTask->m_startVIDs[elem.first] = {};
      wholeTask->m_goalVIDs[elem.first] = {};
    }
    m_wholeTasks.push_back(wholeTask);
  }
}




















/*
void
TMPStrategyMethod::
GenerateCapabilityRoadmaps(MPLibrary* _library, vector<HandoffAgent*> _agents,
                           vector<std::shared_ptr<MPTask>> _tasks, Robot* m_robot,
                           std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                                              _ITPlacementMethods){
  if(m_useITs){
    GenerateInteractionTemplates(_library, _agents, m_robot);
    PlaceInteractionTemplates(_library, m_robot, _ITPlacementMethods);
  }

  for(auto& r : m_robot->GetMPProblem()->GetRobots()){
    r->SetVirtual(true);
  }

  GenerateDummyAgents();

  for(auto const& elem : m_dummyMap){
    auto capability = elem.first;
    Simulation::GetStatClass()->StartClock("Construct CapabilityMap " + capability);
    HandoffAgent* dummyAgent = elem.second;
    dummyAgent->InitializeRoadmap();

    std::unordered_map<size_t, size_t> handoffVIDMap;
    std::unordered_map<size_t, size_t> inverseVIDMap;
    auto graph = dummyAgent->GetMPSolution()->GetRoadmap();

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
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent, m_robot, _library);
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
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent, m_robot, _library);
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
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent, m_robot, _library);
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

  for(auto& r : m_robot->GetMPProblem()->GetRobots()){
    if(r.get() != m_robot){
      r->SetVirtual(true);
    }
  }

}

void
TMPStrategyMethod::
GenerateInteractionTemplates(MPLibrary* _library, vector<HandoffAgent*> _agents, Robot* m_robot){
  for(auto& info : _library->GetMPProblem()->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    _library->GetMPSolution()->AddInteractionTemplate(it);
  }
  // Loop through interaction templates

  ITConstructor constructor(_library, _agents, m_robot);
  for(auto& it : _library->GetMPSolution()->GetInteractionTemplates()){
    constructor.ConstructIT(it.get());
  }
}

void
TMPStrategyMethod::
PlaceInteractionTemplates(MPLibrary* _library, Robot* m_robot,
                          std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                          _ITPlacementMethods){

  if(m_debug){
    std::cout << "Finding IT Locations" << std::endl;
  }
  auto originalProblem = m_robot->GetMPProblem();
  _library->SetMPProblem(originalProblem);

  for(auto& it : _library->GetMPSolution()->GetInteractionTemplates()){
    FindITLocations(it.get(), m_robot);
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
        relativeCfg.TransformCfg(centerCfg);

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
              cfg.TransformCfg(centerCfg);
            }
            m_megaRoadmap->AddEdge(source, target, eit->property());
          }
        }
      }
    }
    Simulation::GetStatClass()->StopClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
  }
  Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
}

void
TMPStrategyMethod::
CopyCapabilityRoadmaps(vector<HandoffAgent*> _agents, Robot* m_robot){
  for(auto agent : _agents){

    //Copy corresponding capability roadmap into agent
    auto graph = m_capabilityRoadmaps[agent->GetCapability()];
    auto g = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(agent->GetRobot());
    *g = *graph;
    g->SetRobot(agent->GetRobot());
    agent->SetRoadmapGraph(g);
    if(m_debug){
      graph->Write(agent->GetRobot()->GetLabel() + ".map",
                   m_robot->GetMPProblem()->GetEnvironment());
    }
  }
}

void
TMPStrategyMethod::
TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg){
  double x = _relativeCfg[0];
  double y = _relativeCfg[1];
  double theta = _centerCfg[2]*PI;

  double newX = x*cos(theta) - y*sin(theta);
  double newY = x*sin(theta) + y*cos(theta);
  double oldTheta = _relativeCfg[2];

  _relativeCfg.SetLinearPosition({newX, newY, oldTheta});

  _relativeCfg += _centerCfg;
}
void
TMPStrategyMethod::
ConnectDistinctRoadmaps(vector<size_t> _roadmap1, vector<size_t> _roadmap2,
    HandoffAgent* _agent, Robot* m_robot, MPLibrary* _library) {
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
      if(m_robot->IsManipulator()){
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

*/
