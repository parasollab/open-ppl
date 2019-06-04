#include "TMPStrategyMethod.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/TaskPlan.h"

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
	m_placementMethod = _node.Read("poiPlacementMethod", false, "", 
			"Methods for placing points of interest in the environment");

  for(auto& child : _node){
		// Load the environment file used to create ITs
		if(child.Name() == "InteractionEnvironment"){
			if(!m_interactionEnvironment){
				m_interactionEnvironment = std::unique_ptr<Environment>(new Environment(child));
			}
		}
		//TODO::Figure out if this is going to be extracted or how to find a problem pointer
		//TODO::Temp fix may be to figure out where the problem pointe is needed and delay that functionality
		//else if(child.Name() == "PlacementMethod"){
		//	AddPlacementMethod(PlacementMethod::Factory(m_problem(), child));
		//}
	}

}
/*
TMPStrategyMethod::
TMPStrategyMethod(bool _useITs, bool _debug, std::string _dmLabel, double _connectionThreshold,
									Environment* _interactionEnvironment, 
									std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>& _ITPlacementMethods) :
								  m_useITs(_useITs), 
									m_debug(_debug),
									m_dmLabel(_dmLabel), 
									m_connectionThreshold(_connectionThreshold){

	m_interactionEnvironment = std::unique_ptr<Environment>(_interactionEnvironment);
  for(auto& pm : _ITPlacementMethods){
		m_ITPlacementMethods[pm.first] = pm.second->Clone();
	}

}*/

TMPStrategyMethod::
~TMPStrategyMethod(){
	delete m_combinedRoadmap;
}

/******************************************Configure*****************************************************/

void
TMPStrategyMethod::
Initialize(){

}

void 
TMPStrategyMethod::
Initialize(Robot* _robot){
  m_initialized = true;

  m_robot = _robot;

  this->GetTaskPlan()->GetTeam().clear();

	//TODO::load team into task plan

  this->GetTaskPlan()->GenerateDummyAgents(); 
  
  ResetCapabilityRoadmaps();

  //m_wholeTasks.clear();
  //m_wholeTaskStartEndPoints.clear();

  if(m_combinedRoadmap){
		delete m_combinedRoadmap;
  }
	m_combinedRoadmap = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(m_robot);

  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));
	m_solution->SetRoadmap(m_robot,m_combinedRoadmap);

}

void
TMPStrategyMethod::
ResetCapabilityRoadmaps(){
  m_capabilityRoadmaps.clear();
  m_transformedRoadmaps.clear();;
}

/****************************************Call Method*****************************************************/

TaskPlan*
TMPStrategyMethod::
PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
          vector<std::shared_ptr<MPTask>> _tasks){
  if(!m_initialized){
	throw RunTimeException(WHERE, "TMPStrategyMethod not initialized.");
  }

	this->GetMPLibrary()->SetMPSolution(m_solution.get());

  this->GetTaskPlan()->GetTeam() = _agents;

  this->GetTaskPlan()->CreateWholeTasks(_tasks);

  return new TaskPlan();
}

void
TMPStrategyMethod::
operator()(){
}

/*****************************************Accessors******************************************************/

Robot* 
TMPStrategyMethod::
GetRobot(){
	return m_robot;
}

/*
HandoffAgent*
TMPStrategyMethod::
GetCapabilityAgent(std::string _robotType){
	return m_dummyMap[_robotType];
}
WholeTask*
TMPStrategyMethod::
GetWholeTask(std::shared_ptr<MPTask> _subtask){
	return m_subtaskMap[_subtask];
}
*/
/**************************************Combined Roadmap**************************************************/

void
TMPStrategyMethod::
CreateCombinedRoadmap(){
  for(auto agent : this->GetTaskPlan()->GetTeam()){
    agent->GetRobot()->SetVirtual(true);
  }
  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  //this->GetMPLibrary()->InitializeMPProblem(originalProblem);
  this->GetMPLibrary()->SetMPProblem(originalProblem);

  for(auto& it : m_solution->GetInteractionTemplates()){
    //m_solution->AddInteractionTemplate(it);
    FindITLocations(it.get());
  }

  std::cout << "Found Handoff Locations" << std::endl;

  TransformITs();
  SetupWholeTasks();

  CombinedRoadmap connector(m_connectionThreshold,this->GetMPLibrary());
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);

  if(true){
    for(auto agent : this->GetTaskPlan()->GetTeam()){
      auto robot = agent->GetRobot();
      auto cfg = robot->GetSimulationModel()->GetState();
      auto vid = m_combinedRoadmap->AddVertex(cfg);
	  std::vector<size_t> vids = {vid};
      m_wholeTaskStartEndPoints.push_back(vids);
    }
  }

  std::vector<Cfg> startAndGoal;
  for(auto vid : m_wholeTaskStartEndPoints){
    startAndGoal.push_back(m_combinedRoadmap->GetVertex(vid[0]));
  }
	auto dummyMap = this->GetTaskPlan()->GetDummyMap();
  for(auto it = dummyMap.begin(); it != dummyMap.end(); it++){
    const std::string capability = it->first;
    auto graph = connector.ConnectInteractionTemplates(
                          m_solution->GetInteractionTemplates(),
                          capability,
                          startAndGoal,
                          m_combinedRoadmap);

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
        auto megaVID = m_combinedRoadmap->AddVertex(vit->property());
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
        if(!m_combinedRoadmap->IsEdge(source, target)){
          m_combinedRoadmap->AddEdge(source, target, eit->property());
        }
      }
    }

    m_combinedRoadmap->Write("MegaTemplates.map", robot->GetMPProblem()->GetEnvironment());
    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
  }
}

void
TMPStrategyMethod::
GenerateITs(){
  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  this->GetMPLibrary()->SetMPProblem(originalProblem);

  for(auto& info : originalProblem->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    m_solution->AddInteractionTemplate(it);
    //FindITLocations(it);
  }

  std::cout << "Found Handoff Locations" << std::endl;

  // Loop through handoff templates, set start constraints for handoff, set
  // dummy robot for handoff task by capability, and solve handoff task.
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  problemCopy->SetEnvironment(std::move(m_interactionEnvironment));
  this->GetMPLibrary()->SetMPProblem(problemCopy.get());
  // Set robots to virtual so that planning handoffs does not cause collisions
  std::list<HandoffAgent*> unusedAgents;


  for(auto& currentTemplate : m_solution->GetInteractionTemplates()){
    Simulation::GetStatClass()->StartClock("Construct InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());

    unusedAgents.clear();
    std::copy(this->GetTaskPlan()->GetTeam().begin(), this->GetTaskPlan()->GetTeam().end(), std::back_inserter(unusedAgents));
    auto handoffTasks = currentTemplate->GetInformation()->GetInteractionTasks();
    std::unordered_map<std::shared_ptr<MPTask>, HandoffAgent*> agentTasks;
    // Loop through all tasks and assign a robot of matching capability to the
    // task, then configuring the robot at the goal constraint.
    for(auto task : handoffTasks){
      this->GetMPLibrary()->SetTask(task.get());
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
          this->GetMPLibrary()->SetMPSolution(sol);
          this->GetMPLibrary()->SetTask(task.get());
          auto sampler = this->GetMPLibrary()->GetSampler("UniformRandomFree");
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
        this->GetMPLibrary()->Solve(problemCopy.get(), task.get(), handoffSolution.get());
      }
      // Solve for manipulator robot teams
      else {
        std::vector<Cfg> startPoints;
        MPSolution* sol = new MPSolution(m_robot);
        this->GetMPLibrary()->SetMPSolution(sol);
        auto sampler = this->GetMPLibrary()->GetSampler("UniformRandomFree");
        size_t numNodes = 1, numAttempts = 100;
        auto boundingBox = task->GetStartConstraint()->GetBoundary();
        this->GetMPLibrary()->SetTask(task.get());
        sampler->Sample(numNodes, numAttempts, boundingBox,
            std::back_inserter(startPoints));

        if(startPoints.empty())
          throw RunTimeException(WHERE, "No valid start handoff position for the robot.");

        startPoints[0].ConfigureRobot();
        std::cout << startPoints[0].PrettyPrint() << std::endl;

        this->GetMPLibrary()->Solve(problemCopy.get(), task.get(),
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


  this->GetMPLibrary()->SetMPProblem(originalProblem);
  this->GetMPLibrary()->SetMPSolution(m_solution.get());
  this->GetMPLibrary()->SetTask(originalProblem->GetTasks(m_robot)[0].get());
}

void
TMPStrategyMethod::
FindITLocations(InteractionTemplate* _it){
  //for(auto& method : m_ITPlacementMethods){
		std::cout << "Calling " + m_placementMethod << std::endl;
    Simulation::GetStatClass()->StartClock("Placing Templates with: " + m_placementMethod);
    auto method = this->GetPoIPlacementMethod(m_placementMethod);
		method->PlaceIT(_it, this->GetMPLibrary()->GetMPSolution());
    Simulation::GetStatClass()->StopClock("Placing Templates with: " + m_placementMethod);
		std::cout << "Finished " + m_placementMethod << std::endl;
  //}
}


void
TMPStrategyMethod::
TransformITs(){

  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  this->GetMPLibrary()->SetMPProblem(originalProblem);

  std::cout << "Found Handoff Locations" << std::endl;
  Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
  auto vcm = this->GetMPLibrary()->GetValidityChecker("terrain_solid");
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
          const VID newVID = m_combinedRoadmap->AddVertex(relativeCfg);
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
        for(auto vit = m_combinedRoadmap->begin(); vit != m_combinedRoadmap->end(); vit++){
          std::cout << vit->descriptor() << std::endl;
          std::cout << vit->property().PrettyPrint() << std::endl;
        }
        for(auto v : r){
          std::cout << m_combinedRoadmap->GetVertex(v).PrettyPrint() << std::endl;
        }
      }

      m_combinedRoadmap->InstallHook(RoadmapType::HookType::AddEdge, "debug",
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
          if(!m_combinedRoadmap->IsEdge(source, target)){
            // Call translate cfg on the all the intermediates and built
            // up a new vector of intermediates to store in the edge property
            // before storing it in the megaRoadmap
            std::vector<Cfg> intermediates = eit->property().GetIntermediates();
            for(auto cfg : intermediates){
              cfg.TransformCfg(centerCfg);
            }
            m_combinedRoadmap->AddEdge(source, target, eit->property());
          }
        }
      }
      m_combinedRoadmap->RemoveHook(RoadmapType::HookType::AddEdge, "debug");
    }
    Simulation::GetStatClass()->StopClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
  }
  
  Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
}

void
TMPStrategyMethod::
SetupWholeTasks(){
  for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
    // find a start and goal configuration for the coordinator
    auto task = wholeTask->m_task;
	this->GetMPLibrary()->SetTask(task.get());
    auto startBox = task->GetStartConstraint()->GetBoundary();
    std::vector<Cfg> startPoints;
    auto sampler = this->GetMPLibrary()->GetSampler("UniformRandomFree");
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
    auto dummyMap = this->GetTaskPlan()->GetDummyMap();
    for(auto const& elem : dummyMap) {
      // Set library robot to the corresponding capability
      task->SetRobot(elem.second->GetRobot());
      this->GetMPLibrary()->SetTask(task.get());
      // Sample to find valid start and goal points in the environment
      auto startBox = task->GetStartConstraint()->GetBoundary();
      std::vector<Cfg> startPoints;
      auto sampler = this->GetMPLibrary()->GetSampler("UniformRandomFreeTerrain");
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
    this->GetMPLibrary()->SetTask(task.get());

    // Create 0 weight edges between each capability and the coordinator
    // configuration.
    auto coordinatorStartVID = m_combinedRoadmap->AddVertex(
                    wholeTask->m_startPoints[m_robot->GetLabel()][0]);

    wholeTask->m_startVIDs[m_robot->GetLabel()] = {coordinatorStartVID};

    auto coordinatorGoalVID = m_combinedRoadmap->AddVertex(wholeTask->m_goalPoints[m_robot->GetLabel()][0]);

    wholeTask->m_goalVIDs[m_robot->GetLabel()] = {coordinatorGoalVID};

    const DefaultWeight<Cfg> weight;

    Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
    for(auto const& elem : wholeTask->m_startPoints){
      if(elem.first == m_robot->GetLabel())
        continue;

      for(auto start : elem.second) {
        auto agentStartVID = m_combinedRoadmap->AddVertex(start);

        // Add the start points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentStartVID});
        wholeTask->m_startVIDs[elem.first].push_back(agentStartVID);
        m_combinedRoadmap->AddEdge(coordinatorStartVID, agentStartVID, {weight,weight});
      }

    }

    for(auto const& elem : wholeTask->m_goalPoints){
      if(elem.first == m_robot->GetLabel())
        continue;

      for(auto goal : elem.second) {
        auto agentGoalVID = m_combinedRoadmap->AddVertex(goal);

        // Add the end points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentGoalVID});
        wholeTask->m_goalVIDs[elem.first].push_back(agentGoalVID);
        m_combinedRoadmap->AddEdge(coordinatorGoalVID, agentGoalVID, {weight,weight});
      }

    }
    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
  }
  this->GetMPLibrary()->SetTask(m_robot->GetMPProblem()->GetTasks(m_robot)[0].get());
}

/**************************************Combined Roadmap**************************************************/

TaskPlan*
TMPStrategyMethod::
AssignTasks(){
	return new TaskPlan();
}

void
TMPStrategyMethod::
DecomposeTasks(){}

/**************************************Helper Methods**************************************************/
/*
void
TMPStrategyMethod::
GenerateDummyAgents(){
  m_dummyMap.clear();
  // Load the dummyMap, which stores a dummy agent for each agent capability.
  for(auto agent : this->GetTaskPlan()->GetTeam()){
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
void
TMPStrategyMethod::
AddPlacementMethod(std::unique_ptr<PlacementMethod> _pm){
	m_ITPlacementMethods[_pm->GetLabel()] = std::move(_pm);
}
*/

