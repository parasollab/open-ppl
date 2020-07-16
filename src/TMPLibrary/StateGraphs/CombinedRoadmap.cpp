#include "CombinedRoadmap.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/StateGraphs/Helpers/ITConnector.h"
#include "TMPLibrary/StateGraphs/Helpers/ITConstructor.h"
#include "TMPLibrary/TaskPlan.h"

#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"

/*------------------------------ Construction --------------------------------*/
CombinedRoadmap::
CombinedRoadmap(){
	this->SetName("CombinedRoadmap");
};

CombinedRoadmap::
CombinedRoadmap(XMLNode& _node) : StateGraph(_node) {
	this->SetName("CombinedRoadmap");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric for checking "
      "nearest agents and charging locations.");
  m_connectionThreshold = _node.Read("connectionThreshold",true,1.2, 0., 1000.,
      "Acceptable variabliltiy in IT paths.");
	m_discrete = _node.Read("discrete", false, false, "Flag for creating a dsicrete grid world");

  for(auto& child : _node){
		// Load the environment file used to create ITs
		if(child.Name() == "InteractionEnvironment"){
			//if(!m_interactionEnvironment){
				m_interactionEnvironment = std::unique_ptr<Environment>(new Environment(child));
			//}
		}
	}
}

/*------------------------------ Construction --------------------------------*/

void
CombinedRoadmap::
Initialize(){

	ResetRobotTypeRoadmaps();

  //m_solution = std::unique_ptr<MPSolution>(new MPSolution(this->GetTaskPlan()->GetCoordinator()->GetRobot()));
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(this->GetPlan()->GetCoordinator()->GetRobot()));
	m_solution->SetRoadmap(this->GetPlan()->GetCoordinator()->GetRobot(),m_graph);

	//TODO Move these to helper classes
	if(m_discrete)
		GenerateDiscreteITs();
	else
		GenerateITs();

	StateGraph::Initialize();
	
	//if(m_debug) {
  //  Simulation::Get()->AddRoadmap(m_graph,
  //    glutils::color(0., 1., 0., 0.2));
	//}
}

/*------------------------------ Accessors --------------------------------*/

void
CombinedRoadmap::
LoadStateGraph(){
	StateGraph::LoadStateGraph();
	CopyRobotTypeRoadmaps();
}

/*------------------------------ Helpers --------------------------------*/


void
CombinedRoadmap::
ResetRobotTypeRoadmaps(){
  m_capabilityRoadmaps.clear();
  m_transformedRoadmaps.clear();;
}

void
CombinedRoadmap::
CopyRobotTypeRoadmaps(){
  for(auto agent : this->GetTaskPlan()->GetTeam()){
    //Copy corresponding capability roadmap into agent
    auto graph = m_capabilityRoadmaps[agent->GetCapability()];
    auto g = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(agent->GetRobot());
    *g = *graph;
    g->SetRobot(agent->GetRobot());
    agent->SetRoadmapGraph(g);

    // Write the capability map.
    graph->Write(agent->GetRobot()->GetLabel() + ".map",
        this->GetMPProblem()->GetEnvironment());
  }
}

/*------------------------------ Construction Helpers --------------------------------*/

void
CombinedRoadmap::
ConstructDiscreteRoadmap() {
	auto vcm = this->GetMPLibrary()->GetValidityChecker("terrain_solid");
	for(auto member : this->GetTaskPlan()->GetTeam()) {
		member->GetRobot()->SetVirtual(true);
	}

	TransformITs();//Should have manully specified locations for now

	//Setup Whole Tasks	
  for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
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
		
		auto startCfg = startPoints[0];
		int x = int(startCfg[0] + .5);
		int y = int(startCfg[1] + .5);
		startCfg.SetData({double(x),double(y),0});
		auto goalCfg = goalPoints[0];
		x = int(goalCfg[0] + .5);
		y = int(goalCfg[1] + .5);
		goalCfg.SetData({double(x),double(y),0});
    wholeTask->m_startPoints[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {startCfg};
    wholeTask->m_goalPoints[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {goalCfg};

		auto startVID = m_graph->AddVertex(startCfg);
		auto goalVID = m_graph->AddVertex(goalCfg);
	
		wholeTask->m_startVIDs[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {startVID};
		wholeTask->m_goalVIDs[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {goalVID};
	
    auto dummyMap = this->GetTaskPlan()->GetDummyMap();
    for(auto const& elem : dummyMap) {
			auto dummyStart = startCfg;
			dummyStart.SetRobot(elem.second->GetRobot());	
			auto dummyGoal = goalCfg;
			dummyGoal.SetRobot(elem.second->GetRobot());

			DefaultWeight<Cfg> weight;
			weight.SetWeight(0);
			if(vcm->IsValid(dummyStart, "ValidateStartCfg")) {
				wholeTask->m_startPoints[elem.first].push_back(dummyStart);
				auto dummyVID = m_graph->AddVertex(dummyStart);
				wholeTask->m_startVIDs[elem.first].push_back(dummyVID);

				m_graph->AddEdge(startVID,dummyVID,weight);
			}
			if(vcm->IsValid(dummyGoal, "ValidateStartCfg")) {
				wholeTask->m_goalPoints[elem.first].push_back(dummyGoal);
				auto dummyVID = m_graph->AddVertex(dummyGoal);
				wholeTask->m_goalVIDs[elem.first].push_back(dummyVID);

				m_graph->AddEdge(dummyVID,goalVID,weight);
			}
		}

	}
	//Construct Robot-type roadmaps
	auto envBoundary = this->GetPlan()->GetCoordinator()->GetRobot()->GetMPProblem()->GetEnvironment()->GetBoundary();
	auto xRange = envBoundary->GetRange(0);
	auto yRange = envBoundary->GetRange(1);



	for(auto& elem : this->GetTaskPlan()->GetDummyMap()) {
		int x = std::ceil(xRange.min);

		auto robot = elem.second->GetRobot();
		std::vector<std::vector<bool>> validMatrix;
		std::vector<std::vector<Cfg>> cfgMatrix;
		std::vector<std::vector<size_t>> vidMatrix;

		for(int i = 0; i < std::floor(xRange.Length()); i++) {
			cfgMatrix.push_back({});
			validMatrix.push_back({});
			vidMatrix.push_back({});
		}
		
		auto roadmap = std::shared_ptr<RoadmapGraph<Cfg,DefaultWeight<Cfg>>>(new RoadmapGraph<Cfg,DefaultWeight<Cfg>>(robot));

		while(x < xRange.max) {
			int y = std::ceil(yRange.min);
			while(y < yRange.max) {
				Cfg cfg(robot);
				cfg.SetData({double(x),double(y),0});
				
				cfgMatrix[x-std::ceil(xRange.min)].push_back(cfg);
				validMatrix[x-std::ceil(xRange.min)].push_back(vcm->IsValid(cfg, "Building robot-type roadmaps."));

				if(vcm->IsValid(cfg, "Building robot-type roadmaps.")){
					vidMatrix[x-std::ceil(xRange.min)].push_back(roadmap->AddVertex(cfg));
				}
				else {
					vidMatrix[x-std::ceil(xRange.min)].push_back(MAX_INT);
				}
				y += 1;	
			}
			x += 1;
		}

		if(m_debug) {
			for(size_t i = 0 ; i < cfgMatrix.size(); i++) {
				for(size_t j = 0 ; j < cfgMatrix[i].size(); j++) {
					std::cout << cfgMatrix[i][j].PrettyPrint() << " ";
				}
				std::cout << std::endl << std::endl;
			}
			for(size_t i = 0 ; i < cfgMatrix.size(); i++) {
				for(size_t j = 0 ; j < cfgMatrix[i].size(); j++) {
					std::cout << validMatrix[i][j] << "  ";
				}
				std::cout << std::endl << std::endl;
			}
			for(size_t i = 0 ; i < cfgMatrix.size(); i++) {
				for(size_t j = 0 ; j < cfgMatrix[i].size(); j++) {
					std::cout << vidMatrix[i][j] << "  ";
				}
				std::cout << std::endl << std::endl;
			}
		}

		DefaultWeight<Cfg> weight;
		weight.SetWeight(1);
		weight.SetTimeSteps(2);
		for(size_t i = 0 ; i < vidMatrix.size(); i++) {
			for(size_t j = 0 ; j < vidMatrix[i].size(); j++) {
				if(vidMatrix[i][j] == MAX_INT)
					continue;
				if(j < vidMatrix[i].size()-1 and vidMatrix[i][j+1] != MAX_INT) {//connect up

					Cfg source = roadmap->GetVertex(vidMatrix[i][j]);
					Cfg target = roadmap->GetVertex(vidMatrix[i][j+1]);
					Cfg middle = target;
					middle.SetData({(source[0]+target[0])/2, (source[1]+target[1])/2, (source[2]+target[2])/2});

					std::vector<Cfg> intermediates = {source,middle,target};
					weight.SetIntermediates(intermediates);

					roadmap->AddEdge(vidMatrix[i][j],vidMatrix[i][j+1],weight);

					intermediates = {target, middle, source};
					weight.SetIntermediates(intermediates);

					roadmap->AddEdge(vidMatrix[i][j+1],vidMatrix[i][j],weight);
				}
				
				if(i == 0)
					continue;

				//Connect left
				if(vidMatrix[i-1][j] != MAX_INT) {

					Cfg source = roadmap->GetVertex(vidMatrix[i][j]);
					Cfg target = roadmap->GetVertex(vidMatrix[i-1][j]);
					Cfg middle = target;
					middle.SetData({(source[0]+target[0])/2, (source[1]+target[1])/2, (source[2]+target[2])/2});

					std::vector<Cfg> intermediates = {source,middle,target};
					weight.SetIntermediates(intermediates);

					roadmap->AddEdge(vidMatrix[i][j],vidMatrix[i-1][j],weight);

					intermediates = {target, middle, source};
					weight.SetIntermediates(intermediates);

					roadmap->AddEdge(vidMatrix[i-1][j],vidMatrix[i][j],weight);
				}
			}
		}
		m_capabilityRoadmaps[elem.first] = roadmap;
		//Copy robot type roadmaps into combined roadmap
		std::unordered_map<size_t,size_t> oldToNew;
		for(auto vit = roadmap->begin(); vit != roadmap->end(); vit++) {
			oldToNew[vit->descriptor()] = m_graph->AddVertex(vit->property());
		}
		for(auto vit = roadmap->begin(); vit != roadmap->end(); vit++) {
			if(vit->descriptor() == 174 or vit->descriptor() == 97)
				std::cout << "Seems to be a problem here." << std::endl;
			for(auto eit = vit->begin(); eit != vit->end(); eit++) {
				if(eit->target() == 174 or eit->target() == 97)
					std::cout << "Seems to be a problem here." << std::endl;
				m_graph->AddEdge(oldToNew[eit->source()], oldToNew[eit->target()], weight);
			}
		}
	}
	

	for(auto member : this->GetTaskPlan()->GetTeam()) {
		member->GetRobot()->SetVirtual(false);
	}

}

void
CombinedRoadmap::
ConstructGraph(){
	if(m_discrete) {
		ConstructDiscreteRoadmap();
		return;
	}
  std::cout << "Creating combined roadmap and robot-type roadmaps." << std::endl;
  for(auto agent : this->GetTaskPlan()->GetTeam()){
    agent->GetRobot()->SetVirtual(true);
  }
  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = this->GetMPProblem();
  //this->GetMPLibrary()->InitializeMPProblem(originalProblem);
  this->GetMPLibrary()->SetMPProblem(originalProblem);

  for(auto& it : this->GetTaskPlan()->GetInteractionTemplates()){
    FindITLocations(it.get());
  }

  std::cout << "Found Handoff Locations" << std::endl;

  TransformITs();
  SetupWholeTasks();

  ITConnector connector(m_connectionThreshold,this->GetMPLibrary());
  //auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);

  if(true){//Adds robot starting locations to combined roadmap
    for(auto agent : this->GetTaskPlan()->GetTeam()){
      auto robot = agent->GetRobot();
      //auto cfg = robot->GetSimulationModel()->GetState();
      auto cfg = this->GetMPProblem()->GetInitialCfg(robot);
			cfg.SetRobot(this->GetTaskPlan()->GetCapabilityAgent(robot->GetCapability())->GetRobot());
			auto vid = m_graph->AddVertex(cfg);
			std::vector<size_t> vids = {vid};
			m_wholeTaskStartEndPoints.push_back(vids);
    }
  }

  std::vector<Cfg> startAndGoal;
  for(auto vid : m_wholeTaskStartEndPoints){
    startAndGoal.push_back(m_graph->GetVertex(vid[0]));
  }
	auto dummyMap = this->GetTaskPlan()->GetDummyMap();
  for(auto it = dummyMap.begin(); it != dummyMap.end(); it++){
    const std::string capability = it->first;
    auto graph = connector.ConnectInteractionTemplates(
                          this->GetTaskPlan()->GetInteractionTemplates(),
                          capability,
                          startAndGoal,
                          m_graph);

    auto robot = it->second->GetRobot();
    graph->Write("CoordinatorTemplates.map", robot->GetMPProblem()->GetEnvironment());

    std::priority_queue<std::pair<size_t,size_t>> pq;

    this->GetTaskPlan()->GetStatClass()->StartClock("Construction MegaRoadmap");
    // Copy over newly found vertices
    std::unordered_map<size_t,size_t> handoffVIDMap;
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      // Add vertices found in building the capability map to the mega roadmap
      // without copying over the handoff vertices that already exist
      if(handoffVIDMap.find(vit->descriptor()) == handoffVIDMap.end()){
        auto megaVID = m_graph->AddVertex(vit->property());
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
        if(!m_graph->IsEdge(source, target)){
          m_graph->AddEdge(source, target, eit->property());
        }
      }
    }

    m_graph->Write("MegaTemplates.map", robot->GetMPProblem()->GetEnvironment());
    this->GetTaskPlan()->GetStatClass()->StopClock("Construction MegaRoadmap");
  }
}

void 
CombinedRoadmap::
GenerateDiscreteITs() {

  auto originalProblem = this->GetMPProblem();
  this->GetMPLibrary()->SetMPProblem(originalProblem);

  for(auto& info : originalProblem->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    this->GetTaskPlan()->AddInteractionTemplate(it);
  }

	auto& dummyMap = this->GetTaskPlan()->GetDummyMap();

  for(auto& currentTemplate : this->GetTaskPlan()->GetInteractionTemplates()){
		//Assumes just a receiving and delivering robot 
  	auto handoffTasks = currentTemplate->GetInformation()->GetInteractionTasks();
		Cfg receiving(dummyMap[handoffTasks[0]->GetCapability()]->GetRobot());
		auto receivingRoadmap = new RoadmapGraph<Cfg,DefaultWeight<Cfg>>(handoffTasks[0]->GetRobot());
		receivingRoadmap->AddVertex(receiving);

    currentTemplate->AddRoadmap(receivingRoadmap);
    currentTemplate->AddPath({receiving}, originalProblem);
		currentTemplate->AddHandoffCfg(receiving, originalProblem);


		Cfg delivering(dummyMap[handoffTasks[1]->GetCapability()]->GetRobot());
		delivering.SetData({1,0,0});
		auto deliveringRoadmap = new RoadmapGraph<Cfg,DefaultWeight<Cfg>>(handoffTasks[1]->GetRobot());
		deliveringRoadmap->AddVertex(delivering);

    currentTemplate->AddRoadmap(deliveringRoadmap);
    currentTemplate->AddPath({delivering}, originalProblem);
		currentTemplate->AddHandoffCfg(delivering, originalProblem);


    currentTemplate->ConnectRoadmaps(this->GetPlan()->GetCoordinator()->GetRobot(), originalProblem);
	}

	
}

void
CombinedRoadmap::
GenerateITs(){
  auto originalProblem = this->GetMPProblem();
  this->GetMPLibrary()->SetMPProblem(originalProblem);

  for(auto& info : originalProblem->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    this->GetTaskPlan()->AddInteractionTemplate(it);
  }

  // Loop through handoff templates, set start constraints for handoff, set
  // dummy robot for handoff task by capability, and solve handoff task.
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*this->GetMPProblem()));
  problemCopy->SetEnvironment(std::move(m_interactionEnvironment));
  this->GetMPLibrary()->SetMPProblem(problemCopy.get());
  // Set robots to virtual so that planning handoffs does not cause collisions
  std::list<HandoffAgent*> unusedAgents;


  for(auto& currentTemplate : this->GetTaskPlan()->GetInteractionTemplates()){
    this->GetTaskPlan()->GetStatClass()->StartClock("Construct InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());

    unusedAgents.clear();
    std::copy(this->GetTaskPlan()->GetTeam().begin(), this->GetTaskPlan()->GetTeam().end(), 	
							std::back_inserter(unusedAgents));
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

          MPSolution* sol = new MPSolution(this->GetPlan()->GetCoordinator()->GetRobot());
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
        MPSolution* sol = new MPSolution(this->GetPlan()->GetCoordinator()->GetRobot());
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


      //handoffSolution->GetRoadmap()->Write("indHandoffTemplate" + std::to_string(check) + ".map", problemCopy->GetEnvironment());
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


    currentTemplate->ConnectRoadmaps(this->GetPlan()->GetCoordinator()->GetRobot(), originalProblem);

    this->GetTaskPlan()->GetStatClass()->StopClock("Construct InteractionTemplate "
                + currentTemplate->GetInformation()->GetLabel());
    this->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
                            +"::Vertices", currentTemplate->GetConnectedRoadmap()->get_num_vertices());
    this->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
                            +"::Edges", currentTemplate->GetConnectedRoadmap()->get_num_vertices());

    size_t count = 0;
    for(auto rm : currentTemplate->GetRoadmaps()){
      count++;
      this->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
          +"::"+std::to_string(count)
          +"::Vertices", rm->get_num_vertices());
      this->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
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
  this->GetMPLibrary()->SetTask(originalProblem->GetTasks(this->GetPlan()->GetCoordinator()->GetRobot())[0].get());
}

void
CombinedRoadmap::
FindITLocations(InteractionTemplate* _it){
  //for(auto& method : m_ITPlacementMethods){

		if(m_pmLabel == "")
			return;

		std::cout << "Calling " + m_pmLabel << std::endl;
    this->GetTaskPlan()->GetStatClass()->StartClock("Placing Templates with: " + m_pmLabel);
    auto method = this->GetPoIPlacementMethod(m_pmLabel);
		method->PlaceIT(_it, this->GetMPLibrary()->GetMPSolution());
    this->GetTaskPlan()->GetStatClass()->StopClock("Placing Templates with: " + m_pmLabel);
		std::cout << "Finished " + m_pmLabel << std::endl;
  //}
}

void
CombinedRoadmap::
TransformITs(){

	std::vector<size_t> invalidVIDs;

  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = this->GetMPProblem();
  this->GetMPLibrary()->SetMPProblem(originalProblem);

  std::cout << "Found Handoff Locations" << std::endl;
  this->GetTaskPlan()->GetStatClass()->StartClock("Construction MegaRoadmap");
  auto vcm = this->GetMPLibrary()->GetValidityChecker("terrain_solid");
  for(auto& currentTemplate : this->GetTaskPlan()->GetInteractionTemplates()){

    if(m_debug){
      auto g = currentTemplate->GetConnectedRoadmap();
      std::cout << "Original handoff position" << std::endl;
      for(auto vit = g->begin(); vit!=g->end(); vit++){
        std::cout << vit->property().PrettyPrint() << std::endl;
      }
    }

    //this->GetTaskPlan()->GetStatClass()->StartClock("Placement InteractionTemplate "
    //          + currentTemplate->GetInformation()->GetLabel());
    for(auto centerCfg : currentTemplate->GetInformation()->GetTemplateLocations()){
      this->GetTaskPlan()->GetStatClass()->StartClock("Placement InteractionTemplate "
                + currentTemplate->GetInformation()->GetLabel());

      RoadmapGraph<Cfg, DefaultWeight<Cfg>>* graph = currentTemplate->GetConnectedRoadmap();

			std::unordered_set<size_t> invalids;

      // Copy vertices and map the change in VIDs.
      std::unordered_map<VID, VID> oldToNew;
      for(auto vit = graph->begin(); vit != graph->end(); ++vit) {
        const VID oldVID = vit->descriptor();
        auto relativeCfg = vit->property();
        relativeCfg.TransformCfg(centerCfg.GetBaseTransformation());
				if(m_discrete){
					int x = int(relativeCfg[0] + .5);
					int y = int(relativeCfg[1] + .5);
					relativeCfg.SetData({double(x),double(y),0});
				}
        bool isValid = vcm->IsValid(relativeCfg, "ValidateITCfg");
				const VID newVID = m_graph->AddVertex(relativeCfg);
				oldToNew[oldVID] = newVID;
        if(!isValid){
        	invalidVIDs.push_back(newVID);
					invalids.insert(newVID);
				}
      }

      // Keep track of the distinct transformed handoff roadmaps
      for(auto distinctRoadmap : currentTemplate->GetDistinctRoadmaps()) {
        std::vector<size_t> transformedRoadmap;
        for(auto vid : distinctRoadmap) {
					auto newVID = oldToNew[vid];
					if(invalids.count(newVID))
						continue;
          //transformedRoadmap.push_back(oldToNew[vid]);
          transformedRoadmap.push_back(newVID);
        }
        m_transformedRoadmaps.push_back(transformedRoadmap);
      }


      if(m_debug){
        auto r = m_transformedRoadmaps.front();
        std::cout << "Transformed Postion of Roadmap" << std::endl;
        for(auto vit = m_graph->begin(); vit != m_graph->end(); vit++){
          std::cout << vit->descriptor() << std::endl;
          std::cout << vit->property().PrettyPrint() << std::endl;
        }
        for(auto v : r){
          std::cout << m_graph->GetVertex(v).PrettyPrint() << std::endl;
        }
      }

      m_graph->InstallHook(RoadmapType::HookType::AddEdge, "debug",
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
          if(!m_graph->IsEdge(source, target)){
            // Call translate cfg on the all the intermediates and built
            // up a new vector of intermediates to store in the edge property
            // before storing it in the megaRoadmap
            std::vector<Cfg> intermediates = eit->property().GetIntermediates();
            for(auto cfg : intermediates){
              cfg.TransformCfg(centerCfg.GetBaseTransformation());
            }
						//TODO validate the edge if its not an interaction edge
            m_graph->AddEdge(source, target, eit->property());
          }
        }
      }
      m_graph->RemoveHook(RoadmapType::HookType::AddEdge, "debug");
    	this->GetTaskPlan()->GetStatClass()->StopClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
    }
  }
 
	for(auto vid : invalidVIDs){
		m_graph->DeleteVertex(vid);
	} 
  this->GetTaskPlan()->GetStatClass()->StopClock("Construction MegaRoadmap");
}

void
CombinedRoadmap::
SetupWholeTasks(){
	this->GetMPLibrary()->SetMPSolution(m_solution.get());
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

    wholeTask->m_startPoints[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {startPoints[0]};
    wholeTask->m_goalPoints[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {goalPoints[0]};

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

    task->SetRobot(this->GetPlan()->GetCoordinator()->GetRobot());
    this->GetMPLibrary()->SetTask(task.get());

    // Create 0 weight edges between each capability and the coordinator
    // configuration.
    auto coordinatorStartVID = m_graph->AddVertex(
                    wholeTask->m_startPoints[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()][0]);

    wholeTask->m_startVIDs[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {coordinatorStartVID};

    auto coordinatorGoalVID = m_graph->AddVertex(wholeTask->m_goalPoints[
															this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()][0]);

    wholeTask->m_goalVIDs[this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel()] = {coordinatorGoalVID};

    const DefaultWeight<Cfg> weight;

    this->GetTaskPlan()->GetStatClass()->StartClock("Construction MegaRoadmap");
    for(auto const& elem : wholeTask->m_startPoints){
      if(elem.first == this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel())
        continue;

      for(auto start : elem.second) {
        auto agentStartVID = m_graph->AddVertex(start);

        // Add the start points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentStartVID});
        wholeTask->m_startVIDs[elem.first].push_back(agentStartVID);
        m_graph->AddEdge(coordinatorStartVID, agentStartVID, {weight,weight});
      }

    }

    for(auto const& elem : wholeTask->m_goalPoints){
      if(elem.first == this->GetPlan()->GetCoordinator()->GetRobot()->GetLabel())
        continue;

      for(auto goal : elem.second) {
        auto agentGoalVID = m_graph->AddVertex(goal);

        // Add the end points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentGoalVID});
        wholeTask->m_goalVIDs[elem.first].push_back(agentGoalVID);
        m_graph->AddEdge(coordinatorGoalVID, agentGoalVID, {weight,weight});
      }

    }
    this->GetTaskPlan()->GetStatClass()->StopClock("Construction MegaRoadmap");
  }
  this->GetMPLibrary()->SetTask(this->GetMPProblem()->GetTasks(
												this->GetPlan()->GetCoordinator()->GetRobot())[0].get());
}


std::shared_ptr<RoadmapGraph<Cfg,DefaultWeight<Cfg>>> 
CombinedRoadmap::
GetCapabilityRoadmap(HandoffAgent* _agent) {
	return m_capabilityRoadmaps[_agent->GetCapability()];
}





