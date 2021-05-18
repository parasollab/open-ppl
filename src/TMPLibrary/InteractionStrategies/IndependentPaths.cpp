#include "IndependentPaths.h"

/*---------------------------- Construction --------------------------*/

IndependentPaths::
IndependentPaths() {
  this->SetName("IndependentPaths");
}

IndependentPaths::
IndependentPaths(XMLNode& _node) : InteractionStrategyMethod(_node) {
  this->SetName("IndependentPaths");
  m_samplerLabel = _node.Read("samplerLabel", true, "", "Sampler for all Robots");
}

IndependentPaths::
~IndependentPaths() {}

/*------------------------------ Interface ----------------------------*/

  /*
void
IndependentPaths::
PlanInteraction(StateGraph* sg, std::shared_ptr<MPProblem> problemCopy) {

  cout << m_samplerLabel << endl;

  auto testSampler = sg->GetMPLibrary()->GetSampler(m_samplerLabel);
  testSampler->Print(std::cout);

  cout << "Done" << endl;

  auto originalProblem = this->GetMPProblem();

  for(auto& info : originalProblem->GetInteractionInformations()){
    //auto it = new InteractionTemplate(info.get());
    //this->GetPlan()->AddInteractionTemplate(it);
    //FindITLocations(it);
    auto inter = new Interaction(info.get());
    this->GetPlan()->AddInteraction(inter);
  }

  sg->GetMPLibrary()->SetMPProblem(problemCopy.get());
  // Set robots to virtual so that planning handoffs does not cause collisions
  std::list<HandoffAgent*> unusedAgents;

  for(auto& currentTemplate : this->GetPlan()->GetInteractions()){
    this->GetPlan()->GetStatClass()->StartClock("Construct InteractionTemplate "
        + currentTemplate->GetInformation()->GetLabel());

    unusedAgents.clear();
    std::copy(this->GetPlan()->GetTeam().begin(), this->GetPlan()->GetTeam().end(),
        std::back_inserter(unusedAgents));

    auto handoffTasks = currentTemplate->GetInformation()->GetInteractionTasks();
    std::unordered_map<std::shared_ptr<MPTask>, HandoffAgent*> agentTasks;
    // Loop through all tasks and assign a robot of matching capability to the
    // task, then configuring the robot at the goal constraint.

    for(auto task : handoffTasks){
      sg->GetMPLibrary()->SetTask(task.get());
      for(auto agent : unusedAgents){
        if(agent->GetCapability() == task->GetCapability()){
          agentTasks[task] = agent;
          Robot* tempRobot = problemCopy->GetRobot(agent->GetRobot()->GetLabel());
          task->SetRobot(tempRobot);
          unusedAgents.remove(agent);
          // Confiure tempRobot at the goal constraint for the task
          // - Sample at the point of the goal constraint
          // - Get CFG from sample and place tempRobot there
          auto boundingBox = task->GetGoalConstraints().front()->
            GetBoundary();
          std::vector<Cfg> goalPoints;

          MPSolution* sol = new MPSolution(this->GetPlan()->GetCoordinator()->GetRobot());
          sg->GetMPLibrary()->SetMPSolution(sol);
          sg->GetMPLibrary()->SetTask(task.get());
          auto sampler = sg->GetMPLibrary()->GetSampler(m_samplerLabel);
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
        sg->GetMPLibrary()->Solve(problemCopy.get(), task.get(), handoffSolution.get(),
            currentTemplate->GetInformation()->GetMPStrategy(), LRand(),
            currentTemplate->GetInformation()->GetMPStrategy());
      }
      // Solve for manipulator robot teams
      else {
        std::vector<Cfg> startPoints;
        MPSolution* sol = new MPSolution(this->GetPlan()->GetCoordinator()->GetRobot());
        sg->GetMPLibrary()->SetMPSolution(sol);
        auto sampler = sg->GetMPLibrary()->GetSampler(m_samplerLabel);
        size_t numNodes = 1, numAttempts = 100;
        auto boundingBox = task->GetStartConstraint()->GetBoundary();
        sg->GetMPLibrary()->SetTask(task.get());
        sampler->Sample(numNodes, numAttempts, boundingBox,std::back_inserter(startPoints));

        if(startPoints.empty())
          throw RunTimeException(WHERE, "No valid start handoff position for the robot.");

        startPoints[0].ConfigureRobot();
        std::cout << startPoints[0].PrettyPrint() << std::endl;

        sg->GetMPLibrary()->Solve(problemCopy.get(), task.get(),
            handoffSolution.get(), currentTemplate->GetInformation()->GetMPStrategy()
            , LRand(), currentTemplate->GetInformation()->GetMPStrategy());
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
      }else{
        // Add final configuration of path to template
        std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
        currentTemplate->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), originalProblem);
        std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().back() << std::endl;
      }
    }

    currentTemplate->ConnectRoadmaps(this->GetPlan()->GetCoordinator()->GetRobot(), originalProblem);

    this->GetPlan()->GetStatClass()->StopClock("Construct InteractionTemplate "
        + currentTemplate->GetInformation()->GetLabel());
    this->GetPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
        +"::Vertices", currentTemplate->GetConnectedRoadmap()->get_num_vertices());
    this->GetPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
        +"::Edges", currentTemplate->GetConnectedRoadmap()->get_num_vertices());

    size_t count = 0;
    for(auto rm : currentTemplate->GetRoadmaps()){
      count++;
      this->GetPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
          +"::"+std::to_string(count)
          +"::Vertices", rm->get_num_vertices());
      this->GetPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
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
}
  */
