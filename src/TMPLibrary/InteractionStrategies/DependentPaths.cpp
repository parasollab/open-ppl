#include "DependentPaths.h"

#include "MPProblem/InteractionInformation.h"
#include "TMPLibrary/TMPTools/InteractionTemplate.h"

/*---------------------------- Construction --------------------------*/

DependentPaths::
DependentPaths() {
  this->SetName("DependentPaths");
}

DependentPaths::
DependentPaths(XMLNode& _node) : InteractionStrategyMethod(_node) {
  this->SetName("DependentPaths");
  m_firstSamplerLabel = _node.Read("firstSamplerLabel", true, "", "Sampler for first robot");
  m_restSamplerLabel = _node.Read("restSamplerLabel", true, "", "Sampler for all but first robot");
}

DependentPaths::
~DependentPaths() {}

/*------------------------------ Interface ----------------------------*/

void
DependentPaths::
PlanInteraction(StateGraph* sg, std::shared_ptr<MPProblem> problemCopy){

  cout << "First Sampler: " << m_firstSamplerLabel << endl;
  auto firstSampler = sg->GetMPLibrary()->GetSampler(m_firstSamplerLabel);
  firstSampler->Print(std::cout);

  cout << "All Others Sampler: " << m_restSamplerLabel << endl;
  auto restSampler = sg->GetMPLibrary()->GetSampler(m_restSamplerLabel);
  restSampler->Print(std::cout);

  auto originalProblem = this->GetMPProblem();
  for(auto& info : originalProblem->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    sg->GetTaskPlan()->AddInteractionTemplate(it);
    //FindITLocations(it);
  }

  sg->GetMPLibrary()->SetMPProblem(problemCopy.get());
  // Set robots to virtual so that planning handoffs does not cause collisions

  std::list<HandoffAgent*> unusedAgents;
  for(auto& currentTemplate : sg->GetTaskPlan()->GetInteractionTemplates()){
    sg->GetTaskPlan()->GetStatClass()->StartClock("Construct InteractionTemplate "+ currentTemplate->GetInformation()->GetLabel());

    unusedAgents.clear();
    std::copy(sg->GetTaskPlan()->GetTeam().begin(), sg->GetTaskPlan()->GetTeam().end(),std::back_inserter(unusedAgents));

    auto handoffTasks = currentTemplate->GetInformation()->GetInteractionTasks();
    std::unordered_map<std::shared_ptr<MPTask>, HandoffAgent*> agentTasks;

    // Loop through all tasks and assign a robot of matching capability to the
    // task, then configuring the robot at the goal constraint.

    double endEffectorBoundaryRadius = 0.5; //TODO: automatically compute or make input parameter
    //endEffectorBoundaryRadius = tempRobot->GetMultiBody()->GetBody(effBodyIndex)->GetWorldPolyhedron().GetMaxRadius();

    bool firstTime = true;
    vector<double> eeVector;
    for(auto task : handoffTasks){
      sg->GetMPLibrary()->SetTask(task.get());
      for(auto agent : unusedAgents){
        if(agent->GetCapability() == task->GetCapability()) {
          agentTasks[task] = agent;
          Robot* tempRobot = problemCopy->GetRobot(agent->GetRobot()->GetLabel());
          task->SetRobot(tempRobot);
          unusedAgents.remove(agent);
          // Confiure tempRobot at the goal constraint for the task
          // - Sample at the point of the goal constraint
          // - Get CFG from sample and place tempRobot there
          std::vector<Cfg> goalPoints;

          MPSolution* sol = new MPSolution(sg->GetTaskPlan()->GetCoordinator()->GetRobot());
          sg->GetMPLibrary()->SetMPSolution(sol);
          sg->GetMPLibrary()->SetTask(task.get());

          auto taskBoundary = task->GetGoalConstraints().front()->GetBoundary();
          cout << "Here is taskBoundary" << endl;
          cout << *taskBoundary << endl;

          tempRobot->SetVirtual(true);

          string samplerLabel = m_restSamplerLabel;
          if(firstTime)
            samplerLabel = m_firstSamplerLabel;
          auto sampler = sg->GetMPLibrary()->GetSampler(samplerLabel);
          cout << "Attempting a robot goal with this sampler: ";
          sampler->Print(std::cout);
          size_t numNodes = 1, numAttempts = 100;
          if(firstTime) {
            cout << "First time running sampler with no constraints" << endl;
            firstTime = false;
            bool keepSampling = true;
            while(keepSampling){
              goalPoints.clear();
              //sample first robot, no end effector constraints
            sampler->Sample(numNodes, numAttempts, taskBoundary,std::back_inserter(goalPoints));

	    //compute end effector placement (for future sampling constraint)
            if(goalPoints.empty())
              throw RunTimeException(WHERE, "No valid final handoff position for the robot.");
            goalPoints[0].ConfigureRobot();
            cout << "Robot Cfg: " << goalPoints[0].PrettyPrint() << endl;
	    auto it = tempRobot->GetWorldContactPoint().begin();
            eeVector.clear();
            for(size_t i=0; i<3; ++i) {
              eeVector.push_back(*it);
              it++;
            }
	          cout << "Grip Point: "; std::copy(eeVector.begin(), eeVector.end(), std::ostream_iterator<double>(cout, " ")); cout << endl;

                double diffX = pow(eeVector[0],2);
                double diffY = pow(eeVector[1]-4,2);
                double diffZ = pow(eeVector[2],2);
                if(sqrt(diffX + diffY + diffZ) <= 14){
                  keepSampling = false;
                }
              }
          }else{
            cout << "Subsequent time running sampler with ee constraints" << endl;

	    cout << "Grip Point: "; std::copy(eeVector.begin(), eeVector.end(), std::ostream_iterator<double>(cout, " ")); cout << endl;
            WorkspaceBoundingSphere endEffectorBoundary(eeVector, endEffectorBoundaryRadius);
            cout << "End Effector Boundary: " << endEffectorBoundary << endl;

            WorkspaceBoundingBox baseBoundary(3);
            for(size_t i = 0; i<3; ++i) {
              auto range = taskBoundary->GetRange(i);
              baseBoundary.SetRange(i, range.min, range.max);
            }
            cout << "Base Boundary" << endl;
            cout << baseBoundary << endl;
            sampler->SetBaseBoundary(&baseBoundary);

            auto envBoundary = sg->GetMPProblem()->GetEnvironment()->GetBoundary();
            cout << "Env Boundary" << endl;
            cout << *envBoundary << endl;
            int numSamplerRuns = 0;
            vector<double> allEEDist;
            while((numSamplerRuns < 1000) && (goalPoints.empty())){
              vector<double> secondEEVector;
              vector<Cfg> tempGoalPoints;
              if(samplerLabel == "RV"){
                sampler->Sample(numNodes, numAttempts, envBoundary, &endEffectorBoundary,std::back_inserter(tempGoalPoints));
              }else{
                sampler->Sample(numNodes, numAttempts, taskBoundary,std::back_inserter(tempGoalPoints));
              }
              for(auto it = tempGoalPoints.begin(); it != tempGoalPoints.end(); it++){
                it->ConfigureRobot();
	        auto it2 = tempRobot->GetWorldContactPoint().begin();
                for(size_t i=0; i<3; ++i) {
                  secondEEVector.push_back(*it2);
                  it2++;
                }
                double diffX = pow(eeVector[0]-secondEEVector[0],2);
                double diffY = pow(eeVector[1]-secondEEVector[1],2);
                double diffZ = pow(eeVector[2]-secondEEVector[2],2);
                allEEDist.push_back(sqrt(diffX + diffY + diffZ));
                cout << "Distance between end effectors: " << allEEDist.back() << endl;
                if(allEEDist.back() <= 1){
                  goalPoints.push_back(*it);
                }
              }
              if(goalPoints.empty()){
                numSamplerRuns++;
              }

            }
          cout << "Number of tries of sampler: " << numSamplerRuns << endl;
          cout << "Min distance between end effectors: " << *(min_element(allEEDist.begin(), allEEDist.end())) << endl;
          cout << "Max distance between end effectors: " << *(max_element(allEEDist.begin(), allEEDist.end())) << endl;
          cout << "Average distance between end effectors: " << accumulate(allEEDist.begin(), allEEDist.end(), 0.0) / allEEDist.size() << endl;
          }
          tempRobot->SetVirtual(false);

          if(goalPoints.empty())
            throw RunTimeException(WHERE, "No valid final handoff position for the robot.");
          goalPoints[0].ConfigureRobot();
          cout << "Robot Cfg: " << goalPoints[0].PrettyPrint() << endl;
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
    firstTime = true;
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

      string samplerLabel = m_firstSamplerLabel;
      if(firstTime)
        samplerLabel = m_firstSamplerLabel;

      cout << "Sampling startPoints" << endl;
      std::vector<Cfg> startPoints;

      MPSolution* sol = new MPSolution(sg->GetTaskPlan()->GetCoordinator()->GetRobot());
      sg->GetMPLibrary()->SetMPSolution(sol);

      auto sampler = sg->GetMPLibrary()->GetSampler(samplerLabel);
      size_t numNodes = 1, numAttempts = 100;

      auto taskBoundary = task->GetStartConstraint()->GetBoundary();

      sg->GetMPLibrary()->SetTask(task.get());

      if(firstTime) {
        firstTime = false;
        cout << "First Time running sampler with no constraints for start" << endl;
        sampler->Sample(numNodes, numAttempts, taskBoundary,std::back_inserter(startPoints));
      }else{
        //Need to update to pass in end effector constraint
        cout << "Running sampler with end effector constraints for start" << endl;
	sampler->Sample(numNodes, numAttempts, taskBoundary,std::back_inserter(startPoints));
      }
      if(startPoints.empty())
        throw RunTimeException(WHERE, "No valid start handoff position for the robot.");

      startPoints[0].ConfigureRobot();
      std::cout << startPoints[0].PrettyPrint() << std::endl;

      sg->GetMPLibrary()->Solve(problemCopy.get(), task.get(),
        handoffSolution.get(), currentTemplate->GetInformation()->GetMPStrategy()
            , LRand(), currentTemplate->GetInformation()->GetMPStrategy());

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

    currentTemplate->ConnectRoadmaps(sg->GetTaskPlan()->GetCoordinator()->GetRobot(), originalProblem);

    sg->GetTaskPlan()->GetStatClass()->StopClock("Construct InteractionTemplate "
        + currentTemplate->GetInformation()->GetLabel());
    sg->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
        +"::Vertices", currentTemplate->GetConnectedRoadmap()->get_num_vertices());
    sg->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
        +"::Edges", currentTemplate->GetConnectedRoadmap()->get_num_vertices());

    size_t count = 0;
    for(auto rm : currentTemplate->GetRoadmaps()){
      count++;
      sg->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
          +"::"+std::to_string(count)
          +"::Vertices", rm->get_num_vertices());
      sg->GetTaskPlan()->GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
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
