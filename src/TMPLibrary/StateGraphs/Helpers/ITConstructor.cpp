#include "ITConstructor.h"

#include "MPProblem/Environment/Environment.h"
#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"

ITConstructor::
ITConstructor(MPLibrary* _library,
              std::vector<HandoffAgent*> _memberAgents,
              Robot* _superRobot,
							TaskPlan* _taskPlan){
  m_library = _library;
  m_problem = _library->GetMPProblem();
  m_superRobot = _superRobot;
  m_memberAgents = _memberAgents;
  m_problemCopy = std::shared_ptr<MPProblem>(m_problem);
	m_taskPlan = _taskPlan;
}



void
ITConstructor::
ConstructIT(InteractionTemplate* _it){

  auto interactionEnvironment = std::unique_ptr<Environment>(
                                _it->GetInformation()->GetInteractionEnvironment());
  m_problemCopy->SetEnvironment(std::move(interactionEnvironment));

  std::list<HandoffAgent*> unusedAgents;
  std::copy(m_memberAgents.begin(), m_memberAgents.end(), std::back_inserter(unusedAgents));
  auto handoffTasks = _it->GetInformation()->GetInteractionTasks();
  std::unordered_map<std::shared_ptr<MPTask>, HandoffAgent*> agentTasks;
  // Loop through all tasks and assign a robot of matching capability to the
  // task, then configuring the robot at the goal constraint.
  for(auto task : handoffTasks){
    m_library->SetTask(task.get());
    for(auto agent : unusedAgents){
      if(agent->GetCapability() == task->GetCapability()){
        agentTasks[task] = agent;
        Robot* tempRobot = m_problemCopy->GetRobot(agent->GetRobot()->GetLabel());
        task->SetRobot(tempRobot);
        unusedAgents.remove(agent);
        // Confiure tempRobot at the goal constraint for the task
        //        - Sample at the point of the goal constraint
        //        - Get CFG from sample and place tempRobot there
        auto boundingBox = task->GetGoalConstraints().front()->
          GetBoundary();
        std::vector<Cfg> goalPoints;

        MPSolution* sol = new MPSolution(m_superRobot);
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
    auto robot = m_problemCopy->GetRobot(agent->GetRobot()->GetLabel());
    robot->SetVirtual(true);
  }
  int check = 0;
  for(auto task : handoffTasks){
    Robot* taskRobot = m_problemCopy->GetRobot(agentTasks[task]->GetRobot()->GetLabel());
    std::unique_ptr<MPSolution> handoffSolution(new MPSolution(taskRobot));
    // Store the current configuration of the robot, since the multibody
    // will be moved while solving.
    auto currentConfig = taskRobot->GetMultiBody()->GetCurrentDOFs();
    if(m_debug){
      for(auto& robot : m_problemCopy->GetRobots()){
        std::cout << robot->GetLabel() << " - " << robot.get()
          << ": " << robot->GetMultiBody()->GetCurrentDOFs()
          << " - " << robot->IsVirtual() << std::endl;
      }
    }

    task->SetRobot(taskRobot);
    // Solve for non-mainpulator robot teams
    if(!taskRobot->IsManipulator()){
      m_library->Solve(m_problemCopy.get(), task.get(), handoffSolution.get());
    }
    // Solve for manipulator robot teams
    else {
      std::vector<Cfg> startPoints;
      MPSolution* sol = new MPSolution(m_superRobot);
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

      m_library->Solve(m_problemCopy.get(), task.get(),
          handoffSolution.get(), "FixedPRM", LRand(), "FixedPRM");
    }

    taskRobot->GetMultiBody()->Configure(currentConfig);

    if(true){
      std::cout << "Size of path: " << handoffSolution->GetPath()->Cfgs().size() << std::endl;
      for(auto cfg : handoffSolution->GetPath()->Cfgs()){
        std::cout << cfg.PrettyPrint() << std::endl;
      }
    }


    handoffSolution->GetRoadmap()->Write("indHandoffTemplate" + std::to_string(check)
                                  + ".map", m_problemCopy->GetEnvironment());
    check++;

    // Store the roadmap for each task in the handoff
    _it->AddRoadmap(handoffSolution->GetRoadmap());
    _it->AddPath(handoffSolution->GetPath()->Cfgs(), m_problem);

    if(_it->GetInformation()->SavedPaths()){
      //_it->AddPath(handoffSolution->GetPath()->Cfgs(),
      //    handoffSolution->GetPath()->Length());
      std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
      _it->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().front(), m_problem);
      std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().front() << std::endl;
    }
    else{
      // Add final configuration of path to template
      std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
      _it->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), m_problem);
      std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().back() << std::endl;
    }
  }


  _it->ConnectRoadmaps(m_superRobot, m_problem);

  m_taskPlan->GetStatClass()->StopClock("Construct InteractionTemplate "
      + _it->GetInformation()->GetLabel());

  std::cout << "Trying to write handoffTemplate Map" << std::endl;
  _it->GetConnectedRoadmap()->Write("handoffTemplate.map", m_problemCopy->GetEnvironment());

  // Reset the agents to non-virtual, since they could be used in the next
  // template.
  for(auto agent : unusedAgents){
    m_problem->GetRobot(agent->GetRobot()->GetLabel())->SetVirtual(false);
  }
}
