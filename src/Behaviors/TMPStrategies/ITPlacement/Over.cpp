#include "OverlappingWorkspacesDensity.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"

OverlappingWorkspacesDensity::
OverlappingWorkspacesDensity(MPProblem* _problem) : PlacementMethod(_problem) {}


OverlappingWorkspacesDensity::
OverlappingWorkspacesDensity(MPProblem* _problem, XMLNode& _node) : PlacementMethod(_problem) {
  m_label = _node.Read("label", true, "", "label for a fixed base it placement method");
  m_proximity = _node.Read("proximity", true, nan(""), 0., 1000., "distance to check for proximity of other configurations");
  m_density = _node.Read("density", true, nan(""), 0., 1000., "number of cfgs required within proximity to place IT");
  m_dmLabel = _node.Read("dmLabel", true, "", "distance metric to compute proximity");
}

void
OverlappingWorkspacesDensity::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library){
  _solution->AddInteractionTemplate(_it);

  auto tasks = _it->GetInformation()->GetTasks();

  auto coordinator = static_cast<Coordinator*>(tasks[0]->GetRobot()->GetAgent());

  // Assuming two robots for now
  // Get the capability to determine the workspaces

  std::vector<HandoffAgent*> capabilityAgents;
  for(auto& task : tasks){
    auto agent = coordinator->GetCapabilityAgent(task->GetCapability());
    capabilityAgents.push_back(agent);
  }

  // Sample over the workspace and find density of configurations
  Robot* oldRobot = nullptr;
  if(_library->GetTask()){
    oldRobot = _library->GetTask()->GetRobot();
  }
  else {
    auto task = new MPTask(coordinator->GetRobot());
    _library->SetTask(task);
  }
  auto sampler = _library->GetSampler("UniformRandomFreeTerrain");
  auto numNodes = 1000, numAttempts = 100;
  for(auto agent : capabilityAgents){
    //TODO::probably need a check if it uses robots of the same capability
    std::vector<Cfg> samplePoints;
    _library->GetTask()->SetRobot(agent->GetRobot());
    sampler->Sample(numNodes, numAttempts, m_problem->GetEnvironment()->GetBoundary(),
                    std::back_inserter(samplePoints));
    // Save sampled points to the robot's roadmap
    std::cout << "Sample Points: " << std::endl;
    for(auto cfg: samplePoints){
      std::cout << cfg.PrettyPrint() << std::endl;
    }
    std::cout << "Finished Sample Points" << std::endl;
    for(auto cfg : samplePoints){
      agent->GetMPSolution()->GetRoadmap(agent->GetRobot())->GetGraph()->AddVertex(cfg);
    }
  }
  if(oldRobot){
    _library->GetTask()->SetRobot(oldRobot);
  }
  else{
    _library->SetTask(nullptr);
  }
  auto graph1 = capabilityAgents[0]->GetMPSolution()->GetRoadmap(capabilityAgents[0]->GetRobot())->GetGraph();
  auto graph2 = capabilityAgents[1]->GetMPSolution()->GetRoadmap(capabilityAgents[1]->GetRobot())->GetGraph();
  std::vector<Cfg> ITLocations;
  auto dm = _library->GetDistanceMetric(m_dmLabel);

  for(auto vit1 = graph1->begin(); vit1 != graph1->end(); ++vit1) {
    size_t count1 = 0;
    size_t count2 = 0;
    bool used = false;
    for(auto vit2 = graph1->begin(); vit2 != graph1->end(); ++vit2) {
      if(vit1 == vit2) continue;
        if(dm->Distance(vit1->property(), vit2->property()) < m_proximity){
          count1 += 1;
          auto it = std::find(ITLocations.begin(), ITLocations.end(), vit2->property());
          if(it != ITLocations.end()){
            used = true;
            //break;
          }
          else if(count1 >= m_density){
            //break;
          }
        }
    }
    if(used){
      continue;
    }
    for(auto vit2 = graph2->begin(); vit2 != graph2->end(); ++vit2) {
      if(vit1 == vit2) continue;
        std::cout << "Distance: " << dm->Distance(vit1->property(), vit2->property()) << std::endl;
        if(dm->Distance(vit1->property(), vit2->property()) < m_proximity){
          count2 += 1;
          if(count2 >= m_density){
            //break;
          }
        }
    }
    if(used){
      continue;
    }
    if(count1 >= m_density and count2 >= m_density){
      ITLocations.push_back(vit1->property());
    }
  }
  std::cout << "Number of ITLocations: " << ITLocations.size() << std::endl;
  for(auto cfg : ITLocations){
    _it->GetInformation()->AddTemplateLocation(cfg);
  }
}
