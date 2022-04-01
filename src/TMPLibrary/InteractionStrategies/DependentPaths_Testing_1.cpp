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
    sg->GetTaskPlan()->GetStatClass()->StartClock("Construct Interaction Template "+ currentTemplate->GetInformation()->GetLabel());

    unusedAgents.clear();
    std::copy(sg->GetTaskPlan()->GetTeam().begin(), sg->GetTaskPlan()->GetTeam().end(),std::back_inserter(unusedAgents));

    auto handoffTasks = currentTemplate->GetInformation()->GetInteractionTasks();
    std::unordered_map<std::shared_ptr<MPTask>, HandoffAgent*> agentTasks;

    // Loop through all tasks and assign a robot of matching capability to the
    // task, then configuring the robot at the goal constraint.

    //double endEffectorBoundaryRadius = 0.5; //TODO: automatically compute or make input parameter
    //endEffectorBoundaryRadius = tempRobot->GetMultiBody()->GetBody(effBodyIndex)->GetWorldPolyhedron().GetMaxRadius();

    bool firstTime = true;
    vector<double> eeVector;
    for(auto task : handoffTasks){
      sg->GetMPLibrary()->SetTask(task.get());
      MPSolution* sol = new MPSolution(sg->GetTaskPlan()->GetCoordinator()->GetRobot());
      sg->GetMPLibrary()->SetMPSolution(sol);
      std::vector<vector<double>> seedPoints; //Added to be the seed for each task
      for(auto agent : unusedAgents){
        if(agent->GetCapability() == task->GetCapability()) {
          agentTasks[task] = agent;
          Robot* tempRobot = problemCopy->GetRobot(agent->GetRobot()->GetLabel());
          std::unique_ptr<MPSolution> handoffSolution(new MPSolution(tempRobot));
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
          if(firstTime){
            samplerLabel = m_firstSamplerLabel;
          }
          auto sampler = sg->GetMPLibrary()->GetSampler(samplerLabel);
          cout << "Attempting a robot goal with this sampler: ";
          sampler->Print(std::cout);
          size_t numNodes = 10, numAttempts = 100;
          if(firstTime) {
            cout << "First time running sampler with no constraints" << endl;
            firstTime = false;
            bool keepSampling = true;
            while(keepSampling){
              goalPoints.clear();
              seedPoints.clear();
              //sample first robot, no end effector constraints, assumes this will complete
              sampler->Sample(numNodes, numAttempts, taskBoundary,std::back_inserter(goalPoints));

	          //compute end effector placement (for future sampling constraint)
              if(goalPoints.empty()){
                throw RunTimeException(WHERE, "No valid final handoff position for the robot.");
              }
              //Maybe change the keepSampling Flag to check for a certain amount of valid points
              keepSampling = false; //may need to change my conditions for sampling.
              for (size_t i=0; i<goalPoints.size(); i++){ //we're using all nodes (10 for now)
                  goalPoints[i].ConfigureRobot();
                  auto it = tempRobot->GetWorldContactPoint().begin();
                  eeVector.clear();
                  for (size_t w=0; w<3; ++w){
                    eeVector.push_back(*it);
                    it++;
                  }
                  seedPoints.push_back(eeVector); //Adding these points to the seedPoints object such that the following robots will sample around it
              }
            }
          }
          else{
            cout << "Subsequent time running sampler with ee constraints" << endl;

	      //cout << "Grip Point: "; std::copy(eeVector.begin(), eeVector.end(), std::ostream_iterator<double>(cout, " ")); cout << endl;
            //WorkspaceBoundingSphere endEffectorBoundary(eeVector, endEffectorBoundaryRadius);
            //cout << "End Effector Boundary: " << endEffectorBoundary << endl;
            //putting a sphere around each goalpoint (ideally)
            //Handoff Distance is arbitrary and was added here to test things
            double handoffRadius = .5; //taken from end effector boundary radius
	        //seedpoints.size
	        WorkspaceBoundingSphere* seedBoundaries;
	        seedBoundaries = (WorkspaceBoundingSphere*) malloc(sizeof(WorkspaceBoundingSphere)*seedPoints.size());
	        for (unsigned int i=0; i<seedPoints.size(); i++){ //pushing back the boundaries
	    	  seedBoundaries[i] = WorkspaceBoundingSphere(seedPoints[i], handoffRadius);
	        }
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
            
            //Attempt to sample only around seed locations
            for(unsigned int i=0; i<seedPoints.size(); i++){
              auto pointBoundary = seedBoundaries[i];
              int samplesDone = 0; //going to use this as a counter just in case
              //50 sampling attempts near end effector boundary
              //Dont know if the extra attempts are necessary with RV
              while((samplesDone<50) && (goalPoints.empty())){
                vector<double> secondEEVector;
                vector<Cfg> tempGoalPoints;
                if(samplerLabel=="RV"){
                  sampler->Sample(numNodes, numAttempts, envBoundary, &pointBoundary, std::back_inserter(tempGoalPoints));
                }else{//Should probably throw an error here
                  sampler->Sample(numNodes, numAttempts, taskBoundary, std::back_inserter(tempGoalPoints));
                }
                //checking how close these temp values are to the goalpoints
                //dont know if this is necessary given sampling is done with RV
                for (auto it=tempGoalPoints.begin(); it!=tempGoalPoints.end(); it++){
                  it->ConfigureRobot();
                  auto it2 = tempRobot->GetWorldContactPoint().begin();
                  for (size_t w=0; w<3; ++w){
                    secondEEVector.push_back(*it2);
                    it2++;
                  }
                  //passing closeness check
                  double diffX = pow(seedPoints[i][0]-secondEEVector[0], 2);
                  double diffY = pow(seedPoints[i][1]-secondEEVector[1], 2);
                  double diffZ = pow(seedPoints[i][2]-secondEEVector[2], 2);
                  allEEDist.push_back(sqrt(diffX+diffY+diffZ));
                  if (allEEDist.back()<=1){ 
                    goalPoints.push_back(*it);
                  }
                }
                if (goalPoints.empty()){
                  numSamplerRuns++;
                } 
		      }
    
            }
          }
          tempRobot->SetVirtual(false);

          if(goalPoints.empty()){
            throw RunTimeException(WHERE, "No valid final handoff position for the robot.");
          }
          //Added this section to configure in each goal configuration
          //if GoalPoints are not empty they should be added to the interaction template
          for (size_t x = 0; x<goalPoints.size()-1; x++){
            goalPoints[x].ConfigureRobot();
            sg->GetMPLibrary()->Solve(problemCopy.get(), task.get(),
                handoffSolution.get(), currentTemplate->GetInformation()->GetMPStrategy()
                , LRand(), currentTemplate->GetInformation()->GetMPStrategy());
            handoffSolution->GetRoadmap()->Write("indHandoffTemplate" + std::to_string(check) + ".map", problemCopy->GetEnvironment());

            // Store the roadmap for each task in the handoff
            auto rob = handoffSolution->GetRoadmap()->GetRobot();
            handoffSolution->GetRoadmap()->SetRobot(originalProblem->GetRobot(rob->GetLabel()));
            currentTemplate->AddRoadmap(handoffSolution->GetRoadmap());
            currentTemplate->AddPath(handoffSolution->GetPath()->Cfgs(), originalProblem);
            currentTemplate->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), originalProblem);
          }
          cout << "Robot Cfg: " << goalPoints[0].PrettyPrint() << endl;
          break;
        }
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
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the unused agents to virtual before planning.
  }
}
