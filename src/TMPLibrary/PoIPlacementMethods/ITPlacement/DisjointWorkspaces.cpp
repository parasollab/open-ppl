#include "DisjointWorkspaces.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/Agent.h"

#include "TMPLibrary/TaskPlan.h"

#include "Utilities/MPUtils.h"

DisjointWorkspaces::
DisjointWorkspaces(){
	this->SetName("DisjointWorkspaces");
}

DisjointWorkspaces::
DisjointWorkspaces(XMLNode& _node) : ITPlacementMethod(_node) {
	this->SetName("DisjointWorkspaces");
  m_precision = _node.Read("precision", false, 4., 1., 100.,
              "How many divisions around each edge of the boundary do you want to sample?");
  m_maxAttempts = _node.Read("maxAttempts", true, nan(""), 0., 1000., "Max number of attempts to place a CFG");
}

std::unique_ptr<ITPlacementMethod>
DisjointWorkspaces::
Clone(){
	return std::unique_ptr<ITPlacementMethod>(new DisjointWorkspaces(*this));	
}

void
DisjointWorkspaces::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution){
  //_solution->AddInteractionTemplate(_it);

	m_sampled.clear();

  auto tasks = _it->GetInformation()->GetInteractionTasks();

  //auto coordinator = static_cast<Coordinator*>(tasks[0]->GetRobot()->GetAgent());

  // Assuming two robots for now
  // Get the capability to determine the workspaces

  std::vector<Agent*> capabilityAgents;
  for(auto& task : tasks){
    auto agent = this->GetTaskPlan()->GetCapabilityAgent(task->GetCapability());
    capabilityAgents.push_back(agent);
  }

  // Sample over the workspace and find density of configurations
  Robot* oldRobot = nullptr;
  if(this->GetMPLibrary()->GetTask()){
    oldRobot = this->GetMPLibrary()->GetTask()->GetRobot();
  }
  else {
		//TODO::Kind of a hack and need to evaluate
    auto task = new MPTask(this->GetTaskPlan()->GetCoordinator()->GetRobot());
    this->GetMPLibrary()->SetTask(task);
  }

  // Sample over the workspace and find density of configurations
  // Look at terrain boundaries and density around those
  // Maybe sample around the boundaries

  auto terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();
  for(size_t i = 0; i < capabilityAgents.size(); i++){
		for(size_t j = 0; j < capabilityAgents.size(); j++){
			auto agent1 = capabilityAgents[i]; 
			auto agent2 = capabilityAgents[j]; 
			if(agent1 == agent2)
				continue;
			for(auto& terrain1 : terrainMap[agent1->GetRobot()->GetCapability()]){
				for(auto& terrain2 : terrainMap[agent2->GetRobot()->GetCapability()]){
					if(!terrain1.IsNeighbor(terrain2))
						continue;
					if(terrain1.GetPerimeter() < terrain2.GetPerimeter()){
						bool receiving = (agent1->GetRobot()->GetCapability() 
															== _it->GetInformation()->GetTypeTasks("receiving")[0]->GetCapability());
						SampleBorder(terrain1, receiving, _it, agent1);
					}
					else {
						bool receiving = (agent2->GetRobot()->GetCapability() 
															== _it->GetInformation()->GetTypeTasks("receiving")[0]->GetCapability());
						SampleBorder(terrain2, receiving, _it, agent1);
					}
				}
			}
		}
	}
	
	/*for(auto& agent : capabilityAgents){
    std::string cap = agent->GetRobot()->GetCapability();
    auto& terrains = terrainMap[cap];
		
		bool receiving = (cap == _it->GetInformation()->GetTypeTasks("receiving")[0]->GetCapability());

    for(auto& terrain : terrains){
      if(terrain.IsVirtual())
				continue;
			// Boundaries around the terrain to sample for the other capability
      // involved in the interaction
      //std::vector<WorkspaceBoundingBox*> sampleSpaces;

      std::vector<Cfg> samplePoints;

      //auto boundary = terrain.GetBoundary();
    }
		if(!_it->GetInformation()->GetTemplateLocations().empty())
			break;
  }
	*/
  // Testing to see if Coordinator interface works

  /*if(_it->GetInformation()->GetLabel() == "robotmatingsequence1"){
    Cfg location({4,0,0}, coordinator->GetRobot());
    _it->GetInformation()->AddTemplateLocation(location);
  }
  else {
    Cfg location({6,0,0}, coordinator->GetRobot());
    _it->GetInformation()->AddTemplateLocation(location);
  }*/

  if(oldRobot){
    this->GetMPLibrary()->GetTask()->SetRobot(oldRobot);
  }
  else{
    this->GetMPLibrary()->SetTask(nullptr);
  }
  if(m_debug){
    std::cout << "OUTPUTTING ALL TEMPLATE LOCATIONS FOUND." << std::endl;
    for(auto location : _it->GetInformation()->GetTemplateLocations()){
      std::cout << location.PrettyPrint() << std::endl;
    }
  }
}


bool
DisjointWorkspaces::
CheckLocation(Cfg _cfg, InteractionTemplate* _it){
  auto vcm = this->GetMPLibrary()->GetValidityChecker("terrain_solid");
  std::cout << "Checking Location" << std::endl;
  std::cout << _cfg.PrettyPrint() << std::endl;
  bool valid = true;
  for(auto position : _it->GetPositions()){
    Cfg relativeCfg = position;
    std::cout << "Relative Cfg" << std::endl;
    std::cout << relativeCfg.PrettyPrint() << std::endl;
    //TODO::Change robot to appropriate robot capability
    //TranslateCfg(_cfg, relativeCfg);
    relativeCfg.TransformCfg(_cfg.GetBaseTransformation());
    if(!vcm->IsValid(relativeCfg, "ValidateITCfg")){
      valid = false;
      if(m_debug){
        std::cout << "Invalid Cfg: " <<  relativeCfg.PrettyPrint() << std::endl;
      }
      break;
    }
    auto envBoundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();
    if(!envBoundary->InBoundary(relativeCfg)){
      if(m_debug){
        std::cout << "Invalid Cfg: " <<  relativeCfg.PrettyPrint() << std::endl;
      }
      valid = false;
      break;
    }
  }
  if(!valid){
    std::cout << "FAILING TO FIND STUFF HERE" << std::endl;
  }
  else {
    std::cout << "Valid" << std::endl;
  }
  std::cout << "Done checking validity for" + _cfg.PrettyPrint() << std::endl;
  return valid;
}


double
DisjointWorkspaces::
GetRandomDouble(double _min, double _max){
  //std::uniform_real_distribution<double> unif(_min, _max);
  //std::default_random_engine re;
  double re = (double)DRand();// RAND_MAX;
  return _min + re * (_max - _min);
  //return unif(re);
}

void
DisjointWorkspaces::
SampleBorder(Terrain& _terrain, bool _receiving, InteractionTemplate* _it, Agent* _agent){

      std::vector<Cfg> samplePoints;
			for(auto& boundary : _terrain.GetBoundaries()){
      	//TODO::Implement checks for other types of boundaries
      	if(m_sampled[boundary.get()])
					continue;
				m_sampled[boundary.get()] = true;
      	if(boundary->Name() == "WorkspaceBoundingBox"){
        	auto box = static_cast<const WorkspaceBoundingBox*>(boundary.get());
        	auto center = box->GetCenter();
        	std::cout << _agent->GetRobot()->GetCapability() << " RANGES: " << std::endl;;
        	for(size_t range = 0; range < box->GetDimension(); range++){
          	std::cout << box->GetRange(range) << std::endl;
        	}
        	//Sample sides of box
        	auto width = box->GetRange(0).Length();
        	auto xSize = width / m_precision;
        	auto height = box->GetRange(1).Length();
        	auto ySize = height / m_precision;

        	std::cout << "Width: " << width << " Height: " << height << std::endl
                  << "xSize: " << xSize << " ySize: " << ySize << std::endl;

        	double toaddX = 0;
        	double toaddY = 0;

        	for(size_t i = 1; i <= m_precision; i++){
          /*auto borderBoxLeft = new WorkspaceBoundingBox(box->GetDimension());
          auto borderBoxRight = new WorkspaceBoundingBox(box->GetDimension());
          Range<double> h(box->GetRange(1).min + i*size, box->GetRange(1).min + (i+1)*size);

          //TODO::Set to proper robot radius times some factor
          auto radius = 1;
          Range<double> leftW(box->GetRange(0).min+radius, box->GetRange(0).min);
          Range<double> rightW(box->GetRange(0).max, box->GetRange(0).max+radius);

          Range<double> z(-.5,5);

          borderBoxLeft->SetRange(0, leftW);
          borderBoxLeft->SetRange(1, h);
          borderBoxLeft->SetRange(2,z);
}

          borderBoxRight->SetRange(0, rightW);
          borderBoxRight->SetRange(1, h);
          borderBoxRight->SetRange(2,z);

          sampleSpaces
          */

          	auto x = box->GetRange(0).min;
          	auto yMin = box->GetRange(1).min + toaddY;
          	auto yMax = box->GetRange(1).min + ySize*i;
          	auto z = 1.0;
          	if(!_receiving)
							z = 0;	
						for(size_t i=0; i < m_maxAttempts; i++){
            	double y = GetRandomDouble(yMin,yMax);
            	Cfg sampleLeft({x,y,z},_agent->GetRobot());
            	sampleLeft[2] = z;
            	if(CheckLocation(sampleLeft, _it)){
              	samplePoints.push_back(sampleLeft);
              	break;
            	}
          	}

          	x = box->GetRange(0).max;
          	(_receiving) ? z = 0 : z = 1;
						for(size_t i=0; i < m_maxAttempts; i++){
            	double y = GetRandomDouble(yMin,yMax);
            	Cfg sampleRight({x,y,z},_agent->GetRobot());
            	sampleRight[2] = z;
            	if(CheckLocation(sampleRight, _it)){
              	samplePoints.push_back(sampleRight);
              	break;
            	}
          	}
          	(_receiving) ? z = -.5 : z = .5;
          	auto y = box->GetRange(1).min;
          	auto xMin = box->GetRange(0).min + toaddX;
          	auto xMax = box->GetRange(0).min + xSize*i;
          	for(size_t i=0; i < m_maxAttempts; i++){
            	double x = GetRandomDouble(xMin,xMax);
            	Cfg sampleBottom({x,y,z},_agent->GetRobot());
            	sampleBottom[2] = z;
            	if(CheckLocation(sampleBottom, _it)){
              	samplePoints.push_back(sampleBottom);
              	break;
            	}
          	}

          	y = box->GetRange(1).max;
          	(_receiving) ? z = .5 : z = -.5;
          	for(size_t i=0; i < m_maxAttempts; i++){
            	double x = GetRandomDouble(xMin,xMax);
            	Cfg sampleTop({x,y,z},_agent->GetRobot());
            	sampleTop[2] = z;
            	if(CheckLocation(sampleTop, _it)){
              	samplePoints.push_back(sampleTop);
              	break;
            	}
          	}

          	toaddX += xSize;
          	toaddY += ySize;
        	}
      	}
			}
      if(m_debug) 
				std::cout << "Sample Points" << std::endl;
      for(auto cfg : samplePoints){
        if(m_debug){
          std::cout << "Robot Label: " << cfg.GetRobot()->GetLabel() << std::endl;
          std::cout << cfg.PrettyPrint() << std::endl;
        }
        /*
           bool valid = CheckLocation(cfg, this->GetMPLibrary(), _it);//, capabilityAgents);
           if(valid){
           _it->GetInformation()->AddTemplateLocation(cfg);
           }
           */
        _it->GetInformation()->AddTemplateLocation(cfg);
      }
      if(m_debug and samplePoints.empty()){
        std::cout << "No locations found for: " << _terrain.GetBoundaries()[0]->GetCenter() << std::endl;
      }
}
