#include "DisjointWorkspaces.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

#include "Utilities/MPUtils.h"

DisjointWorkspaces::
DisjointWorkspaces(MPProblem* _problem) : ITPlacementMethod(_problem) {}


DisjointWorkspaces::
DisjointWorkspaces(XMLNode& _node) : ITPlacementMethod(_node) {
  m_precision = _node.Read("precision", false, 4., 1., 100.,
              "How many divisions around each edge of the boundary do you want to sample?");
  m_maxAttempts = _node.Read("maxAttempts", true, nan(""), 0., 1000., "Max number of attempts to place a CFG");
}

std::unique_ptr<PlacementMethod>
DisjointWorkspaces::
Clone(){
	return std::unique_ptr<PlacementMethod>(new DisjointWorkspaces(*this));	
}

void
DisjointWorkspaces::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, TMPStrategyMethod* _tmpMethod){
  //_solution->AddInteractionTemplate(_it);

  auto tasks = _it->GetInformation()->GetInteractionTasks();

  //auto coordinator = static_cast<Coordinator*>(tasks[0]->GetRobot()->GetAgent());

  // Assuming two robots for now
  // Get the capability to determine the workspaces

  std::vector<HandoffAgent*> capabilityAgents;
  for(auto& task : tasks){
    auto agent = _tmpMethod->GetCapabilityAgent(task->GetCapability());
    capabilityAgents.push_back(agent);
  }

  // Sample over the workspace and find density of configurations
  Robot* oldRobot = nullptr;
  if(_library->GetTask()){
    oldRobot = _library->GetTask()->GetRobot();
  }
  else {
    auto task = new MPTask(_tmpMethod->GetRobot());
    _library->SetTask(task);
  }

  // Sample over the workspace and find density of configurations
  // Look at terrain boundaries and density around those
  // Maybe sample around the boundaries

  auto terrainMap = m_problem->GetEnvironment()->GetTerrains();
  for(auto& agent : capabilityAgents){
    std::string cap = agent->GetRobot()->GetCapability();
    auto& terrains = terrainMap[cap];
    for(auto& terrain : terrains){
      // Boundaries around the terrain to sample for the other capability
      // involved in the interaction
      //std::vector<WorkspaceBoundingBox*> sampleSpaces;

      std::vector<Cfg> samplePoints;

      auto boundary = terrain.GetBoundary();
      //TODO::Implement checks for other types of boundaries
      if(boundary->Name() == "WorkspaceBoundingBox"){
        auto box = static_cast<const WorkspaceBoundingBox*>(boundary);
        auto center = box->GetCenter();
        std::cout << cap << " RANGES: " << std::endl;;
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
          auto z = 0.0;
          for(int i=0; i < m_maxAttempts; i++){
            double y = GetRandomDouble(yMin,yMax);
            Cfg sampleLeft({x,y,z},agent->GetRobot());
            sampleLeft[2] = z;
            if(CheckLocation(sampleLeft, _library, _it)){
              samplePoints.push_back(sampleLeft);
              break;
            }
          }

          x = box->GetRange(0).max;
          z = 1;
          for(int i=0; i < m_maxAttempts; i++){
            double y = GetRandomDouble(yMin,yMax);
            Cfg sampleRight({x,y,z},agent->GetRobot());
            sampleRight[2] = z;
            if(CheckLocation(sampleRight, _library, _it)){
              samplePoints.push_back(sampleRight);
              break;
            }
          }
          z = .5;
          auto y = box->GetRange(1).min;
          auto xMin = box->GetRange(0).min + toaddX;
          auto xMax = box->GetRange(0).min + xSize*i;
          for(int i=0; i < m_maxAttempts; i++){
            double x = GetRandomDouble(xMin,xMax);
            Cfg sampleBottom({x,y,z},agent->GetRobot());
            sampleBottom[2] = z;
            if(CheckLocation(sampleBottom, _library, _it)){
              samplePoints.push_back(sampleBottom);
              break;
            }
          }

          y = box->GetRange(1).max;
          z = -.5;//was positive
          for(int i=0; i < m_maxAttempts; i++){
            double x = GetRandomDouble(xMin,xMax);
            Cfg sampleTop({x,y,z},agent->GetRobot());
            sampleTop[2] = z;
            if(CheckLocation(sampleTop, _library, _it)){
              samplePoints.push_back(sampleTop);
              break;
            }
          }

          toaddX += xSize;
          toaddY += ySize;

          /*
          auto x = box->GetRange(0).min;
          auto y = box->GetRange(1).min + ySize*i; // + ySize/2;
          auto z = 0.0;
          Cfg sampleLeft({x,y,z},_coordinator->GetRobot());

          x = box->GetRange(0).max;
          Cfg sampleRight({x,y,z},_coordinator->GetRobot());

          samplePoints.push_back(sampleLeft);
          samplePoints.push_back(sampleRight);

          x = box->GetRange(0).min + xSize*i; // - xSize/2;
          y = box->GetRange(1).min;
          Cfg sampleBottom({x,y,z},_coordinator->GetRobot());

          y = box->GetRange(1).max;
          Cfg sampleTop({x,y,z},_coordinator->GetRobot());

          samplePoints.push_back(sampleBottom);
          samplePoints.push_back(sampleTop);

          */

        }
      }
      std::cout << "Sample Points" << std::endl;
      for(auto cfg : samplePoints){
        if(m_debug){
          std::cout << "Robot Label: " << cfg.GetRobot()->GetLabel() << std::endl;
          std::cout << cfg.PrettyPrint() << std::endl;
        }
        /*
           bool valid = CheckLocation(cfg, _library, _it);//, capabilityAgents);
           if(valid){
           _it->GetInformation()->AddTemplateLocation(cfg);
           }
           */
        _it->GetInformation()->AddTemplateLocation(cfg);
      }
      if(m_debug and samplePoints.empty()){
        std::cout << "No locations found for: " << terrain.GetBoundary()->GetCenter() << std::endl;
      }
    }
    break;
  }

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
    _library->GetTask()->SetRobot(oldRobot);
  }
  else{
    _library->SetTask(nullptr);
  }
  if(m_debug){
    std::cout << "OUTPUTTING ALL TEMPLATE LOCATIONS FOUND." << std::endl;
    for(auto location : _it->GetInformation()->GetTemplateLocations()){
      std::cout << location.PrettyPrint() << std::endl;
    }
  }
}

void
DisjointWorkspaces::
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

bool
DisjointWorkspaces::
CheckLocation(Cfg _cfg, MPLibrary* _library, InteractionTemplate* _it){
  auto vcm = _library->GetValidityChecker("terrain_solid");
  std::cout << "Checking Location" << std::endl;
  std::cout << _cfg.PrettyPrint() << std::endl;
  bool valid = true;
  for(auto position : _it->GetPositions()){
    Cfg relativeCfg = position;
    std::cout << "Relative Cfg" << std::endl;
    std::cout << relativeCfg.PrettyPrint() << std::endl;
    //TODO::Change robot to appropriate robot capability
    //TranslateCfg(_cfg, relativeCfg);
    relativeCfg.TransformCfg(_cfg);
    if(!vcm->IsValid(relativeCfg, "ValidateITCfg")){
      valid = false;
      if(m_debug){
        std::cout << "Invalid Cfg: " <<  relativeCfg.PrettyPrint() << std::endl;
      }
      break;
    }
    auto envBoundary = m_problem->GetEnvironment()->GetBoundary();
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
