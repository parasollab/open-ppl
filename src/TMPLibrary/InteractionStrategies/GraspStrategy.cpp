#include "GraspStrategy.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/Solution/Plan.h"

#ifdef PPL_USE_URDF
  #include "MPProblem/Robot/Kinematics/ur_kin.h"
#endif
/*----------------------- Construction -----------------------*/

GraspStrategy::
GraspStrategy() {
  this->SetName("GraspStrategy");
}

GraspStrategy::
GraspStrategy(XMLNode& _node) : IndependentPaths(_node) {
  this->SetName("GraspStrategy");

  m_objectSMLabel = _node.Read("objectSMLabel",true, "",
                    "SamplerMethod used to create object pose samples.");
  m_objectVCLabel = _node.Read("objectVCLabel", true, "",
                    "ValidityChecker used to check object samples.");
  m_maxAttempts = _node.Read("maxAttempts", false, 100, 1, MAX_INT,
                  "Max number of attempts to get a valid pose sample.");
}

GraspStrategy::
~GraspStrategy() {}

/*------------------------ Interface -------------------------*/

bool
GraspStrategy::
operator()(Interaction* _interaction, State& _start) {

  auto problem = this->GetPlan()->GetCoordinator()->GetRobot()->GetMPProblem();
  auto lib = this->GetMPLibrary();

  _interaction->Initialize();

  // Get initial conditions
  auto stages = _interaction->GetStages();

  // Check that there are at least 4 stages
  if(stages.size() < 3)
    throw RunTimeException(WHERE) << "Grasp sampling assumes at least 3 stages:"
                                     "\n\t0:Groups"
                                     "\n\t1:PreGrasp stages"
                                     "\n\t2:Grasp"
                                     "\n\t(Optional): PostGrasp stages"
                                  << std::endl;

  auto initialConditions = _interaction->GetStageConditions(stages[0]);

  // Assign roles
  AssignRoles(_start,initialConditions);

  std::string allRobotsLabel;
  std::vector<Robot*> allRobots;

  // Grab object robot models
  std::vector<Robot*> objects;
  std::string allObjectsLabel;

  // Construct initial groups
  std::unordered_map<Robot*,RobotGroup*> initialGroups;

  auto originalStart = _start;

  for(auto kv : _start) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {

      initialGroups[r] = group;

      // Save object robots
      if(r->GetMultiBody()->IsPassive()) {
        // Add obj to set of all objects
        objects.push_back(r);
        allObjectsLabel += "::"+r->GetLabel();
      }
      else  {
        // Add manip to set of all robots
        allRobots.push_back(r);
        allRobotsLabel += "::"+r->GetLabel();
      }
    }
  }

  auto objGroup = problem->AddRobotGroup(objects,allObjectsLabel);
  auto group = problem->AddRobotGroup(allRobots,allRobotsLabel);

  //TODO::Figure out where grasp stage is
  size_t graspStage = stages.size() - 1;
  for(size_t i = 1; i < graspStage; i++) {

    // Add composite groups to to grasp solution
    auto toStageSolution = _interaction->GetToStageSolution(stages[i+1]);
    toStageSolution->AddRobotGroup(objGroup);
    auto objGrm = toStageSolution->GetGroupRoadmap(objGroup);

    toStageSolution->AddRobotGroup(group);
    lib->SetMPSolution(toStageSolution);
    auto grm = toStageSolution->GetGroupRoadmap(group);

    // Get object placements
    std::map<Robot*,Cfg> objectPoses;
    for(auto object : objects) {  
      Cfg objectPose(object);

      // Check if object placement is given
      auto initGroup = initialGroups[object];
      auto given = originalStart[initGroup];
      if(given.first) {
        auto gcfg = given.first->GetVertex(given.second);
        objectPose = gcfg.GetRobotCfg(object);
      }
      // If not, sample object pose
      else {
        objectPose = SampleObjectPose(object,_interaction);
      }

      objectPoses[object] = objectPose;
      objectPose.ConfigureRobot();
    }

    // Sample pregrasp joint angles
    auto eeFrames = ComputeEEFrames(_interaction,objectPoses,i);

    std::unordered_map<Robot*,Cfg> pregraspCfgs;
    for(auto kv : eeFrames) {
      auto cfg = ComputeManipulatorCfg(kv.first,kv.second);
      if(!cfg.GetRobot()) {
        std::cout << "Failed to find a valid grasp pose for " << kv.first->GetLabel();
        m_roleMap.clear();
      	return false;
      }
      SetEEDOF(_interaction,cfg,stages[i]);
      pregraspCfgs[kv.first] = cfg;
    }

    // Sample grasp joint angles
    auto nextStageEEFrames = ComputeEEFrames(_interaction,objectPoses,i+1);
    std::unordered_map<Robot*,Cfg> graspCfgs;
    for(auto kv : nextStageEEFrames) {
      auto cfg = ComputeManipulatorCfg(kv.first,kv.second);
      SetEEDOF(_interaction,cfg,stages[i+1]);

      graspCfgs[kv.first] = cfg;
    }

    // Setup group cfgs
    GroupCfg objGcfg(objGrm);
    for(auto kv : objectPoses) {
      objGcfg.SetRobotCfg(kv.first,std::move(kv.second));
    }

    GroupCfg pregrasp(grm);
    for(auto kv : pregraspCfgs) {
      pregrasp.SetRobotCfg(kv.first,std::move(kv.second));
    }

    GroupCfg grasp(grm);
    for(auto kv : graspCfgs) {
      grasp.SetRobotCfg(kv.first,std::move(kv.second));
    }

    // Get start constraints
    auto objVID = objGrm->AddVertex(objGcfg);
    auto preGraspVID = grm->AddVertex(pregrasp);
    State preGraspState;
    preGraspState[objGroup] = std::make_pair(objGrm,objVID);
    preGraspState[group] = std::make_pair(grm,preGraspVID);
    auto preGraspConstraints = GenerateConstraints(preGraspState);

    // Get goal constraints
    auto graspVID = grm->AddVertex(grasp);
    State graspState;
    graspState[objGroup] = std::make_pair(objGrm,objVID);
    graspState[group] = std::make_pair(grm,graspVID);
    auto graspConstraints = GenerateConstraints(graspState);

    // Set active formations for grasp planning problem
    auto startConditions = _interaction->GetStageConditions(stages[0]);
    SetActiveFormations(startConditions,toStageSolution);

    // Create grasp planning tasks
    auto graspTasks = GenerateTasks(startConditions,
        preGraspConstraints,
        graspConstraints);

    _interaction->SetToStageTasks(stages[i+1],graspTasks);

    // Configure objects as static robots
    std::set<Robot*> staticRobots;  
    for(auto obj : objects) {
      staticRobots.insert(obj);
    }

    ConfigureStaticRobots(staticRobots,preGraspState);

    auto toGraspPaths = PlanMotions(graspTasks,toStageSolution,
        "PlanInteraction::"+_interaction->GetLabel()+"::To"+stages[i+1],
        staticRobots,preGraspState);

    ResetStaticRobots();

    // Check if valid solution was found
    if(toGraspPaths.empty()) {
      m_roleMap.clear();
      return false;
    }

    // Save plan information
    _interaction->SetToStagePaths(stages[i+1],toGraspPaths);

    if(i+1 == graspStage) {
      _start = InterimState(_interaction,stages[i+1],stages[i+1],toGraspPaths);
      m_roleMap.clear();
      return true;
    }
    else 
      _start = InterimState(_interaction,stages[i+1],stages[i+2],toGraspPaths);
  }
  //TODO::Compute to post grasp path

  m_roleMap.clear();
  m_objectPoseTasks.clear();

  return true;
}

/*--------------------- Helper Functions ---------------------*/

void
GraspStrategy::
AssignRoles(const State& _state, const std::vector<std::string>& _conditions) {

  // If no start state, filter out motion conditions
  bool validStart = true;
  for(auto kv : _state) {
    if(!kv.second.first or kv.second.second == MAX_INT) {
      validStart = false;
      break;
    }
  }

  std::vector<std::string> filteredConditions;
  if(!validStart) {
    for(auto condition : _conditions) {
      auto c = this->GetTMPLibrary()->GetActionSpace()->GetCondition(condition);
      auto m = dynamic_cast<MotionCondition*>(c);
      if(!m) 
        filteredConditions.push_back(condition);
    }
    // Assign roles from filtered conditions
    InteractionStrategyMethod::AssignRoles(_state,filteredConditions);
  }
  else {
    InteractionStrategyMethod::AssignRoles(_state,_conditions);
  }
}

Cfg
GraspStrategy::
SampleObjectPose(Robot* _object, Interaction* _interaction) {

  auto library = this->GetMPLibrary();
  auto problem = _object->GetMPProblem();
  auto env = problem->GetEnvironment();
  const auto& surfaces = env->GetTerrains().at(_object->GetCapability());
 
  // Sample a stable object pose
  for(size_t i = 0; i < m_maxAttempts; i++) {

    // Create task for object pose
    auto task = std::unique_ptr<MPTask>(new MPTask(_object));
    m_objectPoseTasks.push_back(std::move(task));
    library->SetTask(m_objectPoseTasks.back().get());

    // Grab a random surface boundary
    size_t index = LRand() % surfaces.size();
    const auto& surface = surfaces[index];
    
    const auto& boundaries = surface.GetBoundaries();
    index = LRand() % boundaries.size();
    auto boundary = boundaries[index].get();

    // Sample within the surface boundary
    std::vector<Cfg> samples;
    auto sm = this->GetMPLibrary()->GetSampler(m_objectSMLabel);
    sm->Sample(1,1,boundary,std::back_inserter(samples));

    if(samples.empty())
      continue;

    auto cfg = samples[0];

    // Check if sample is valid
    auto vc = library->GetValidityChecker(m_objectVCLabel);
    if(vc->IsValid(cfg,this->GetNameAndLabel() + "::" + _interaction->GetLabel()))
      return cfg;
  }

  return Cfg(nullptr);
}

Transformation
GraspStrategy::
ComputeEEWorldFrame(const Cfg& _objectPose, const Transformation& _transform) {

  _objectPose.ConfigureRobot();

  auto base = _objectPose.GetRobot()->GetMultiBody()->GetBase();
  auto baseFrame = base->GetWorldTransformation();  

  //auto translation = (-_transform).rotation() * baseFrame.translation() + _transform.translation();
  //auto rotation = (-_transform).rotation() * baseFrame.rotation();
  //return Transformation(translation,rotation);

  return baseFrame * _transform;
}

std::unordered_map<Robot*,Transformation>
GraspStrategy::
ComputeEEFrames(Interaction* _interaction, std::map<Robot*,Cfg>& objectPoses, size_t _stage) {

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto stages = _interaction->GetStages();

  std::unordered_map<Robot*,Transformation> eeFrames;

  auto pregraspConditions = _interaction->GetStageConditions(stages[_stage]);
  for(auto condition : pregraspConditions) {
    auto f = dynamic_cast<FormationCondition*>(as->GetCondition(condition));
    if(!f)
      continue;

    for(auto role : f->GetRoles()) {
      auto robot = m_roleMap[role];
      if(!robot->GetMultiBody()->IsPassive())
        continue;

      const auto& roleInfo = f->GetRoleInfo(role); 
      Transformation transform = roleInfo.transformation;
      Cfg cfg = objectPoses[robot];

      auto frame = ComputeEEWorldFrame(cfg, -transform);
      
      auto refRobot = m_roleMap[roleInfo.referenceRole];
      auto refBase = refRobot->GetMultiBody()->GetBase();
      auto refBaseTransformation = refBase->GetWorldTransformation();

      // Current have to rotate base of ur5e bc of weird urdf stuff, but it messes up this calculation
      if(m_doctorBaseOrientation)
        refBaseTransformation = Transformation(refBaseTransformation.translation());

      auto translation = (-refBaseTransformation).rotation() * frame.translation() + (-refBaseTransformation).translation();
      auto rotation = (-refBaseTransformation).rotation() * frame.rotation();

      //auto translation = (-frame).rotation() * refBaseTransformation.translation() + (-frame).translation();
      //auto rotation = (-frame).rotation() * refBaseTransformation.rotation();

      auto eeFrame = Transformation(translation,rotation);
      eeFrames[refRobot] = eeFrame;
    }
  }

  return eeFrames;
}

Cfg
GraspStrategy::
ComputeManipulatorCfg(Robot* _robot, Transformation& _transform) {

  #ifdef PPL_USE_URDF

  std::cout << "Computing IK for a UR5e. Other robots not currently supported." << std::endl;

  //TODO::Convert transform to individual ur_kin format
 
  auto translation = _transform.translation();
  auto orientation = _transform.rotation().matrix();
 
  double* T = new double[16];
  double q_sols[8*6];
  // Point
  T[3] = translation[0];
  T[7] = translation[1];
  T[11] = translation[2];

  // Orientation matrix
  T[0] = orientation[0][0];
  T[1] = orientation[0][1];
  T[2] = orientation[0][2];

  T[4] = orientation[1][0];
  T[5] = orientation[1][1];
  T[6] = orientation[1][2];

  T[8] = orientation[2][0];
  T[9] = orientation[2][1];
  T[10] = orientation[2][2];

  T[12] = 0;
  T[13] = 0;
  T[14] = 0;
  T[15] = 1;


  if(m_debug) {
    for(int i=0;i<4;i++) {
      for(int j=i*4;j<(i+1)*4;j++)
        printf("%1.3f ", T[j]);
      printf("\n");
    }
  }

  auto num_sols = ur_kinematics::inverse(T, q_sols);

  if(m_debug) {
    for(int i=0;i<num_sols;i++) 
      printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
          q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
  }
  
  //TODO::Validity check cfg and if invalid try the next solution

  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

  for(int i = 0; i < num_sols; i++) {

    std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

    data[0] =  q_sols[2]/PI; 
    data[1] =  0;
    data[2] =  q_sols[1]/PI;
    data[3] =  q_sols[0]/PI;
    data[4] =  q_sols[3]/PI;
    data[5] =  q_sols[4]/PI;
    data[6] =  q_sols[5]/PI;

    for(size_t j = 0; j < data.size(); j++) {
      if(j == 1)
        continue;

      if(data[j] > 1) {
        data[j] = -2 + data[j];
      }
    }

    Cfg cfg(_robot);
    cfg.SetData(data);

    if(vc->IsValid(cfg,this->GetNameAndLabel()))
      return cfg;
  }

  //if(num_sols == 0)
  return Cfg(nullptr);

  #else

  throw RunTimeException(WHERE) << "IK not supported without ROS integration.";
  return Cfg(_robot);

  #endif

}

void
GraspStrategy::
SetEEDOF(Interaction* _interaction, Cfg& _cfg, const std::string& _stage) {
  
  Constraint* constraint = nullptr;
  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Grap role
  std::string role;
  for(auto kv : m_roleMap) {
    if(kv.second == _cfg.GetRobot()) {
      role = kv.first;
      break;
    }
  }

  for(auto condition : _interaction->GetStageConditions(_stage)) {
    auto c = as->GetCondition(condition);
    auto m = dynamic_cast<MotionCondition*>(c);
    if(!m)
      continue;
    constraint = m->GetRoleConstraint(role);
    if(constraint)
      break;
  }

  auto b = dynamic_cast<BoundaryConstraint*>(constraint);
  auto boundary = b->GetBoundary();

  for(size_t i = 0; i < _cfg.DOF(); i++) {
    
    auto range = boundary->GetRange(i);

    // Check if dof is ee 
    // TODO::temp assume ee dof is 1
    if(i == 1) {
      // Sample ee dof within boundary
      auto ee = range.Sample();
      _cfg[i] = ee;
    }
    else {
      // Ensure that cfg satisfies boundary
      if(!range.Contains(_cfg[i]))
        throw RunTimeException(WHERE) << "Sampled EE position does not satisfy condition constraint.";
    }
  }
}
