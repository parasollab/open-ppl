#include "GraspStrategy.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"

#include "MPProblem/Robot/Kinematics/ur_kin.h"

/*----------------------- Construction -----------------------*/

GraspStrategy::
GraspStrategy() {
  this->SetName("GraspStrategy");
}

GraspStrategy::
GraspStrategy(XMLNode& _node) : InteractionStrategyMethod(_node) {
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

  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Get initial conditions
  auto stages = _interaction->GetStages();
  auto initialConditions = _interaction->GetStageConditions(stages[0]);

  // Assign roles
  AssignRoles(_start,initialConditions);

  // Grab object robot models
  std::vector<Robot*> objects;
  for(auto kv : _start) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      if(r->GetMultiBody()->IsPassive())
        objects.push_back(r);
    }
  }

  // Sample object placements
  std::map<Robot*,Cfg> objectPoses;
  for(auto object : objects) {
    auto objectPose = SampleObjectPose(object,_interaction);
    objectPoses[object] = objectPose;
  }

  std::unordered_map<Robot*,Transformation> eeFrames;
  //temp hardcode to second stage
  auto pregraspConditions = _interaction->GetStageConditions(stages[1]);
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

      auto frame = ComputeEEWorldFrame(cfg, transform);
      
      auto refRobot = m_roleMap[roleInfo.referenceRole];
      auto refBase = refRobot->GetMultiBody()->GetBase();
      auto refBaseTransformation = refBase->GetWorldTransformation();

      // Current have to rotate base of ur5e bc of weird urdf stuff, but it messes up this calculation
      if(m_doctorBaseOrientation)
        refBaseTransformation = Transformation(refBaseTransformation.translation());

      auto translation = (-refBaseTransformation).rotation() * frame.translation() + (-refBaseTransformation).translation();
      auto rotation = (-refBaseTransformation).rotation() * frame.rotation();

      auto eeFrame = Transformation(translation,rotation);
      eeFrames[refRobot] = eeFrame;
    }
  }

  std::unordered_map<Robot*,Cfg> manipCfgs;
  for(auto kv : eeFrames) {
    auto cfg = ComputeManipulatorCfg(kv.first,kv.second);
    manipCfgs[kv.first] = cfg;
  }
  

  // TODO::Iterate through stages

  // TODO::For each stage
    //TODO::Set/sample object pose based off of static status

    //TODO::Compute EE placement relative object pose

    //TODO::Compute joint angles using IK

    //TODO::Compute path from previous stage to next stage

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
  }

  // Assign roles from filtered conditions
  InteractionStrategyMethod::AssignRoles(_state,filteredConditions);
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
  // Temp
  Cfg cfg(_objectPose.GetRobot());
  cfg.ConfigureRobot();

  auto base = _objectPose.GetRobot()->GetMultiBody()->GetBase();
  auto baseFrame = base->GetWorldTransformation();  

  auto translation = (-_transform).rotation() * baseFrame.translation() + _transform.translation();
  auto rotation = (-_transform).rotation() * baseFrame.rotation();
  return Transformation(translation,rotation);
}

Cfg
GraspStrategy::
ComputeManipulatorCfg(Robot* _robot, Transformation& _transform) {

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


  for(int i=0;i<4;i++) {
    for(int j=i*4;j<(i+1)*4;j++)
      printf("%1.3f ", T[j]);
    printf("\n");
  }
  auto num_sols = ur_kinematics::inverse(T, q_sols);
  for(int i=0;i<num_sols;i++) 
    printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
       q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
  if(num_sols == 0)
    return Cfg(nullptr);

  //TODO::Validity check cfg and if invalid try the next solution

  std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

  data[0] =  q_sols[2]/PI; 
  data[1] =  0;
  data[2] =  q_sols[1]/PI;
  data[3] =  q_sols[0]/PI;
  data[4] =  q_sols[3]/PI;
  data[5] =  q_sols[4]/PI;
  data[6] =  q_sols[5]/PI;

  for(size_t i = 0; i < data.size(); i++) {
    if(i == 1)
      continue;

    if(data[i] > 1) {
      data[i] = -2 + data[i];
    }
  }
 
  Cfg cfg(_robot);
  cfg.SetData(data);
  return cfg;
}
