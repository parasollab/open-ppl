#include "Cfg_free_tree_2dof.h"
#include "MultiBody.h"
#include "Environment.h"

size_t Cfg_free_tree::m_numOfJoints;

Cfg_free_tree_2dof::Cfg_free_tree_2dof(){
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);
};

Cfg_free_tree_2dof::~Cfg_free_tree_2dof(){}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(const Cfg& _c) {
  vector<double> _v;
  _v = _c.GetData();
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
}

vector<Robot> Cfg_free_tree_2dof::GetRobots(int _numJoints){
  m_numOfJoints = _numJoints;
  vector<Robot> robots;
  Robot::JointMap joints;
  for(int i = 0; i<_numJoints; i++){
    joints.push_back(make_pair(make_pair(i, i+1), Robot::SPHERICAL));
  }
  robots.push_back(Robot(Robot::VOLUMETRIC, Robot::ROTATIONAL, joints, 0));
  return robots;
}

const string Cfg_free_tree_2dof::GetName() const {
  return "Cfg_free_tree_2dof";
}

bool Cfg_free_tree_2dof::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
     
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 m_v[5]*TWOPI, 
						 m_v[4]*TWOPI, 
						 m_v[3]*TWOPI), // RPY
				     Vector3D(m_v[0],m_v[1],m_v[2]));
  
  _env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);  // update link 1.
  for(size_t i=0; i<m_numOfJoints; i+=2) {
    _env->GetMultiBody(robot)->GetFreeBody(i/2+1)
      ->GetBackwardConnection(0).GetDHparameters().alpha = m_v[i+6]*TWOPI;
    _env->GetMultiBody(robot)->GetFreeBody(i/2+1)
      ->GetBackwardConnection(0).GetDHparameters().theta = m_v[i+1+6]*TWOPI;
  }  // config the robot
  
  
  for(int i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); i++) {
    shared_ptr<FreeBody> afb = _env->GetMultiBody(robot)->GetFreeBody(i);
    if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
      afb->GetWorldTransformation();
  }
  
  // since Transformation is calculated in recursive manner, only
  // let the last link(or Freebody) call getWorldTransformation will
  // automatically calculate the transformations for all previous links.
  
  // when all worldTransformations are recalculated by using new cfg, the
  // config of the whole robot is updated.
  
  return true;
}
