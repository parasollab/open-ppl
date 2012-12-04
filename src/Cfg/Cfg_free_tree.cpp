/////////////////////////////////////////////////////////////////////
//
//  Cfg_free_tree.c
//
//  General Description
//	A derived template class from CfgManager. It provides some
//	specific implementation directly related to a multiple joints
//	serial robot.
//
//  Created
//	08/31/99	Guang Song
/////////////////////////////////////////////////////////////////////

#include "Cfg_free_tree.h"
#include "MultiBody.h"
#include "Environment.h"

size_t Cfg_free_tree::m_numOfJoints;

Cfg_free_tree::Cfg_free_tree(){
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);
};

Cfg_free_tree::~Cfg_free_tree(){}

Cfg_free_tree::Cfg_free_tree(const Cfg& _c) {
  vector<double> _v;
  _v = _c.GetData();
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
}

vector<Robot> Cfg_free_tree::GetRobots(int _numJoints){
  m_numOfJoints = _numJoints;
  vector<Robot> robots;
  Robot::JointMap joints;
  for(int i = 0; i<_numJoints; i++){
    joints.push_back(make_pair(make_pair(i, i+1), Robot::REVOLUTE));
  }
  robots.push_back(Robot(Robot::VOLUMETRIC, Robot::ROTATIONAL, joints, 0));
  return robots;
}

const string Cfg_free_tree::GetName() const {
  return "Cfg_free_tree";
}

Vector3D Cfg_free_tree::GetRobotCenterPosition() const {
  return Vector3D(m_v[0], m_v[1], m_v[2]);
}

void Cfg_free_tree::GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb) {
  // this is not EXACTLY accurate, ok with most cases ... TO DO
  // To be accurate, one has to make sure every link is inside the given BB,
  // but here only the base link is taken care of. It is almost fine since
  // a little 'bigger' BB will contain all links.  

  m_v.clear();
  Point3d p = _bb->GetRandomPoint();
  for(size_t i=0 ;i<PosDOF();i++){
    m_v.push_back(p[i]);
  }

  for (size_t i = PosDOF() ; i < m_dof ; ++i)
    m_v.push_back(_bb->GetRandomValueInParameter(i-PosDOF()));
}

bool Cfg_free_tree::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
     
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 m_v[5]*TWOPI, 
						 m_v[4]*TWOPI, 
						 m_v[3]*TWOPI), // RPY
				     Vector3D(m_v[0],m_v[1],m_v[2]));
  
  _env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);  // update link 1.
  for(size_t i=0; i<m_numOfJoints; i++) {
    _env->GetMultiBody(robot)->GetFreeBody(i+1)
      ->GetBackwardConnection(0).GetDHparameters().theta = m_v[i+6]*TWOPI;
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

Vector3D 
Cfg_free_tree::GetRobotCenterofMass(Environment* _env) const {
  ConfigEnvironment(_env);

  Vector3D com(0,0,0);
  shared_ptr<MultiBody> mb =
    _env->GetMultiBody(env->GetRobotIndex());
  GMSPolyhedron poly = mb->GetFreeBody(0)->GetWorldPolyhedron();
  for(vector<Vector3D>::const_iterator vit = poly.m_vertexList.begin(); vit
      != poly.m_vertexList.end(); ++vit)
    com = com + (*vit);
  com = com / poly.m_vertexList.size();

  for(int i=0; i<m_numOfJoints; ++i) {
    GMSPolyhedron poly = mb->GetFreeBody(i)->GetWorldPolyhedron();
    Vector3D polycom(0,0,0);
    for(vector<Vector3D>::const_iterator vit = poly.m_vertexList.begin(); vit
        != poly.m_vertexList.end(); ++vit)
      polycom = polycom + (*vit);
    polycom = polycom / poly.m_vertexList.size();
    com = com + polycom;
  }
  com = com / (m_numOfJoints+1); //count the base

  return com;
}

