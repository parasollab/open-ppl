/////////////////////////////////////////////////////////////////////
/**@file  Cfg_fixed_tree.cpp
  *
  * General Description
  *	A derived template class from Cfg. It provides some 
  *	specific implementation directly related to a multiple joints
  *	serial robot.
  *
  * Created
  * @date08/31/99	
  * @author Guang Song
  *
  * Last Modified
  */
/////////////////////////////////////////////////////////////////////


#include "Cfg_fixed_tree.h"
#include "MultiBody.h"
#include "Environment.h"

size_t Cfg_fixed_tree::m_numOfJoints;

Cfg_fixed_tree::Cfg_fixed_tree() {
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
      m_v.push_back(0);

  NormalizeOrientation();
}

Cfg_fixed_tree::Cfg_fixed_tree(const Cfg& _c) { 
  vector<double> _v;
  _v = _c.GetData();
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
} 

Cfg_fixed_tree::Cfg_fixed_tree(const Cfg_fixed_tree& _c) { 
  vector<double> _v;
  _v = _c.GetData();
  m_dof = _v.size();
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
} 

Cfg_fixed_tree::~Cfg_fixed_tree() {}

vector<Robot> Cfg_fixed_tree::GetRobots(int _numJoints){
  m_numOfJoints = _numJoints;
  vector<Robot> robots;
  Robot::JointMap joints;
  for(int i = 0; i<_numJoints; i++){
    joints.push_back(make_pair(make_pair(i, i+1), Robot::REVOLUTE));
  }
  robots.push_back(Robot(Robot::FIXED, Robot::ROTATIONAL, joints, 0));
  return robots;
}
	
Vector3D Cfg_fixed_tree::GetRobotCenterPosition() const {
  return Vector3D(0, 0, 0);
}


const string Cfg_fixed_tree::GetName() const {
  return "Cfg_fixed_tree";
}

void Cfg_fixed_tree::GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb) {
  // Why following comments are here? This method suppose will generate
  // Cfg whose center of mass will inside a given bounding box....
  
  // this is not EXACTLY accurate, ok with most cases ... TO DO
  // To be accurate, one has to make sure every link is inside the given BB,
  // but here only the base link is taken care of. It is almost fine since
  // a little 'bigger' BB will contain all links. 
  m_v.clear();
  for(size_t i=0; i<m_dof; ++i) 
    m_v.push_back(_bb->GetRandomValueInParameter(i));
}

void Cfg_fixed_tree::GetMovingSequenceNodes(const Cfg& other, vector<double> s, vector<Cfg*>& result) const {
  Cfg* c1 = this->CreateNewCfg();
  result.push_back(c1);
  
  vector<double> _data,_data2;
  for(size_t i=0; i<m_dof; i++) {
    if(i<2)
      _data.push_back(this->GetData()[i]);
    else
      _data.push_back(other.GetData()[i]);
  }
  Cfg* tmp = Cfg::CreateNewCfg(_data);
  result.push_back(tmp);
  
  _data2 = other.GetData();
  Cfg* c2 = Cfg::CreateNewCfg(_data2);
  result.push_back(c2);
}

Cfg*
Cfg_fixed_tree::CreateNewCfg() const {
  Cfg* tmp = new Cfg_fixed_tree();
  *tmp = *this;
  return tmp;
}

bool Cfg_fixed_tree::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
  
  size_t i;
  for(i=0; i<m_numOfJoints; i++) {
    _env->GetMultiBody(robot)->GetFreeBody(i)
      ->GetBackwardConnection(0).GetDHparameters().theta = m_v[i]*TWOPI;
  }  // config the robot
  
  for(i=0; i<m_numOfJoints; i++) {
    shared_ptr<FreeBody> afb = _env->GetMultiBody(robot)->GetFreeBody(i);
    if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
      afb->GetWorldTransformation();
  }
  
  // since Transformation is calculated in recursive manner, only
  // let the last links(or Freebody) call getWorldTransformation will
  // automatically calculate the transformations for all previous links.
  
  // when all worldTransformations are recalculated by using new cfg, the
  // config of the whole robot is updated.
  
  return true;
}


Vector3D
Cfg_fixed_tree::GetRobotCenterofMass(Environment* _env) const {
  ConfigEnvironment(_env);

  Vector3D com(0,0,0);
  shared_ptr<MultiBody> mb = env->GetMultiBody(_env->GetRobotIndex());
  for(int i=0; i<m_numOfJoints; ++i) {
    GMSPolyhedron poly = mb->GetFreeBody(i)->GetWorldPolyhedron();
    Vector3D polycom(0,0,0);
    for(vector<Vector3D>::const_iterator vit = poly.m_vertexList.begin(); vit != poly.m_vertexList.end(); ++vit)
      polycom = polycom + (*vit);
    polycom = polycom / poly.m_vertexList.size();
    com = com + polycom;
  }
  com = com / (m_numOfJoints);

  return com;
}

