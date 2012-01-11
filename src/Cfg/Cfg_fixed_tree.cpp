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
#include "DistanceMetricMethod.h"

int Cfg_fixed_tree::m_numOfJoints;

Cfg_fixed_tree::Cfg_fixed_tree() {
  m_dof = m_numOfJoints;
  m_posDof = 0;

  m_v.clear();
  for(int i=0; i<m_dof; i++)
      m_v.push_back(0);

  NormalizeOrientation();
}

Cfg_fixed_tree::Cfg_fixed_tree(double x, double y, double z,
                               double roll, double pitch, double yaw) {
  cout << "\n\nERROR in Cfg_fixed_tree::Cfg_fixed_tree(double,double,double,double,double,double), not implement yet.\n";
  exit(-1);
}
		  
Cfg_fixed_tree::Cfg_fixed_tree(int _numofJoints) {
  if (m_numOfJoints != _numofJoints ) {
    cout << "ERROR in Cfg_fixed_tree::Cfg_fixed_tree(int), cannot change numofJoints\n";
    exit(-1);
  }
    
  m_dof = m_numOfJoints;
  m_posDof = 0;
  
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);
  
  NormalizeOrientation();
}


Cfg_fixed_tree::Cfg_fixed_tree(const vector<double>& _data) {
  m_dof = m_numOfJoints;
  m_posDof = 0;
  if((int)_data.size() < m_dof) {
    cout << "\n\nERROR in Cfg_fixed_tree::Cfg_fixed_tree(vector<double>), ";
    cout << "size of vector is less than m_dof\n";
    exit(-1);
  }
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_data[i]);
  
  NormalizeOrientation();
}


Cfg_fixed_tree::Cfg_fixed_tree(const Cfg& _c) { 
  vector<double> _v;
  _v = _c.GetData();
  m_dof = _v.size();
  m_posDof = 0;
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
} 

Cfg_fixed_tree::~Cfg_fixed_tree() {}
	
Vector3D Cfg_fixed_tree::GetRobotCenterPosition() const {
  return Vector3D(0, 0, 0);
}


const char* Cfg_fixed_tree::GetName() const {
  return "Cfg_fixed_tree";
}
  
void Cfg_fixed_tree::GetRandomCfg(double R, double rStep){
  m_v.clear();
  
  for(int i=0; i<m_numOfJoints; i++) {
    double jointAngle = (2.0*rStep)*DRand() - rStep;
    m_v.push_back(jointAngle);
  }
}


void Cfg_fixed_tree::GetRandomCfg(Environment* _env,shared_ptr<BoundingBox> _bb) {
    Cfg::GetRandomCfg(_env,_bb);
}

void Cfg_fixed_tree::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundingBox());
}


void Cfg_fixed_tree::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
  //randomly sample params
  m_v.clear();
  for(int i=0; i<m_dof; ++i)
    m_v.push_back( double(2.0)*DRand() - double(1.0) );
  
  //scale to appropriate length
  Cfg_fixed_tree origin;
  dm->ScaleCfg(env, incr, origin, *this);
  
  NormalizeOrientation();
}

void Cfg_fixed_tree::GetRandomCfg_CenterOfMass(Environment *_env,shared_ptr<BoundingBox> _bb) {
  // Why following comments are here? This method suppose will generate
  // Cfg whose center of mass will inside a given bounding box....
  
  // this is not EXACTLY accurate, ok with most cases ... TO DO
  // To be accurate, one has to make sure every link is inside the given BB,
  // but here only the base link is taken care of. It is almost fine since
  // a little 'bigger' BB will contain all links. 
  m_v.clear();
  for(int i=0; i<m_dof; ++i) 
    m_v.push_back(_bb->GetRandomValueInParameter(i));
}

void Cfg_fixed_tree::GetRandomCfg_CenterOfMass(Environment *_env) {
  GetRandomCfg_CenterOfMass(_env, _env->GetBoundingBox());
}

void Cfg_fixed_tree::GetMovingSequenceNodes(const Cfg& other, vector<double> s, vector<Cfg*>& result) const {
  Cfg* c1 = this->CreateNewCfg();
  result.push_back(c1);
  
  vector<double> _data,_data2;
  for(int i=0; i<m_dof; i++) {
    if(i<2)
      _data.push_back(this->GetData()[i]);
    else
      _data.push_back(other.GetData()[i]);
  }
  Cfg* tmp = new Cfg_fixed_tree(_data);
  result.push_back(tmp);
  
  _data2 = other.GetData();
  Cfg* c2 = new Cfg_fixed_tree(_data2);
  result.push_back(c2);
}


bool Cfg_fixed_tree::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
  
  int i;
  for(i=0; i<m_numOfJoints; i++) {
    _env->GetMultiBody(robot)->GetFreeBody(i)
      ->GetBackwardConnection(0).GetDHparameters().theta = m_v[i]*360.0;
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

