#include "Cfg_free_tree_2dof.h"
#include "MultiBody.h"
#include "Environment.h"

int Cfg_free_tree_2dof::m_numOfJoints;

Cfg_free_tree_2dof::Cfg_free_tree_2dof(){
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);
};

Cfg_free_tree_2dof::~Cfg_free_tree_2dof(){}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(int _numofJoints) {
  if(m_numOfJoints != _numofJoints ) {
    cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(int), cannot change numofJoints\n";
    exit(-1);
  }
  
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);
  
  NormalizeOrientation(); 
}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(double x, double y, double z, 
		   double roll, double pitch, double yaw) {
  cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(double,double,double,double,double,double), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(const Vector6D& _v) {
  cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(Vector6D), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(const vector<double> &_v){
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(vector<double>), ";
    cout << "size of vector less than m_dof\n";
    exit(-1);
  }
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();

};

Cfg_free_tree_2dof::Cfg_free_tree_2dof(const Cfg& _c) {
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  vector<double> _v;
  _v = _c.GetData();
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(Cfg&), ";
    cout << "size of cfg data less than m_dof\n";
    exit(-1);
  }
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
}


const char* Cfg_free_tree_2dof::GetName() const {
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
  for(int i=0; i<m_numOfJoints; i+=2) {
    _env->GetMultiBody(robot)->GetFreeBody(i/2+1)
      ->GetBackwardConnection(0).GetDHparameters().alpha = m_v[i+6]*360.0;
    _env->GetMultiBody(robot)->GetFreeBody(i/2+1)
      ->GetBackwardConnection(0).GetDHparameters().theta = m_v[i+1+6]*360.0;
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
