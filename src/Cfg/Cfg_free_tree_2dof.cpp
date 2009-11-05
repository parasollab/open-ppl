#include "Cfg_free_tree_2dof.h"

#include "Cfg.h"
#include "Cfg_free.h"
#include "MultiBody.h"
#include "Environment.h"
#include "util.h"
#include "DistanceMetrics.h"

int Cfg_free_tree_2dof::NumofJoints;

Cfg_free_tree_2dof::Cfg_free_tree_2dof(){
  dof = 6 + NumofJoints;
  posDof = 3;
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(0);

  obst = -1;
  tag = -1;
  clearance = -1;
};

Cfg_free_tree_2dof::~Cfg_free_tree_2dof(){}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(int _numofJoints) {
  if(NumofJoints != _numofJoints ) {
    cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(int), cannot change numofJoints\n";
    exit(-1);
  }
  
  dof = 6 + NumofJoints;
  posDof = 3;
  
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(0);
  
  Normalize_orientation(); 

  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(double x, double y, double z, 
		   double roll, double pitch, double yaw) {
  cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(double,double,double,double,double,double), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(const Vector6<double>& _v) {
  cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(Vector6<double>), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree_2dof::Cfg_free_tree_2dof(const vector<double> &_v){
  dof = 6 + NumofJoints;
  posDof = 3;
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(vector<double>), ";
    cout << "size of vector less than dof\n";
    exit(-1);
  }
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_v[i]);
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;

};

Cfg_free_tree_2dof::Cfg_free_tree_2dof(const Cfg& _c) {
  dof = 6 + NumofJoints;
  posDof = 3;
  vector<double> _v;
  _v = _c.GetData();
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free_tree_2dof::Cfg_free_tree_2dof(Cfg&), ";
    cout << "size of cfg data less than dof\n";
    exit(-1);
  }
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_v[i]);
  Normalize_orientation();

  obst = _c.obst;
  tag = _c.tag;
  clearance = _c.clearance;
}


const char* Cfg_free_tree_2dof::GetName() const {
  return "Cfg_free_tree_2dof";
}


Cfg* Cfg_free_tree_2dof::CreateNewCfg() const {
  Cfg* tmp = new Cfg_free_tree_2dof();
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_free_tree_2dof::CreateNewCfg(vector<double>& data) const {
  vector<double> _data;
  if((int)data.size() < dof) {
    cout << "\n\nERROR in Cfg_free_tree_2dof::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than dof\n";
    exit(-1);
  }
  for(int i=0; i<dof; i++)
    _data.push_back(data[i]);
  Cfg* tmp = new Cfg_free_tree_2dof(_data);
  return tmp;
}


bool Cfg_free_tree_2dof::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
     
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 v[5]*TWOPI, 
						 v[4]*TWOPI, 
						 v[3]*TWOPI), // RPY
				     Vector3D(v[0],v[1],v[2]));
  
  _env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);  // update link 1.
  for(int i=0; i<NumofJoints; i+=2) {
    _env->GetMultiBody(robot)->GetFreeBody(i/2+1)
      ->GetBackwardConnection(0).GetDHparameters().alpha = v[i+6]*360.0;
    _env->GetMultiBody(robot)->GetFreeBody(i/2+1)
      ->GetBackwardConnection(0).GetDHparameters().theta = v[i+1+6]*360.0;
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
