#include "Cfg_free_multi.h"

#include "MultiBody.h"
#include "Environment.h"
#include "util.h"
#include "DistanceMetricMethod.h"

int Cfg_free_multi::NumofRobots;

Cfg_free_multi::Cfg_free_multi() {
  dof = 6*NumofRobots;
  posDof = dof/2;

  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(0);

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_free_multi::~Cfg_free_multi() {}


Cfg_free_multi::Cfg_free_multi(double x, double y, double z, 
		   double roll, double pitch, double yaw) {
  cout << "\n\nERROR in Cfg_free_multi::Cfg_free_multi(double, double, double, double, double, double), not applicable\n";
  exit(-1);
}


Cfg_free_multi::Cfg_free_multi(const Vector6D& _v) {
  cout << "\n\nERROR in Cfg_free_multi::Cfg_free_multi(const Vector6D), not applicable\n";
  exit(-1);
}


Cfg_free_multi::Cfg_free_multi(const vector<double>& _v) {
  dof = 6*NumofRobots;
  posDof = dof/2;
  
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free_multi::Cfg_free_multi(const vector<double>&), ";
    cout << "size of vector is less than " << dof << endl;
    exit(-1);
  }
  v.clear();
  v.insert(v.end(), _v.begin(), _v.begin()+dof);
  
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_free_multi::Cfg_free_multi(const Cfg& _c) {
  dof = 6*NumofRobots;
  posDof = dof/2;
  
  vector<double> _v = _c.GetData();
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free_multi::Cfg_free_multi(const vector<double>&), ";
    cout << "size of vector is less than " << dof << endl;
    exit(-1);
  }
  v.clear();
  v.insert(v.end(), _v.begin(), _v.begin()+dof);
  
  Normalize_orientation();
 
  obst = _c.obst;
  tag = _c.tag;
  clearance = _c.clearance;
}


const char* Cfg_free_multi::GetName() const {
  return "Cfg_free_multi";
}


Cfg* Cfg_free_multi::CreateNewCfg() const {
  Cfg* tmp = new Cfg_free_multi();
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_free_multi::CreateNewCfg(vector<double>& data) const {
  Vector6D _data;
  if((int)data.size() < dof) {
    cout << "\n\nERROR in Cfg_free_multi::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than " << dof << endl;
    exit(-1);
  }
  for(int i=0; i<dof; i++)
    _data[i] = data[i];
  Cfg* tmp = new Cfg_free_multi(_data);
  return tmp;
}


Vector3D Cfg_free_multi::GetRobotCenterPosition() const {
  double x = 0;
  double y = 0;
  double z = 0;

  for(int i=0; i<NumofRobots; ++i) {
    x += v[i*3+0];
    y += v[i*3+1];
    z += v[i*3+2];
  }

  return Vector3D(x/NumofRobots, y/NumofRobots, z/NumofRobots);
}


bool Cfg_free_multi::ConfigEnvironment(Environment* env) const {
  shared_ptr<MultiBody> mb = env->GetMultiBody(env->GetRobotIndex());
 
  for(int i=0; i<NumofRobots; ++i) {
    // configure the robot according to current Cfg: joint parameters
    // (and base locations/orientations for free flying robots.)
    Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						   v[posDof+i*3+2]*TWOPI, 
						   v[posDof+i*3+1]*TWOPI, 
						   v[posDof+i*3+0]*TWOPI),
				       Vector3D(v[i*3+0],v[i*3+1],v[i*3+2]));
    // update link i
    mb->GetFreeBody(i)->Configure(T1);
  }
  
  return true;
}


void Cfg_free_multi::GetRandomCfg(Environment* env) {
  Cfg::GetRandomCfg(env);
}

