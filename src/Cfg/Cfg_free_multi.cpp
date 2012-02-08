#include "Cfg_free_multi.h"

#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetricMethod.h"

int Cfg_free_multi::NumofRobots;

Cfg_free_multi::Cfg_free_multi() {
  m_dof = 6*NumofRobots;
  m_posDof = m_dof/2;

  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);
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
  m_dof = 6*NumofRobots;
  m_posDof = m_dof/2;
  
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free_multi::Cfg_free_multi(const vector<double>&), ";
    cout << "size of vector is less than " << m_dof << endl;
    exit(-1);
  }
  m_v.clear();
  m_v.insert(m_v.end(), _v.begin(), _v.begin()+m_dof);
  
  NormalizeOrientation();
}

Cfg_free_multi::Cfg_free_multi(const Cfg& _c) {
  m_dof = 6*NumofRobots;
  m_posDof = m_dof/2;
  
  vector<double> _v = _c.GetData();
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free_multi::Cfg_free_multi(const vector<double>&), ";
    cout << "size of vector is less than " << m_dof << endl;
    exit(-1);
  }
  m_v.clear();
  m_v.insert(m_v.end(), _v.begin(), _v.begin()+m_dof);
  
  NormalizeOrientation();
}


const char* Cfg_free_multi::GetName() const {
  return "Cfg_free_multi";
}

Vector3D Cfg_free_multi::GetRobotCenterPosition() const {
  double x = 0;
  double y = 0;
  double z = 0;

  for(int i=0; i<NumofRobots; ++i) {
    x += m_v[i*3+0];
    y += m_v[i*3+1];
    z += m_v[i*3+2];
  }

  return Vector3D(x/NumofRobots, y/NumofRobots, z/NumofRobots);
}


bool Cfg_free_multi::ConfigEnvironment(Environment* env) const {
  shared_ptr<MultiBody> mb = env->GetMultiBody(env->GetRobotIndex());
 
  for(int i=0; i<NumofRobots; ++i) {
    // configure the robot according to current Cfg: joint parameters
    // (and base locations/orientations for free flying robots.)
    Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						   m_v[m_posDof+i*3+2]*TWOPI, 
						   m_v[m_posDof+i*3+1]*TWOPI, 
						   m_v[m_posDof+i*3+0]*TWOPI),
				       Vector3D(m_v[i*3+0],m_v[i*3+1],m_v[i*3+2]));
    // update link i
    mb->GetFreeBody(i)->Configure(T1);
  }
  
  return true;
}


void Cfg_free_multi::GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb) {
  Cfg::GetRandomCfg(_env,_bb);
}

void Cfg_free_multi::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundingBox());
}

