#include "Cfg_free_multi.h"

#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetricMethod.h"

int Cfg_free_multi::NumofRobots;

Cfg_free_multi::Cfg_free_multi() {
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);
}

Cfg_free_multi::~Cfg_free_multi() {}

Cfg_free_multi::Cfg_free_multi(const Cfg& _c) {
  vector<double> _v = _c.GetData();
  m_v.clear();
  m_v.insert(m_v.end(), _v.begin(), _v.begin()+m_dof);
  
  NormalizeOrientation();
}

vector<Robot> Cfg_free_multi::GetRobots(int _numRobots){
  vector<Robot> robots;
  Robot::JointMap joints;
  for(int i = 0; i<_numRobots; i++){
    robots.push_back(Robot(Robot::VOLUMETRIC, Robot::ROTATIONAL, joints, i));
  }
  return robots;
}

const string Cfg_free_multi::GetName() const {
  return "Cfg_free_multi";
}

Vector3D Cfg_free_multi::GetRobotCenterPosition() const {
  double x = 0;
  double y = 0;
  double z = 0;

  for(int i=0; i<NumofRobots; ++i) {
    x += m_v[i*6+0];
    y += m_v[i*6+1];
    z += m_v[i*6+2];
  }

  return Vector3D(x/NumofRobots, y/NumofRobots, z/NumofRobots);
}


bool Cfg_free_multi::ConfigEnvironment(Environment* env) const {
  shared_ptr<MultiBody> mb = env->GetMultiBody(env->GetRobotIndex());
 
  for(int i=0; i<NumofRobots; ++i) {
    // configure the robot according to current Cfg: joint parameters
    // (and base locations/orientations for free flying robots.)
    Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						   m_v[i*6+5]*TWOPI, 
						   m_v[i*6+4]*TWOPI, 
						   m_v[i*6+3]*TWOPI),
				       Vector3D(m_v[i*6+0],m_v[i*6+1],m_v[i*6+2]));
    // update link i
    mb->GetFreeBody(i)->Configure(T1);
  }
  
  return true;
}

void Cfg_free_multi::GetRandomCfg(double _r, double _rStep) {
  m_v = vector<double>(m_dof, 0);
  
  for(int i=0; i<NumofRobots; ++i) {
    double alpha = 2.0*M_PI*DRand();
    double beta  = 2.0*M_PI*DRand();
    double z = _r*cos(beta);
    double z1 = _r*sin(beta);

    double roll = (2.0*_rStep)*DRand() - _rStep;
    double pitch = (2.0*_rStep)*DRand() - _rStep;
    double yaw = (2.0*_rStep)*DRand() - _rStep;

    m_v[i*3+0] = z1*cos(alpha);
    m_v[i*3+1] = z1*sin(alpha);
    m_v[i*3+2] = z;
    m_v[NumofRobots*3+i*3+0] = roll;
    m_v[NumofRobots*3+i*3+1] = pitch;
    m_v[NumofRobots*3+i*3+2] = yaw;
  }
}


void Cfg_free_multi::GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb) {
  Cfg::GetRandomCfg(_env,_bb);
}

void Cfg_free_multi::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundingBox());
}

void
Cfg_free_multi::GetRandomCfg_CenterOfMass(Environment *_env, shared_ptr<Boundary> _bb) {
  m_v = vector<double>(m_dof, 0);
  
  for(int r=0; r<NumofRobots; ++r) {
    Point3d p;
    if(BoundingBox* bbox = dynamic_cast<BoundingBox*>(_bb.get()))
    {
      BoundingBox singleRobotBB(3, 3);
      for(int i=0; i<3; ++i) {
        pair<double, double> range = bbox->GetRange(r*3+i);
        singleRobotBB.SetParameter(i, range.first, range.second);
      }
      p = singleRobotBB.GetRandomPoint();
    } else {
      if(r > 0)
        cerr << "WARNING:: In Cfg_free_multi::GetRandomCfg_CenterOfMass: using boundary type other than bounding box, GetRandomPoint() may be incorrect for robot " << r << ".\n";
      p = _bb->GetRandomPoint();
    }
    for(int i=0; i<3; ++i)
      m_v[r*3+i] = p[i];
  }

  for(int i=NumofRobots*3; i<m_dof; ++i)
    m_v[i] = _bb->GetRandomValueInParameter(i-NumofRobots*3);
}

void Cfg_free_multi::GetRandomCfg_CenterOfMass(Environment *_env) {
  GetRandomCfg_CenterOfMass(_env, _env->GetBoundingBox());
}

