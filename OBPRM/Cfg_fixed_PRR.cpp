// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg_fixed_PRR.c
//
//  General Description
//      A derived class from Cfg. It provides some specific
//      implementation closely related to a PRR robot.
//      Degree of Freedom: 3
//
//  Created
//      08/31/99        Guang Song
//
//  Last Modified
//
/////////////////////////////////////////////////////////////////////

#include "Cfg_fixed_PRR.h"

#include "MultiBody.h"
#include "Environment.h"
#include "util.h"


Cfg_fixed_PRR::Cfg_fixed_PRR() {
  dof = 3;
  posDof = 1;
  v.clear();
  v.push_back(0);
  v.push_back(0);
  v.push_back(0);

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_fixed_PRR::Cfg_fixed_PRR(double x, double y, double z,
                   double roll, double pitch, double yaw) {
  cout<<"\n\nError in Cfg_fixed_PRR::Cfg_fixed_PRR(double,double,double,double,double,double), not implement yet\n";
  exit(-1);
}


Cfg_fixed_PRR::Cfg_fixed_PRR(double zz, double ceta1, double ceta2) {
  dof = 3;
  posDof = 1;
  v.clear();
  v.push_back(zz);
  v.push_back(ceta1);
  v.push_back(ceta2);
  
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_fixed_PRR::Cfg_fixed_PRR(const Vector3<double>& _v) {
  dof = 3;
  posDof = 1;
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_v[i]);
  
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_fixed_PRR::Cfg_fixed_PRR(const Cfg& _c) {
  dof = 3;
  posDof = 1;
  vector<double> _v;
  _v = _c.GetData();
  if(_v.size() < dof) {
    cout << "\n\nERROR in Cfg_fixed_PRR::Cfg_fixed_PRR(Cfg&), ";
    cout << "size of cfg data is less than 3\n";
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
		    

Cfg_fixed_PRR::~Cfg_fixed_PRR() {}


void Cfg_fixed_PRR::equals(const Cfg& c) {
  dof = c.DOF();
  posDof = c.posDOF();
  v.clear();
  v = c.GetData();
  obst = c.obst;
  tag = c.tag;
  clearance = c.clearance;
}
	

Vector3D Cfg_fixed_PRR::GetRobotCenterPosition() const {
   return Vector3D(0, 0, v[0]);
}


const char* Cfg_fixed_PRR::GetName() const {
  return "Cfg_fixed_PRR";
}

  
void Cfg_fixed_PRR::GetRandomCfg(double R, double rStep){
  double zz;
  if(OBPRM_drand() > 0.5)
    zz = R;
  else
    zz = -R;
  
  double ceta1, ceta2;
  ceta1 = (2.0*rStep)*OBPRM_drand() - rStep;
  ceta2 = (2.0*rStep)*OBPRM_drand() - rStep;
  
  v.clear();
  v.push_back(zz);
  v.push_back(ceta1);
  v.push_back(ceta2);
  
  obst = -1;
  tag = -1;
  clearance = -1;
}


void Cfg_fixed_PRR::GetRandomRay(double incr) {
  double alpha,beta,z, z1;
  
  alpha = 2.0*M_PI*OBPRM_drand();
  beta  = 2.0*M_PI*OBPRM_drand();
  z = incr*cos(beta);
  z1 = incr*sin(beta);
   
  v.clear();
  v.push_back(z1*cos(alpha));
  v.push_back(z1*sin(alpha));
  v.push_back(z);
  
  obst = -1;
  tag = -1;
  clearance = -1;
}


void Cfg_fixed_PRR::GetRandomCfg(Environment* env) {
  Cfg::GetRandomCfg(env);
}
  
void Cfg_fixed_PRR::GetRandomCfg_CenterOfMass(Environment *env) {
  double *boundingBox = env->GetBoundingBox();
  double zz = boundingBox[4] +
    (boundingBox[5]-boundingBox[4])*OBPRM_drand();
  
  double ceta1 =1.0 * OBPRM_drand();
  double ceta2 =1.0 * OBPRM_drand();
  
  v.clear();
  v.push_back(zz);
  v.push_back(ceta1);
  v.push_back(ceta2);
  
  obst = -1;
  tag = -1;
  clearance = -1;
}


bool Cfg_fixed_PRR::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
  
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  double zz = v[0];
  double ceta1 = v[1]*360.0;
  double ceta2 = v[2]*360.0;
  _env->GetMultiBody(robot)->GetFreeBody(0)
    ->GetBackwardConnection(0)->GetDHparameters().theta = ceta1;
  _env->GetMultiBody(robot)->GetFreeBody(0)
    ->GetBackwardConnection(0)->GetDHparameters().d = zz;
  _env->GetMultiBody(robot)->GetFreeBody(0)
    ->GetForwardConnection(0)->GetDHparameters().theta = ceta2;
  
  // calculate WorldTransformation recursively from the very last link.
  _env->GetMultiBody(robot)->GetFreeBody(1)->GetWorldTransformation();
  return true;
}


bool Cfg_fixed_PRR::GenerateOverlapCfg(Environment *env,  // although env and robot is not used here,
				       int robot,            // they are needed in other Cfg classes.
				       Vector3D robot_start,
				       Vector3D robot_goal,
				       Cfg *resultCfg){

  // formulas here is from Craig book P128 with a little modifications.
  double x,y,a,b,r,L1,c2,s2; // known variables
  double  ceta1,ceta2, zz; // unknown ones.
  
  x = robot_goal[0];  // world frame
  y = robot_goal[1];
  a = robot_start[0]; // robot frame
  b = robot_start[1];
  r = sqrt(a*a+b*b);
  L1 = env->GetMultiBody(robot)->GetFreeBody(1)
    ->GetBackwardConnection(0)->GetDHparameters().a;
  c2 = ((x*x+y*y)-L1*L1-r*r)/(2.*L1*r);
  if(c2 >= 1.0 || c2 <= -1.0 ) return false;
  s2 = sqrt(1.-c2*c2);
  
  // find solution to this inverse kinematics problem.
  ceta1 = atan2(y,x) - atan2(r*s2, L1+r*c2);
  ceta2 = atan2(s2,c2) - atan2(b,a);
  zz = robot_goal[2];
  
  // pass back the Cfg for this pose.
  *resultCfg = Cfg_fixed_PRR(zz, ceta1/6.2832, ceta2/6.2832); // normalize to [0, 1]
  return true;
}


//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
//===================================================================
void Cfg_fixed_PRR::GenSurfaceCfgs4ObstNORMAL(Environment * env, 
					      Stat_Class& Stats, 
					      CollisionDetection* cd, 
					      int obstacle, int nCfgs, 
					      CDInfo& _cdInfo, 
					      vector<Cfg*>& surface) const {
  cout << "Error in Cfg_fixed_PRR::GenSurfaceCfgs4ObstNORMAL(), not implemented yet" << endl;
  exit(10);
}


Cfg* Cfg_fixed_PRR::CreateNewCfg() const {
  Cfg* tmp = new Cfg_fixed_PRR();
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_fixed_PRR::CreateNewCfg(vector<double>& data) const {
  Vector3<double> _data;
  if(data.size() < 3) {
    cout << "\n\nERROR in Cfg_fixed_PRR::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than 3\n";
    exit(-1);
  }
  for(int i=0; i<3; i++)
    _data[i] = data[i];
  Cfg* tmp = new Cfg_fixed_PRR(_data);
  return tmp;
}
