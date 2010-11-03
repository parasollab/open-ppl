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

#include "Cfg.h"
#include "Cfg_free.h"
#include "MultiBody.h"
#include "Environment.h"
#include "util.h"
#include "DistanceMetricMethod.h"

int Cfg_free_tree::NumofJoints;

Cfg_free_tree::Cfg_free_tree(){
  dof = 6 + NumofJoints;
  posDof = 3;
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(0);

  obst = -1;
  tag = -1;
  clearance = -1;
};

Cfg_free_tree::~Cfg_free_tree(){}

Cfg_free_tree::Cfg_free_tree(int _numofJoints) {
  if(NumofJoints != _numofJoints ) {
    cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(int), cannot change numofJoints\n";
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

Cfg_free_tree::Cfg_free_tree(double x, double y, double z, 
		   double roll, double pitch, double yaw) {
  cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(double,double,double,double,double,double), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree::Cfg_free_tree(const Vector6D& _v) {
  cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(Vector6D), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree::Cfg_free_tree(const vector<double> &_v){
  dof = 6 + NumofJoints;
  posDof = 3;
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(vector<double>), ";
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

Cfg_free_tree::Cfg_free_tree(const Cfg& _c) {
  dof = 6 + NumofJoints;
  posDof = 3;
  vector<double> _v;
  _v = _c.GetData();
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(Cfg&), ";
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

void Cfg_free_tree::GetRandomCfg(Environment* env) {
  Cfg::GetRandomCfg(env);
}

void Cfg_free_tree::equals(const Cfg& c) {
  vector<double> _v;
  _v = c.GetData();
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free_tree::equals(Cfg&), ";
    cout << "size of cfg data less than dof\n";
    exit(-1);
  }
  v.clear();
  for (int i = 0; i < dof; i ++)
    v.push_back(_v[i]);

  obst = c.obst;
  tag = c.tag;
  clearance = c.clearance;
}

const char* Cfg_free_tree::GetName() const {
  return "Cfg_free_tree";
}

Cfg* Cfg_free_tree::CreateNewCfg() const {
  Cfg* tmp = new Cfg_free_tree();
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_free_tree::CreateNewCfg(vector<double>& data) const {
  vector<double> _data;
  if((int)data.size() < dof) {
    cout << "\n\nERROR in Cfg_free_tree::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than dof\n";
    exit(-1);
  }
  for(int i=0; i<dof; i++)
    _data.push_back(data[i]);
  Cfg* tmp = new Cfg_free_tree(_data);
  return tmp;
}



Vector3D Cfg_free_tree::GetRobotCenterPosition() const {
  return Vector3D(v[0], v[1], v[2]);
}


Vector3D Cfg_free_tree::GetRobotCenterofMass(Environment* env) const {
  ConfigEnvironment(env);

  Vector3D com(0,0,0);
  shared_ptr<MultiBody> mb = env->GetMultiBody(env->GetRobotIndex());
  for(int i=0; i<=NumofJoints; ++i) {
    GMSPolyhedron poly = mb->GetFreeBody(i)->GetWorldPolyhedron();
    Vector3D poly_com(0,0,0);
    for(vector<Vector3D>::const_iterator V = poly.vertexList.begin(); V != poly.vertexList.end(); ++V)
      poly_com = poly_com + (*V);
    poly_com = poly_com / poly.vertexList.size();
    com = com + poly_com;
  }
  com = com / (NumofJoints+1);

  return com;
}


void Cfg_free_tree::GetRandomCfg(double R, double rStep){
  double alpha,beta,z, z1;
  double jointAngle;
  
  alpha = 2.0*M_PI*OBPRM_drand();
  beta  = 2.0*M_PI*OBPRM_drand();
  z = R*cos(beta);
  z1 = R*sin(beta);

  double roll, pitch, yaw;
  roll = (2.0*rStep)*OBPRM_drand() - rStep;
  pitch = (2.0*rStep)*OBPRM_drand() - rStep;
  yaw = (2.0*rStep)*OBPRM_drand() - rStep;
  
  Vector6D base(z1*cos(alpha),z1*sin(alpha),z,roll,pitch,yaw);
  
  int i;
  v.clear();
  for( i=0; i<6; ++i)
    v.push_back(base[i]);
  for(i=0; i<NumofJoints; i++) {
    jointAngle = (2.0*rStep)*OBPRM_drand() - rStep;
    // or: jointAngle = 0.0; I am not sure which is more reasonable now. Guang
    v.push_back(jointAngle);
  }
  
  obst = -1;
  tag = -1;
  clearance = -1;
}

void Cfg_free_tree::GetRandomCfg_CenterOfMass(Environment *env) {
  // this is not EXACTLY accurate, ok with most cases ... TO DO
  // To be accurate, one has to make sure every link is inside the given BB,
  // but here only the base link is taken care of. It is almost fine since
  // a little 'bigger' BB will contain all links.
  
  BoundingBox* boundingBox = env->GetBoundingBox();
  v.clear();
  for (int i = 0 ; i < dof ; ++i)
    v.push_back(boundingBox->GetRandomValueInParameter(i));
  
  obst = -1;
  tag = -1;
  clearance = -1;
}


bool Cfg_free_tree::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
     
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 v[5]*TWOPI, 
						 v[4]*TWOPI, 
						 v[3]*TWOPI), // RPY
				     Vector3D(v[0],v[1],v[2]));
  
  _env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);  // update link 1.
  int i;
  for( i=0; i<NumofJoints; i++) {
    _env->GetMultiBody(robot)->GetFreeBody(i+1)
      ->GetBackwardConnection(0).GetDHparameters().theta = v[i+6]*360.0;
  }  // config the robot
  
  
  for(i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); i++) {
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

bool 
Cfg_free_tree::
GenerateOverlapCfg(Environment *env,
		   int robot, //not used here, needed in other Cfg classes
		   Vector3D robot_start, Vector3D robot_goal, 
		   Cfg *resultCfg){
  int i;
  Vector3D diff = robot_goal - robot_start;

  BoundingBox *bbox = env->GetBoundingBox();

  vector<double> result;
  for(i=0; i<3; ++i)
    result.push_back(diff[i]);
  for(i=3; i<dof; ++i)
    result.push_back(bbox->GetRandomValueInParameter(i));

  *resultCfg = Cfg_free_tree(result);
  return true;
}


//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
//===================================================================
void Cfg_free_tree::GenSurfaceCfgs4ObstNORMAL
(Environment * env, Stat_Class& Stats, CollisionDetection* cd, int obstacle, int nCfgs, 
CDInfo& _cdInfo, vector<Cfg*>& surface){
  surface.clear();
  static const int SIZE = 1;
  //static double jointAngles[SIZE][3] = {{0.0, 0.0, 0.0}, {0.25, 0.25, 0.25}, {0.0, 0.4, 0.0},
  //                                      {0.4, 0.6, 0.4},};
  std::string Callee(GetName());
  {std::string Method("-cfg_free_tree::GenSurfaceCfg4ObstNORMAL");  Callee = Callee+Method;}

  int robot = env->GetRobotIndex();
  GMSPolyhedron &polyRobot = env->GetMultiBody(robot)->GetFreeBody(0)->GetPolyhedron();
  GMSPolyhedron &polyObst = env->GetMultiBody(obstacle)->GetFixedBody(0)
    ->GetWorldPolyhedron();
  int num = 0;
//   int tries = 3 * num;
//   int i = 0;

  shared_ptr<MultiBody> base(new MultiBody());
  base->AddBody(env->GetMultiBody(robot)->GetFreeBody(0));
  
  while(num < nCfgs) {
    int robotTriIndex = (int)(OBPRM_drand()*polyRobot.polygonList.size());
    int obstTriIndex = (int)(OBPRM_drand()*polyObst.polygonList.size());
    vector<Cfg*> tmp;
    GetCfgByOverlappingNormal(env, Stats, cd, 
			      polyRobot, polyObst, 
			      robotTriIndex, obstTriIndex, 
			      _cdInfo,
			      base, tmp);
    if(!tmp.empty()) {
      vector<double> basePose = tmp[0]->GetData();
      for(int j=0; j<SIZE; ++j) {
	vector<double> serialData = basePose;  // for clearness, have basePose tmp variable.
	for(int i=0; i<NumofJoints; ++i) {  // now add joint angles.
	  serialData.push_back(OBPRM_drand());
	}
	Cfg* serial = this->CreateNewCfg(serialData);
	if(!serial->isCollision(env,Stats,cd,_cdInfo) && serial->InBoundingBox(env)) {
	  
	  surface.push_back(serial);
	  ++num;
	}
	else
	  delete serial;
        for(vector<Cfg*>::iterator I = tmp.begin(); I != tmp.end(); ++I)
          delete *I;
      }
    }
    //i++;
  }
}


