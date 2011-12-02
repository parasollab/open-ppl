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
#include "MPProblem.h"
#include "ValidityChecker.hpp"


int Cfg_fixed_tree::NumofJoints;

Cfg_fixed_tree::Cfg_fixed_tree() {
  dof = NumofJoints;
  posDof = 0;

  v.clear();
  for(int i=0; i<dof; i++)
      v.push_back(0);

  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_fixed_tree::Cfg_fixed_tree(double x, double y, double z,
                               double roll, double pitch, double yaw) {
  cout << "\n\nERROR in Cfg_fixed_tree::Cfg_fixed_tree(double,double,double,double,double,double), not implement yet.\n";
  exit(-1);
}
		  
Cfg_fixed_tree::Cfg_fixed_tree(int _numofJoints) {
  if (NumofJoints != _numofJoints ) {
    cout << "ERROR in Cfg_fixed_tree::Cfg_fixed_tree(int), cannot change numofJoints\n";
    exit(-1);
  }
    
  dof = NumofJoints;
  posDof = 0;
  
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(0);
  
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_fixed_tree::Cfg_fixed_tree(const vector<double>& _data) {
  dof = NumofJoints;
  posDof = 0;
  if((int)_data.size() < dof) {
    cout << "\n\nERROR in Cfg_fixed_tree::Cfg_fixed_tree(vector<double>), ";
    cout << "size of vector is less than dof\n";
    exit(-1);
  }
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_data[i]);
  
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_fixed_tree::Cfg_fixed_tree(const Cfg& _c) { 
  vector<double> _v;
  _v = _c.GetData();
  dof = _v.size();
  posDof = 0;
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_v[i]);
  Normalize_orientation();

  obst = _c.obst;
  tag = _c.tag;
  clearance = _c.clearance;
} 

Cfg_fixed_tree::~Cfg_fixed_tree() {}

void Cfg_fixed_tree::equals(const Cfg& c) {
  dof = c.DOF();
  posDof = c.posDOF();
  v.clear();
  v = c.GetData();
  obst = c.obst;
  tag = c.tag;
  clearance = c.clearance;
}
	
Vector3D Cfg_fixed_tree::GetRobotCenterPosition() const {
  return Vector3D(0, 0, 0);
}


const char* Cfg_fixed_tree::GetName() const {
  return "Cfg_fixed_tree";
}
  
void Cfg_fixed_tree::GetRandomCfg(double R, double rStep){
  v.clear();
  
  for(int i=0; i<NumofJoints; i++) {
    double jointAngle = (2.0*rStep)*DRand() - rStep;
    v.push_back(jointAngle);
  }
  
  obst = -1;
  tag = -1;
  clearance = -1;
}


void Cfg_fixed_tree::GetRandomCfg(Environment* env) {
    Cfg::GetRandomCfg(env);
}


void Cfg_fixed_tree::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
  //randomly sample params
  v.clear();
  for(int i=0; i<DOF(); ++i)
    v.push_back( double(2.0)*DRand() - double(1.0) );
  
  //scale to appropriate length
  Cfg_fixed_tree origin;
  dm->ScaleCfg(env, incr, origin, *this);
  
  Normalize_orientation();
  
  obst = -1;
  tag = -1;
  clearance = -1;
}

/*
void Cfg_fixed_tree::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
  incr = 0.005;
  v.clear();
  
  for(int i=0; i<NumofJoints; i++)
    v.push_back(DRand()*incr); //was multiplied by DefaultRange, probably an error
}
*/


void Cfg_fixed_tree::GetRandomCfg_CenterOfMass(Environment *env) {
  // Why following comments are here? This method suppose will generate
  // Cfg whose center of mass will inside a given bounding box....
  
  // this is not EXACTLY accurate, ok with most cases ... TO DO
  // To be accurate, one has to make sure every link is inside the given BB,
  // but here only the base link is taken care of. It is almost fine since
  // a little 'bigger' BB will contain all links. 
  v.clear();
  for(int i=0; i<dof; ++i) 
    v.push_back(env->GetBoundingBox()->GetRandomValueInParameter(i));
    
  obst = -1;
  tag = -1;
  clearance = -1;
}

void Cfg_fixed_tree::GetMovingSequenceNodes(const Cfg& other, vector<double> s, vector<Cfg*>& result) const {
  Cfg* c1 = this->CreateNewCfg();
  result.push_back(c1);
  
  vector<double> _data,_data2;
  for(int i=0; i<dof; i++) {
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
  for(i=0; i<NumofJoints; i++) {
    _env->GetMultiBody(robot)->GetFreeBody(i)
      ->GetBackwardConnection(0).GetDHparameters().theta = v[i]*360.0;
  }  // config the robot
  
  for(i=0; i<NumofJoints; i++) {
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

bool Cfg_fixed_tree::GenerateOverlapCfg(Environment *env,  // although env and robot is not used here,
					int robot,            // they are needed in other Cfg classes.
					Vector3D robot_start, 
					Vector3D robot_goal, 
					Cfg *resultCfg){

  vector<double> treeData;
  for(int i=0; i<NumofJoints; i++)
    treeData.push_back(env->GetBoundingBox()->GetRandomValueInParameter(i));
  
  // pass back the Cfg for this pose.
  *resultCfg = Cfg_fixed_tree(treeData);
  return true;
}

//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
// Guang Song 08/24/99
//===================================================================
void Cfg_fixed_tree::GenSurfaceCfgs4ObstNORMAL(MPProblem* mp, Environment * env,
					       Stat_Class& Stats,
					       string vc_method, 
					       int obstacle, int nCfgs, 
					       CDInfo& _cdInfo, 
					       vector<Cfg*>& surface) const {
  cout << "\n\nERROR in Cfg_fixed_tree::GenSurfaceCfgs4ObstNORMAL(), not implemented yet\n";
  exit(10);
}

Cfg* Cfg_fixed_tree::CreateNewCfg() const {
  Cfg* tmp = new Cfg_fixed_tree(NumofJoints);
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_fixed_tree::CreateNewCfg(vector<double>& data) const {
  vector<double> _data;
  if((int)data.size() < dof) {
    cout << "\n\nERROR in Cfg_fixed_tree::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than dof\n";
    exit(-1);
  }
  for(int i=0; i<dof; i++)
    _data.push_back(data[i]);
  Cfg* tmp = new Cfg_fixed_tree(_data);
  return tmp;
}


void Cfg_fixed_tree::c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d) {
  cout << "Error in Cfg_fixed_tree::c1_towards_c2(), not implement yet.\n";
  exit(-1);
}

