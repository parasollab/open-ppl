/////////////////////////////////////////////////////////////////////
//
//  Cfg_free_multiple.cpp
//
//  General Description
//      A derived class from CfgManager. It provides some 
//      specific implementation directly related to a set of free
//      flying objects.
//
//  Created
//      06/29/00        Sujay
//
//  Last Modified By:
//	09/07/00	Ian
//
/////////////////////////////////////////////////////////////////////

#include "util.h"
#include "Vectors.h"
#include "Cfg_free.h"
#include "Cfg_free_multiple.h"
#include "Environment.h"
#include "GenerateMapNodes.h"

Cfg_free_multiple::Cfg_free_multiple(int robots) : CfgManager(6 * robots, 3)
{
  MaskNew(robots, 6);
  cout << "initializing" << endl;
}

Cfg_free_multiple::~Cfg_free_multiple() {}


Cfg Cfg_free_multiple::GetRandomCfg(double R, double rStep)
{
  double alpha, beta, z, z1;
  double roll, pitch, yaw;
  vector<double> result;
  int i, j;
  Cfg cfg;

  for (i = 0; i < mask.size(); i++) {
    alpha = 2.0 * M_PI * drand48();
    beta  = 2.0 * M_PI * drand48();
    z = R * cos(beta);
    z1 = R * sin(beta);

    roll = (2.0 * rStep) * drand48() - rStep;
    pitch = (2.0 * rStep) * drand48() - rStep;  
    yaw = (2.0 * rStep) * drand48() - rStep;

    Vector6<double> base(z1 * cos(alpha), z1 * sin(alpha), z,
                         roll, pitch, yaw);

    for (j = 0; j < 6; j++) {
//    if (mask[i][j])
      result.push_back(base[j]);
    }
  }
  cfg.SetData(result);
  cout << "regular gencfg " << endl;
  return cfg;
}

Cfg Cfg_free_multiple::GetRandomCfg_CenterOfMass(double *boundingBox)
{
  vector<double> tmp;
  cout << tmp <<endl;
  int j;
  Cfg cfg;
	
  for(int i = 0; i < mask.size() * 6; i++) {
//  if (mask[int (i / 6)][i % 6]){
    double p;
//  cout << (j = i % 6) << boundingBox[j*2] << boundingBox[j*2+1] <<endl;

    if ((j = i % 6) < 3) {
      int k = 2 * j;
      p = boundingBox[k] + (boundingBox[k + 1] -
				boundingBox[k]) * drand48();
//    cout << p << endl;
      tmp.push_back(p);
    } else
      tmp.push_back(p = drand48());
  }
  cout << tmp <<endl;
  cfg.SetData(tmp);
  return cfg;
}


Cfg Cfg_free_multiple::FindIncrement( 
	const Cfg& _start,
	const Cfg& _goal, 
	int * n_ticks,
	double positionRes,
	double orientationRes)
{
  Cfg diff = _goal - _start; 

  // adding two basically makes this a rough ceiling...
  *n_ticks= (int)max(PositionMagnitude(diff)/positionRes, 
            OrientationMagnitude(diff)/orientationRes) + 2;

  return FindIncrement(_start, _goal, *n_ticks);

}

Cfg Cfg_free_multiple::FindIncrement( 
	const Cfg& c,
	const Cfg& _goal, 
	int  n_ticks)
{
  vector<double> incr;
  Cfg cfg;
  for(int i=0; i < mask.size() * 6; ++i) {
    if((i % 6) < 3) 
      incr.push_back((_goal.v[i] - c.v[i])/n_ticks);
    else
      incr.push_back(DirectedAngularDistance(c.v[i], _goal.v[i])/n_ticks);
  }

  cfg.SetData(incr);
//cout << "incr  " << endl;
  return cfg;
}


void Cfg_free_multiple::IncrementTowardsGoal(
	Cfg &c, 
	const Cfg &goal, 
	const Cfg &increment)
{
//  cout << "INCREMENT TOWARDS GOAL" << endl;
  double tmp;
  int i;
  for(i=0; i < mask.size() * 6; ++i) {
    if ( (i % 6) < 3) {
      if( fabs(goal.v[i]-c.v[i]) < fabs(increment.v[i]))
        c.v[i] = goal.v[i];
      else
        c.v[i] += increment.v[i];
    }
  }
  for(i=0; i < mask.size() * 6; ++i) {
    if ( (i % 6) >= 3) {
      if(c.v[i] != goal.v[i]) {
	double orientationIncr = increment.v[i] < 0.5 ? increment.v[i] : 
                                 1-increment.v[i];
        tmp=DirectedAngularDistance(c.v[i],goal.v[i]);
        if(fabs(tmp) < orientationIncr) {
          c.v[i]=goal.v[i];
        } else {
          c.v[i] += increment.v[i];
          c.v[i] = c.v[i] - floor(c.v[i]);
        }
      }
    }
  }
}



// get entire dof mask
vector<vector<bool> > Cfg_free_multiple::MaskGet(void)
{
  return mask;
}

// get the dof mask for a particular robot
vector<bool> Cfg_free_multiple::MaskGet(int robot_id)
{
  return mask[robot_id];
}

// get a single dof from a particular robot 
bool Cfg_free_multiple::MaskGet(int robot_id, int dof_id)
{
  return mask[robot_id][dof_id];
}

// make new mask from a prebuilt set of dof flags
bool Cfg_free_multiple::MaskNew(vector<vector<bool> > newmask_vec)
{
  mask.erase(mask.begin(), mask.end());
  mask = newmask_vec;
}

// make new (default all active) mask.  numdof_vec provides the dof
// count for each robot. 
bool Cfg_free_multiple::MaskNew(vector<int> numdof_vec)
{
  int i;

  mask.erase(mask.begin(), mask.end());
  for (i = 0; i < numdof_vec.size(); i++)
    mask.push_back(vector<bool>(numdof_vec[i], true));
}

// make new (default all active) mask of <robots> robots each with
// <numdof> dof. 
bool Cfg_free_multiple::MaskNew(int robots, int numdof)
{
  int i;

  mask.erase(mask.begin(), mask.end());
  for (i = 0; i < robots; i++)
    mask.push_back(vector<bool>(numdof, true));
}

// set entire dof mask
bool Cfg_free_multiple::MaskSet(vector<vector<bool> > newmask_vec)
{
  int i, j;

  if ((j = mask.size()) == newmask_vec.size()) {
    for (i = 0; i < j; i++)
      if (mask[i].size() == newmask_vec[i].size())
        mask[i] = newmask_vec[i];
      else
        return false;
      return true;
  } else
    return false;
}

// set mask for a particular robot
bool Cfg_free_multiple::MaskSet(int robot_id, vector<bool> newmask)
{
  if (mask.size() == newmask.size() &&
      robot_id >=0 && robot_id < mask.size()) {
    mask[robot_id] = newmask;
    return true;
  } else
    return false;
}

// set flag for a particular dof of a particular robot
bool Cfg_free_multiple::MaskSet(int robot_id, int dof_id, bool state)
{
 if (robot_id >=0 && robot_id < mask.size() &&
     dof_id >= 0 && dof_id < mask[robot_id].size()) {
   mask[robot_id][dof_id] = state;
   return true;
 } else
   return false;
}

// add robot to mask with initial flags
int Cfg_free_multiple::MaskAddRobot(vector<bool> newmask)
{
  mask.push_back(newmask);
  return mask.size();
}

// add robot to mask with <numdof> dof
int Cfg_free_multiple::MaskAddRobot(int numdof)
{
  mask.push_back(vector<bool>(numdof, true));
  return mask.size();
}

// remove a robot from the mask
int Cfg_free_multiple::MaskDelRobot(int robot_id)
{
  if (robot_id >=0 && robot_id < mask.size()) {
    mask.erase(&mask[robot_id]);
    return mask.size();
  } else
    return -1;
}

// revert dof mask to state before last change
bool Cfg_free_multiple::MaskPrevious(void)
{
  vector<vector<bool> > tmp;
	
  tmp = mask;
  mask = oldmask;
  oldmask = tmp;

  return true;
}

// activate all dof
bool Cfg_free_multiple::MaskOn(void)
{
  int i;

  for (i = 0; i < mask.size(); i++)
    mask[i] = vector<bool>(mask[i].size(), true);
  return true;
}

// activate dof of a particular robot
bool Cfg_free_multiple::MaskOn(int robot_id)
{
  if (robot_id >= 0 && robot_id < mask.size())
    mask[robot_id] = vector<bool>(mask[robot_id].size(), true);
  return true;
}

// deactivate all dof
bool Cfg_free_multiple::MaskOff(void)
{
  int i;

  for (i = 0; i < mask.size(); i++)
    mask[i] = vector<bool>(mask[i].size(), false);
  return true;

}

//deactivate dof for a particular robot
bool Cfg_free_multiple::MaskOff(int robot_id)
{
  if (robot_id >= 0 && robot_id < mask.size())
    mask[robot_id] = vector<bool>(mask[robot_id].size(), false);
  return true;
}

bool Cfg_free_multiple::ConfigEnvironment(const Cfg &c, Environment *_env) {
  vector<double> v = c.GetData();
//	FreeBody *fb;
/*
	cout << _env-> GetRobotIndex() << endl;
	cout << _env-> GetMultiBody(0)-> GetNumBodies() << endl;
*/

// configure the robot according to current Cfg: joint parameters
// (and base locations/orientations for free flying robots.)
	int index = _env->GetRobotIndex();	
//	int num_robots = c.GetData().size() / 6;

  for (int body = 0; body < mask.size(); body++) {
    int i = body * 6;
    Transformation T1 = Transformation(
  			Orientation(Orientation::FixedXYZ,
			v[i + 5] * TWOPI, v[i + 4] * TWOPI, v[i + 3] * TWOPI),
			Vector3D(v[i + 0], v[i + 1], v[i + 2]));
    _env->GetMultiBody(index)->GetFreeBody(body)->Configure(T1);// update link1
  }  // config the robot

//	fb->GetWorldTransformation();

// since Transformation is calculated in recursive manner, only
// let the last link(or Freebody) call getWorldTransformation will
// automatically calculate the transformations for all previous links.

// when all worldTransformations are recalculated by using new cfg, the
// config of the whole robot is updated.

// cout <<"cfgenv";
  return true;
}

// stuff from Cfg_free_serial. Temporarily unnecessary


Vector3D Cfg_free_multiple::GetRobotCenterPosition(const Cfg &c) const {
  vector<double> tmp = c.GetData();
  cout << "center";
  return Vector3D(tmp[0], tmp[1], tmp[2]);
}


Cfg Cfg_free_multiple::GetRandomRay(double incr) {

  double alpha,beta,z, z1;
  vector<double> result;
  int i, j;
  Cfg cfg;

  for (i = 0; i < mask.size(); i++) {
    alpha = 2.0*M_PI*drand48();
    beta  = 2.0*M_PI*drand48();
    z = incr*cos(beta);
    z1 = incr*sin(beta);

    Vector6<double> base(z1*cos(alpha),z1*sin(alpha),z,0.0,0.0,0.0);
    for(j=0; j<6; ++j){
// 	if (mask[i][j])
      result.push_back(base[j]);
    }
  }
  cout << "rray";

  cfg.SetData(result);
	
  return cfg;
}


bool Cfg_free_multiple::GenerateOverlapCfg(
		Environment *env,  // although env and robot is not used here,
		int robot,            // they are needed in other Cfg classes.
		Vector3D robot_start,
		Vector3D robot_goal,
		Cfg *resultCfg){

  int i;
  Vector3D diff = robot_goal - robot_start;

  vector<double> result;
  Cfg cfg;
     
  for(i = 0; i < mask.size() * 6; i++) {
    if((i % 6) < 3)
      result.push_back(diff[i]);
    else
      result.push_back(drand48());
  }

/*     for(i=0; i<3; ++i)
	result.push_back(diff[i]);
     for(i=3; i<dof; ++i)
	result.push_back(drand48());
*/
  // pass back the Cfg for this pose.
  cfg.SetData(result);
  *resultCfg = cfg;
  cout << "overlap";
  return true;
}

