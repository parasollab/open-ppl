// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg.c
//
//  General Description
//      Configuration Data Class, it has all the interface needed
//      by other Motion Planning classes. Since it is abstract, it
//      will have to 'ask' a helper class called CfgManager to
//      provide implementation to some specific functions.
//
//  Created
//      08/31/99        Guang Song
//
//  Last Modified By:
//      08/31/99        Lucia K. Dale - name change for a method
//
/////////////////////////////////////////////////////////////////////

#include "util.h"
#include "Vectors.h"

#include "Cfg.h"
#include "CfgManager.h"
#include "Cfg_free.h"
#include "Environment.h"

#include "DistanceMetrics.h"

CfgManager * Cfg::CfgHelper = new Cfg_free();

// Normalize the orientation to the some range.
void Cfg::Normalize_orientation() {
   CfgHelper->Normalize_orientation(*this);
}

Cfg::Cfg() {
   for(int i=0; i<CfgHelper->GetDOF(); ++i)
	v.push_back(0.0);
}

Cfg::Cfg(const vector<double> &v2) 
{
   v = v2;
   Normalize_orientation();
}

Cfg::Cfg(const Vector6<double> &v2)
{
   for(int i=0; i<6; ++i)
        v.push_back(v2[i]);
   Normalize_orientation();
}


Cfg::Cfg(
	double x,double y,double z, 
	double roll,double pitch,double yaw) {

    Vector6<double> tmp(x,y,z,roll,pitch,yaw);
    for(int i=0; i<6; ++i)
        v.push_back(tmp[i]);
    Normalize_orientation();
}

Cfg::Cfg(double x,double y,double z) {

    Vector3D tmp(x,y,z);
    for(int i=0; i<3; ++i)
        v.push_back(tmp[i]);
    Normalize_orientation();
}

Cfg::~Cfg(){
}

Cfg Cfg::operator+(
                const Cfg &tmp) const{
    vector<double> a;
    for(int i=0; i<v.size(); ++i) 
	a.push_back(v[i]+tmp.v[i]);
    return Cfg(a);
};

Cfg Cfg::operator-(
                const Cfg &tmp) const{
    vector<double> a;
    for(int i=0; i<v.size(); ++i) 
        a.push_back(v[i]-tmp.v[i]);
    return Cfg(a);
};

Cfg Cfg::operator-() const{
    vector<double> a;
    for(int i=0; i<v.size(); ++i) 
        a.push_back(-v[i]);
    return Cfg(a);
};

Cfg Cfg::operator*(double s) {
    vector<double> a;
    for(int i=0; i<v.size(); ++i) 
        a.push_back(v[i]*s);
    return Cfg(a);
};

Cfg Cfg::operator/(double s) {
    vector<double> a;
    for(int i=0; i<v.size(); ++i) 
        a.push_back(v[i]/s);
    return Cfg(a);
};

bool Cfg::operator==(
                const Cfg &tmp) const{
        return v==tmp.v;
};

bool Cfg::operator!=(
                const Cfg &tmp) const{
        return !(v==tmp.v);
};

Cfg Cfg::WeightedSum(const Cfg& first,
                 const Cfg& second, double weight) {
    vector<double> a;
    for(int i=0; i<first.v.size(); ++i) 
        a.push_back(first.v[i]*(1.-weight) + second.v[i]*weight);
    return Cfg(a);
}


bool Cfg::AlmostEqual(const Cfg &_c)
{
   return CfgHelper->AlmostEqual(*this, _c);
}

bool Cfg::isWithinResolution(const Cfg&c, double positionRes,double orientationRes){
   return CfgHelper->isWithinResolution(*this, c, positionRes, orientationRes);
}

Cfg Cfg::InvalidData(){
  return CfgHelper->InvalidData();
}

vector<double> Cfg::GetData() const {
	return v;
}

Vector3D Cfg::GetRobotCenterPosition() /* brc const */{
  return CfgHelper->GetRobotCenterPosition(*this);
}


// Return the number of degrees of freedom for the configuration class
int Cfg::DOFs() {
  return v.size();
}

// Return the range of a single parameter of the configuration (i.e., range of x)
// param = the parameter to get the range for
// In the future, this function should get the range for x,y,z by the bounding box
// Currently it assumes the range for a position parameter to be -10000 to 10000
// and the range for an orientation parameter to be 0 to 1, which should
// also be changed to reflect any self collision in a linked robot
pair<double,double> Cfg::SingleParamRange(int param) {
   return CfgHelper->SingleParamRange(param);
}
 
// Set a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to set the parameter as
int Cfg::SetSingleParam(int param, double value) {
 
  if ((param>=0) && (param<DOFs())) {
    v[param]=value;
    return 1;
  } else {
    return 0;
  }
 
}

// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to increment the parameter by
int Cfg::IncSingleParam(int param, double value) {
 
  if ((param>=0) && (param<DOFs())) {
    v[param]+=value;
    return 1;
  } else {
    return 0;
  }
 
}
 
// Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to retreive
double Cfg::GetSingleParam(int param) {
  if ((param>=0) && (param<DOFs())) {
    return v[param];
  } else {
    return 0;
  }

}

// pt1 & pt2 are two endpts of a line segment
// find the closest point to the current cfg on that line segment
// it could be one of the two endpoints of course
Cfg Cfg::ClosestPtOnLineSegment(const Cfg &pt1, const Cfg &pt2)const{

    Cfg B = pt2   - pt1,
        C = *this - pt1;

    double B_dot_C  =0,
           B_squared=0;

    for (vector<double>::iterator b=B.v.begin(),c=C.v.begin();
                                  b<B.v.end();
                                  ++b,++c){
        B_dot_C   += (*b)*(*c);
        B_squared += (*b)*(*b);
    }

    if (B_dot_C <= 0){
	return pt1;
    } else if (B_dot_C >= B_squared){
	return pt2;
    } else {
	return pt1 + B*(B_dot_C/B_squared);
    }
};

Cfg Cfg::c1_towards_c2(Cfg cfg1, Cfg cfg2, double d){
      Cfg tmp = cfg2 - cfg1;
      tmp = tmp / tmp.PositionMagnitude();
      cfg2 = cfg1 + (tmp * d);
      return cfg2;
};


// generates a random configuration without consideration of bounding box restrictions
Cfg Cfg::GetRandomCfg(double R, double rStep) {
   return CfgHelper->GetRandomCfg(R,rStep);
}

// generates random configuration where workspace robot's CENTER OF MASS (COM)
// is guaranteed to lie within the environment specified bounding box
Cfg Cfg::GetRandomCfg_COM(double *boundingBox) {
   return CfgHelper->GetRandomCfg_COM(boundingBox);
}

// generates random configuration where workspace robot's EVERY VERTEX
// is guaranteed to lie within the environment specified bounding box
Cfg Cfg::GetRandomCfg(Environment *env) {

  // Probably should do something smarter than 3 strikes and exit.
  // eg, if it fails once, check size of bounding box vs robot radius
  // and see if user has an impossibly small (for this robot) bounding 
  // box specified
  int maxTries = 10;

  double *bb = env->GetBoundingBox();

  while (maxTries-- > 0){
     Cfg tmp = GetRandomCfg_COM(bb);
     if (tmp.InBoundingBox(env))
              return tmp;
  }//endwhile


  // Print error message and some helpful (I hope!) statistics and exit...
  cout << "\n\nERROR: GetRandomCfg not able to find anything in bounding box." 
       <<   "\n       robot radius is " 
       << env->GetMultiBody(env->GetRobotIndex())->GetBoundingSphereRadius();
  env->DisplayBB(cout);
  exit(-1);

  // compiler wants this method to return something
  return InvalidData(); 
}

// tests whether or not robot in this configuration has every vertex inside
// the environment specified bounding box
bool Cfg::InBoundingBox(Environment *env) {

  double *bb = env->GetBoundingBox();
  MultiBody *robot = env->GetMultiBody(env->GetRobotIndex());
  // if there are multiple robots, this line need to be changed too.

  // First, a faster, loose check:
  static const double minClearance = robot->GetBoundingSphereRadius();
  const Vector3D centerOfRobot = GetRobotCenterPosition();
  bool isCloseToWall = false;
  for(int i=0; i<3; ++i) {
      if(min(centerOfRobot[i]-bb[2*i], bb[2*i+1]-centerOfRobot[i]) < minClearance) {
            isCloseToWall = true;
            break;
      }
  }
  if(! isCloseToWall) return true;

  // if isCloseToWall, have a strict check.
  ConfigEnvironment(env); // Config the Environment(robot indeed).

  for(int m=0; m<robot->GetFreeBodyCount(); m++) {
     GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
     for(int j = 0 ; j < poly.numVertices ; j++){

        if(poly.vertexList[j][0] < bb[0] || poly.vertexList[j][0] > bb[1] ||
           poly.vertexList[j][1] < bb[2] || poly.vertexList[j][1] > bb[3] ||
           poly.vertexList[j][2] < bb[4] || poly.vertexList[j][2] > bb[5] ){

                return FALSE;
        }//endif
     }
  }
  return TRUE;

}


Cfg Cfg::GetRandomRay(double incr) {
   return CfgHelper->GetRandomRay(incr);
}



void Cfg::IncrementTowardsGoal(
	const Cfg &goal, 
	const Cfg &increment)
{
   CfgHelper->IncrementTowardsGoal(*this, goal, increment);
}


vector<Cfg> Cfg::FindNeighbors(
	Environment *_env,
	const Cfg &increment,
  	CollisionDetection *cd,
	int noNeighbors,
	SID  _cdsetid){

  return CfgHelper->FindNeighbors(*this, _env, increment, cd, noNeighbors, _cdsetid);

}

vector<Cfg> Cfg::FindNeighbors(
	Environment *_env,
	const Cfg& goal,
	const Cfg& increment,
        CollisionDetection *cd,
	int noNeighbors,
	SID  _cdsetid) {

  return CfgHelper->FindNeighbors(*this, _env, goal, increment, cd, noNeighbors, _cdsetid);
}

Cfg Cfg::FindIncrement( 
	const Cfg& _goal, 
	int * n_ticks,
	double positionRes,
	double orientationRes)
{
  return CfgHelper->FindIncrement(*this, _goal, n_ticks, positionRes, orientationRes);
}

Cfg Cfg::FindIncrement( 
	const Cfg& _goal, 
	int  n_ticks)
{
  return CfgHelper->FindIncrement(*this, _goal, n_ticks);
}

 
void Cfg::Increment(const Cfg &_increment)
{
   for(int i=0; i<v.size(); ++i) 
	v[i] += _increment.v[i];
   Normalize_orientation();
   
}


double  Cfg::OrientationMagnitude()
{
  return CfgHelper->OrientationMagnitude(*this);
}


double  Cfg::PositionMagnitude()
{
  return CfgHelper->PositionMagnitude(*this);
}


//Cfg Cfg::GetPositionOrientationFrom2Cfg(const Cfg &c1, const Cfg &c2) {
//  return CfgHelper->GetPositionOrientationFrom2Cfg(c1, c2);
//}

vector<Cfg> Cfg::GetMovingSequenceNodes(const Cfg &other, double s) const {
   return CfgHelper->GetMovingSequenceNodes(*this, other, s);
}



//---------------------------------------------
// Input/Output operators for Cfg
//---------------------------------------------
istream& operator>> (istream&s, Cfg &pt){
	for(int i=0; i<pt.v.size(); ++i) {
	   s >> pt.v[i];
	}
        return s;
};
ostream& operator<< (ostream&s, const Cfg &pt){
    for(int i=0; i<pt.v.size(); ++i) {
	s << setw(4)<<pt.v[i]<<' ';
    }
    return s;
};


void Cfg::print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg) {
     CfgHelper->print_preamble_to_file(env, _fp, numofCfg);

}

void Cfg::print_vizmo_format_to_file(Environment *env, FILE *_fp) {
// Environment *env is not used here. But needed for other Cfg class
// to interpret what an 'abstract' Cfg means.
     CfgHelper->print_vizmo_format_to_file(*this, env, _fp);
}


bool Cfg::ConfigEnvironment(Environment *env) {
     return CfgHelper->ConfigEnvironment(*this, env);
}


bool Cfg::isCollision(Environment *env,CollisionDetection *cd, SID _cdsetid){
     if(!ConfigEnvironment(env))
	 return true;

     // after updating the environment(multibodies), Ask ENVIRONMENT
     // to check collision! (this is more nature.)
     bool answerFromEnvironment = cd->IsInCollision(env, _cdsetid);
     return answerFromEnvironment;
}

bool Cfg::isCollision(Environment *env, CollisionDetection *cd, 
		int robot, int obs, SID _cdsetid){
     if(!ConfigEnvironment(env)) 
	  return true;

     // ask CollisionDetection class directly.
     bool answerFromCD = cd->IsInCollision(env, robot, obs, _cdsetid);
     return answerFromCD;
}

double Cfg::Clearance(Environment *env,CollisionDetection *cd ){
     if(!ConfigEnvironment(env)) 
	return -1;
     return  cd->Clearance(env);
}


//Approximate C-Space Clearance
double Cfg::ApproxCSpaceClearance(Environment *env, CollisionDetection *cd, SID cdsetid, DistanceMetric * dm, SID dmsetid, int n) 
{
  //cout << endl << endl << "Inside ApproxCSpaceClearance: " << flush;
  Cfg cfg = *this;
  //cout << endl << "cfg = " << cfg;

  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double clear, tmpDist;
  clear = 1e10;

  Cfg dir;
  for(int i = 0 ; i < n ; i++){
    dir = GetRandomCfg(env); 
    //cout << endl << "dir = " << dir;

    Cfg tick = cfg;
    Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);
    //cout << endl << "n_ticks = " << n_ticks;

    int tk = 0;
    int collisionFlag = false;
    while(tk < n_ticks && !collisionFlag){

      tick.Increment(incr);

      if(tick.isCollision(env,cd,cdsetid)){

	//cout << endl << "Inside Collision!" << flush;
	//cout << endl << "tick = " << tick;

	tmpDist = dm->Distance(env, cfg, tick, dmsetid);
	//cout << endl << "tmpDist = " << tmpDist;
	if(tmpDist < clear){
	  clear = tmpDist;
        }
	collisionFlag = true;
      }

      if(tk == n_ticks-1 && !collisionFlag){

	//cout << endl << "Inside No Collision!" << flush;
	//cout << endl << "tick = " << tick;

	tmpDist = dm->Distance(env, cfg, tick, dmsetid);
	//cout << endl << "tmpDist = " << tmpDist;

	if(tmpDist < clear){
	  clear = tmpDist;
	}
      }
      tk++;
    }
  }
  //cout << endl << "clear = " << clear;
  return clear;
}



bool Cfg::GenerateOverlapCfg(
		Environment *env,  // although env and robot is not used here,
		int robot,            // they are needed in other Cfg classes.
		Vector3D robot_start, 
		Vector3D robot_goal, 
		Cfg *resultCfg){

   return CfgHelper->GenerateOverlapCfg(env, robot, robot_start, robot_goal, resultCfg);
}


//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
// Guang Song 08/24/99
//===================================================================
vector<Cfg>
Cfg::GenSurfaceCfgs4ObstNORMAL
(Environment * env,CollisionDetection* cd, int obstacle, int nCfgs, SID _cdsetid){
   return CfgHelper->GenSurfaceCfgs4ObstNORMAL(env, cd, obstacle, nCfgs, _cdsetid);
}
