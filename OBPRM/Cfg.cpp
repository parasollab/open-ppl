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
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
//Include standard headers
#include <vector.h>

/////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "Cfg.h"
#include "MultiBody.h"
#include "CfgManager.h"
#include "Cfg_free.h"
#include "Environment.h"
#include "DistanceMetrics.h"
#include "CollisionDetection.h"

/////////////////////////////////////////////////////////////////////
//Init static Data Member
const int InfoCfg::NULL_INFO =-1;

//---------------------------------------------
// Input/Output operators for InfoCfg
//---------------------------------------------
istream& operator>> (istream&s, InfoCfg &_c){
    s >> _c.obst;
    s >> _c.tag;
    s >> _c.clearance;
    _c.Read(s);
    return s;
}
ostream& operator<< (ostream&s, const InfoCfg &_c){
    s << _c.obst      << " ";
    s << _c.tag       << " ";
    s << _c.clearance << " ";
    _c.Write(s);
    return s;
}

void InfoCfg::Write(ostream &s) const {
    s << obst      << " ";
    s << tag       << " ";
    s << clearance << " ";
}

void InfoCfg::Read(istream &s) {
    s >> obst;
    s >> tag;
    s >> clearance;
}

/////////////////////////////////////////////////////////////////////

ClearanceInfo::ClearanceInfo() {
    clearance = -1e10;
    direction = NULL;
    checkOneDirection = 0;
}

ClearanceInfo::ClearanceInfo(double _clearance, Cfg * _direction){
    clearance = _clearance;
    direction = _direction;
    checkOneDirection = 0;
}

ClearanceInfo::ClearanceInfo(double _clearance, Cfg * _direction, int _checkOneDirection){
    clearance = _clearance;
    direction = _direction;
    checkOneDirection = _checkOneDirection;
}

ClearanceInfo::ClearanceInfo(Cfg * _direction, int _checkOneDirection){
    clearance = -1e10;
    direction = _direction;
    checkOneDirection = _checkOneDirection;
}

ClearanceInfo::~ClearanceInfo() {

}


CfgManager * Cfg::CfgHelper = new Cfg_free();

const char* Cfg::GetName() {
    //return CfgHelper->GetName();
    return "Cfg_free_rigid";
}   


// Normalize the orientation to the some range.
void Cfg::Normalize_orientation(int index) {
    CfgHelper->Normalize_orientation(*this, index);
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

Cfg Cfg::operator+(const Cfg &tmp) const{
    vector<double> a;
    for(int i=0; i<v.size(); ++i)
        a.push_back(v[i]+tmp.v[i]);
    return Cfg(a);
}

Cfg Cfg::operator-(const Cfg &tmp) const{
    vector<double> a;
    for(int i=0; i<v.size(); ++i)
        a.push_back(v[i]-tmp.v[i]);
    return Cfg(a);
}

Cfg Cfg::operator-() const{
    vector<double> a;
    for(int i=0; i<v.size(); ++i)
        a.push_back(-v[i]);
    return Cfg(a);
}

Cfg Cfg::operator*(double s) {
    vector<double> a;
    for(int i=0; i<v.size(); ++i)
        a.push_back(v[i]*s);
    return Cfg(a);
}

Cfg Cfg::operator/(double s) {
    vector<double> a;
    for(int i=0; i<v.size(); ++i)
        a.push_back(v[i]/s);
    return Cfg(a);
}

bool Cfg::operator==(const Cfg &tmp) const{

    //return v.size() == tmp.v.size() &&
    //    equal(v.begin(), v.end(), tmp.v.begin());
    return CfgHelper->AlmostEqual(*this, tmp);
}

bool Cfg::operator!=( const Cfg &tmp) const{

        return !(*this==tmp);
}

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

const vector<double>& Cfg::GetData() const {
    return v;
}

Vector3D Cfg::GetRobotCenterPosition(){
    return CfgHelper->GetRobotCenterPosition(*this);
}


// Return the number of degrees of freedom for the configuration class
int Cfg::DOFs() {
    return CfgHelper->GetDOF();
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
        Normalize_orientation(param);
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
        Normalize_orientation(param);
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
Cfg Cfg::ClosestPtOnLineSegment(const Cfg &pt1, const Cfg &pt2) const{
    
    Cfg B = pt2-pt1;
    Cfg C = *this - pt1;
    
    double B_dot_C  =0,
        B_squared=0;
    
    ///Modified for VC
#if defined(_WIN32)
    using namespace std;
#endif
    
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
}

Cfg Cfg::c1_towards_c2(Cfg cfg1, Cfg cfg2, double d){

    Cfg tmp = cfg2 - cfg1;
    tmp = tmp / tmp.PositionMagnitude();
    cfg2 = cfg1 + (tmp * d);
    return cfg2;
}

// generates a random configuration without consideration of bounding box restrictions
Cfg Cfg::GetRandomCfg(double R, double rStep) {
    return CfgHelper->GetRandomCfg(R,rStep);
}

// generates random configuration where workspace robot's CENTER OF MASS
// is guaranteed to lie within the environment specified bounding box
Cfg Cfg::GetRandomCfg_CenterOfMass(double *boundingBox) {
    return CfgHelper->GetRandomCfg_CenterOfMass(boundingBox);
}


// generates random configuration where workspace robot's EVERY VERTEX
// is guaranteed to lie within the environment specified bounding box
Cfg Cfg::GetRandomCfg(Environment *env, int maxTries) {
    
    // Probably should do something smarter than 3 strikes and exit.
    // eg, if it fails once, check size of bounding box vs robot radius
    // and see if user has an impossibly small (for this robot) bounding
    // box specified
    
    double *bb = env->GetBoundingBox();
    
    while (maxTries-- > 0){
        Cfg tmp = GetRandomCfg_CenterOfMass(bb);
        if (tmp.InBoundingBox(env))
            return tmp;
    }//endwhile
    
    
    // Print error message and some helpful (I hope!) statistics and exit...
    cout << "\n\nERROR: GetRandomCfg not able to find anything in bounding box."
        <<   "\n       robot radius is "
        << env->GetMultiBody(env->GetRobotIndex())->GetBoundingSphereRadius();
    env->DisplayBoundingBox(cout);
    exit(-1);
    
    // compiler wants this method to return something
    return InvalidData();
}

// ditto, but with a default number of tries (10)
Cfg Cfg::GetRandomCfg(Environment *env) {
    int default_maxTries = 100;
    return Cfg::GetRandomCfg(env, default_maxTries);
}

// generates a random cfg with a given length 
Cfg Cfg::GetRandomCfg(Environment *env,DistanceMetric *dm,
            SID dmsetid,double length)
{
	Cfg origin;
	Cfg outsideCfg= GetRandomCfg(env,1000);
	
	// first find an outsite configuration with sufficient size

	for(;dm->Distance(env,origin,outsideCfg,dmsetid)<2*length;
			outsideCfg=outsideCfg*2); 
	

	// now, using binary search  find a configuration with the approximate
	// length
	
	Cfg aboveCfg=outsideCfg;
	Cfg belowCfg=origin;
	Cfg currentCfg;
	while  (1) {
	  currentCfg=(aboveCfg+belowCfg)*0.5;
	  /*cout <<"Above " << aboveCfg << endl;
	  cout <<"Current " << currentCfg << endl;
	  cout <<"Below " << belowCfg << endl; */
	  double magnitude=dm->Distance(env,origin,currentCfg,dmsetid);
	  double diff=dm->Distance(env,aboveCfg,belowCfg,dmsetid);
	   cout << "Current magnitude is \n" << magnitude << " " << length<<
		  " " <<diff<< endl;
	  if( (magnitude >=length*0.9) && (magnitude <=length*1.1))
		  break;
          if(magnitude>length) aboveCfg=currentCfg ;
	  else belowCfg=currentCfg; 
          
	}
	return currentCfg;
}

// generates random configuration and then pushes it to the medial axis of
// the free c-space
Cfg Cfg::GetMedialAxisCfg(Environment *_env, CollisionDetection *_cd,
                          SID _cdsetid, CDInfo &_cdInfo, DistanceMetric *_dm,
                          SID _dmsetid, int n) {
    Cfg maprmCfg = GetRandomCfg(_env);
    maprmCfg.PushToMedialAxis(_env,_cd,_cdsetid,_cdInfo,_dm,_dmsetid,n);
    return maprmCfg;
}

// pushes node towards c-space medial axis
void Cfg::PushToMedialAxis(Environment *_env, CollisionDetection *cd,
			     SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
			     SID dmsetid, int n) {
    Cfg cfg = *this;

    if (cfg.isCollision(_env,cd,cdsetid,cdInfo)) {
        ClearanceInfo clearInfo;
        cfg.ApproxCSpaceClearance2(_env,cd,cdsetid,cdInfo,dm,dmsetid,n,clearInfo,1);
	cfg = *clearInfo.getDirection();
	delete clearInfo.getDirection();
    }

    if (!(cfg.isCollision(_env,cd,cdsetid,cdInfo))) {
        cfg.MAPRMfree(_env,cd,cdsetid,cdInfo,dm,dmsetid,n);
    }

    *this = cfg;
}

// pushes free node towards c-space medial axis
void Cfg::MAPRMfree(Environment *_env, CollisionDetection *cd,
                   SID cdsetid, CDInfo &cdInfo, DistanceMetric *dm,
                   SID dmsetid, int n) {
    Cfg cfg = *this;
    Cfg newCfg, oldCfg, dir;
    
    ClearanceInfo clearInfo;
    cfg.ApproxCSpaceClearance2(_env,cd,cdsetid,cdInfo,dm,dmsetid,n,clearInfo,0);
    cfg.info.clearance = clearInfo.getClearance();
    dir = *(clearInfo.getDirection());
    delete clearInfo.getDirection();
    
    int i = 0;
    double stepSize = cfg.info.clearance;
    
    oldCfg = cfg;
    newCfg = oldCfg;

    /// find max. clearance point by stepping out:
    while ((newCfg.info.clearance >= oldCfg.info.clearance) && (newCfg.InBoundingBox(_env))) {
        oldCfg = newCfg;
        newCfg = c1_towards_c2(newCfg, dir, stepSize*-1);
        newCfg.info.clearance = newCfg.ApproxCSpaceClearance(_env,cd,cdsetid,cdInfo,dm,dmsetid,n,0);

        stepSize = newCfg.info.clearance;
        i++;
    }

    if (newCfg.InBoundingBox(_env)) {
        /// binary search betwen oldCfg and newCfg to find max clearance:
        Cfg midCfg;
	double minDistance = _env->GetPositionRes() + _env->GetOrientationRes();
        int maxNumSteps = 200; //arbitrary
        
        do {
            midCfg = (newCfg + oldCfg) / 2; //midpoint between newCfg and oldCfg
            midCfg.info.clearance = midCfg.ApproxCSpaceClearance(_env,cd,cdsetid,cdInfo,dm,dmsetid,n,0);

            if (midCfg.info.clearance > oldCfg.info.clearance) {
                oldCfg = midCfg;
            } else {
                newCfg = midCfg;
            }
            
            i++;
        } while ((dm->Distance(_env, newCfg, oldCfg, dmsetid) > minDistance) && (i <= maxNumSteps));
        
    }
    
    *this = oldCfg;
}

// pushes colliding node towards free space
void Cfg::MAPRMcollision(Environment *_env, CollisionDetection *cd,
                        SID cdsetid, CDInfo& cdInfo, int n) {
    Cfg cfg = *this;
    double stepSize = 0.5;
    
    ///pick n random directions:
    vector <Cfg> directions;
    vector <Cfg> steps;
    for (int i=0; i<n; i++) {
        directions.push_back(GetRandomCfg(_env));
        steps.push_back(cfg);
    }  
    
    if (directions.size()==0)
        *this = cfg;
    
    ///step out along each direction:
    int found = -1;
    while (found < 0) {
        for (int i=0; i<directions.size(); i++) {
            steps[i]=c1_towards_c2(steps[i],directions[i],stepSize);
            if (!(steps[i].isCollision(_env,cd,cdsetid,cdInfo))) {
                found = i;
                break;
            }
        }
        stepSize = stepSize * 2;
    }
    
    *this = steps[found];
}

Cfg Cfg::GetFreeRandomCfg(Environment *env, CollisionDetection *cd, SID _cdsetid,
                          CDInfo& _cdInfo){
    return CfgHelper->GetFreeRandomCfg(env, cd, _cdsetid, _cdInfo);
}


void Cfg::GetNFreeRandomCfgs(vector<Cfg> &nodes, Environment *env,
                 CollisionDetection* cd,SID _cdsetid, CDInfo& _cdInfo, int num) {
      CfgHelper->GetNFreeRandomCfgs(nodes, env, cd, _cdsetid, _cdInfo, num);
};


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
    return true;
    
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
                               Environment *_env, const Cfg &increment,
                               CollisionDetection *cd,
                               int noNeighbors,
                               SID  _cdsetid, CDInfo& _cdInfo){
    
    return CfgHelper->FindNeighbors(*this, 
        _env, increment, 
        cd, 
        noNeighbors, 
        _cdsetid, _cdInfo);
    
}

vector<Cfg> Cfg::FindNeighbors(
                               Environment *_env, const Cfg& goal, const Cfg& increment,
                               CollisionDetection *cd,
                               int noNeighbors,
                               SID  _cdsetid, CDInfo& _cdInfo) {
    
    return CfgHelper->FindNeighbors(*this, 
        _env, goal, increment, 
        cd, 
        noNeighbors, 
        _cdsetid, _cdInfo);
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
    info = InfoCfg(); // reset info to NULL_INFO. 
    
}


Cfg Cfg::GetResolutionCfg(Environment *env)
{
	vector <double> tmp;
	double posRes=env->GetPositionRes();
	double oriRes=env->GetOrientationRes();
	int posDof=CfgHelper->PosDOF();
	int dof=CfgHelper->GetDOF();
	
	for(int i=0;i<dof;i++)
		if(i<posDof) tmp.push_back(posRes);
		else tmp.push_back(oriRes);

	return Cfg(tmp);
}
vector <double> Cfg::GetOrientation()
{
    return CfgHelper->GetOrientation(*this);
}

vector <double> Cfg::GetPosition()
{
    return CfgHelper->GetPosition(*this);
}

double  Cfg::OrientationMagnitude()
{
    return CfgHelper->OrientationMagnitude(*this);
}


double  Cfg::PositionMagnitude()
{
    return CfgHelper->PositionMagnitude(*this);
}


vector<Cfg> Cfg::GetMovingSequenceNodes(const Cfg &other, double s) const {
    return CfgHelper->GetMovingSequenceNodes(*this, other, s);
}


//---------------------------------------------
// Input/Output operators for Cfg
//---------------------------------------------
istream& operator>> (istream&s, Cfg &pt){
    pt.Read(s);
    pt.info.Read(s);
    return s;
}
ostream& operator<< (ostream&s, const Cfg &pt){
    pt.Write(s);
    pt.info.Write(s);
    return s;
}

void Cfg::Write(ostream &os) const {
    for(int i=0; i<v.size(); ++i) {
        os << setw(4)<<v[i]<<' ';
    }
}

void Cfg::Read(istream &is) {
    for(int i=0; i<v.size(); ++i) {
        is >> v[i];
    }
}

void Cfg::print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg) {
    CfgHelper->print_preamble_to_file(env, _fp, numofCfg);
    
}

void Cfg::printLinkConfigurations(Environment *env, vector<Vector6D> &cfigs) const {
    CfgHelper->printLinkConfigurations(*this, env, cfigs);
}


bool Cfg::ConfigEnvironment(Environment *env) {
    return CfgHelper->ConfigEnvironment(*this, env);
}


bool Cfg::isCollision(Environment *env,CollisionDetection *cd, SID _cdsetid,
                      CDInfo& _cdInfo,bool enablePenetration){
    return CfgHelper->isCollision(*this, env, cd, _cdsetid, _cdInfo,enablePenetration);
}

bool Cfg::isCollision(Environment *env, CollisionDetection *cd,
                      int robot, int obs, SID _cdsetid, CDInfo& _cdInfo,
		      bool enablePenetration){
    return CfgHelper->isCollision(*this, env, cd, robot, obs, _cdsetid, _cdInfo,
		     enablePenetration);
}

double Cfg::Clearance(Environment *env,CollisionDetection *cd ){
    if(!ConfigEnvironment(env))
        return -1;
    return  cd->Clearance(env);
}


//Approximate C-Space Clearance
double Cfg::ApproxCSpaceClearance(Environment *env, 
                                  CollisionDetection *cd, 
                                  SID cdsetid, 
                                  CDInfo& cdInfo,
                                  DistanceMetric * dm, 
                                  SID dmsetid, int n, bool bComputePenetration)
{
    ClearanceInfo clearInfo;
    ApproxCSpaceClearance2(env, cd, cdsetid, cdInfo, dm, dmsetid, n, clearInfo, bComputePenetration);
    delete clearInfo.getDirection(); //free direction memory, allocated in ApproxCSpaceClearance2
    return clearInfo.getClearance();
}


//Approximate C-Space Clearance
void
Cfg::ApproxCSpaceClearance2(Environment *env, 
                            CollisionDetection *cd, 
                            SID cdsetid, 
                            CDInfo& cdInfo,
                            DistanceMetric * dm, 
                            SID dmsetid, 
                            int n, 
                            ClearanceInfo & clearInfo,
                            bool bComputePenetration)
{
    Cfg cfg = *this;

    vector <Cfg> directions;
    double dist = 100 * env->GetPositionRes();
    int i;
    for (i=0; i<n; i++) {
        directions.push_back( GetRandomRay(dist) );
    }
    if (directions.size() == 0) { //unable to generate random directions
      return;
    }
    
    double positionRes = env->GetPositionRes();
    double orientationRes = env->GetOrientationRes();
    
    //if collide, set to true. Otherwise, set to false
    bool bInitState = cfg.isCollision( env, cd, cdsetid, cdInfo );

    if( bComputePenetration == false && bInitState == true ) //don't need to compute clearance
      return;
    if( bComputePenetration == true && bInitState == false ) //don't need to compute penetration
      return;

    //find max step size:
    int iRobot = env->GetRobotIndex();
    double incrBound = env->GetMultiBody(iRobot)->GetMaxAxisRange(); //max step size    
    
    vector <Cfg> tick;
    vector <Cfg> incr;
    vector <double> incrSize;
    vector <bool> ignored;
    int num_ignored = 0;
    int n_ticks;

    for (i=0; i<directions.size(); i++) {
        tick.push_back(cfg);
        incr.push_back(cfg.FindIncrement(directions[i], &n_ticks, positionRes, orientationRes));
        incrSize.push_back(incr[i].OrientationMagnitude() + incr[i].PositionMagnitude());
	ignored.push_back(false);
    }

    int stateChangedFlag = false;

    //step out along each direction:
    while ( !stateChangedFlag ) {
        for (int i=0; i<directions.size(); i++) {
            tick[i].Increment(incr[i]);

            if (bComputePenetration) { //finding penetration
	        if (!ignored[i] && (tick[i].isCollision(env, cd, cdsetid, cdInfo) != bInitState)) {
                    if (!(tick[i].InBoundingBox(env))) { //ignore this direction
                        ignored[i] = true;
                        num_ignored++;
                        if (num_ignored == directions.size()) { //if no more directions left, exit loop
			    clearInfo.setClearance(10000);

			    Cfg *tmp = new Cfg();
			    *tmp = cfg;
			    clearInfo.setDirection(tmp);

			    stateChangedFlag = true;
                        }
                    } else {
		        clearInfo.setClearance(dm->Distance(env, tick[i], cfg, dmsetid));

		        Cfg * tmp = new Cfg();
		        *tmp = tick[i];
		        clearInfo.setDirection(tmp);

		        stateChangedFlag = true;
                    }
                }
	    } else { //finding clearance
                if ( (tick[i].isCollision(env, cd, cdsetid, cdInfo) != bInitState) 
		     || !(tick[i].InBoundingBox(env)) ) {
     	            clearInfo.setClearance(dm->Distance(env, tick[i], cfg, dmsetid));

                    Cfg * tmp = new Cfg();
                    *tmp = tick[i];
                    clearInfo.setDirection(tmp);

                    stateChangedFlag = true;
	        }
	    }

	    if (stateChangedFlag) {
	        break; //exit for loop
	    }
 
            //if increment still less than a max bound
            if (incrSize[i] < incrBound) {
                incr[i] = incr[i] * 2;
                incrSize[i] = incrSize[i] * 2;
            }	
	   
        }//end for
    }//end while
  
    // if this cfg is free (state = false) then return smallestDistance (clearance)
    // if this cfg is not free (state = true) then return -smallestDistance (penetration)
    // clearInfo.setClearance((bInitState==false)?clearInfo.getClearance():-clearInfo.getClearance());
    clearInfo.setClearance(clearInfo.getClearance());
    return;
}


vector<Cfg> Cfg::ApproxCSpaceContactPoints(vector<Cfg> directions,
					   Environment *env,
					   CollisionDetection *cd,
					   SID cdsetid,
					   CDInfo &cdInfo) {

  Cfg origin = *this;
  vector<Cfg> contact_points;

  //initialize:
  int i;
  for(i=0; i<directions.size(); i++) {
    contact_points.push_back(origin);
  }
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  bool bInitState = origin.isCollision(env, cd, cdsetid, cdInfo);

  //find max step size:
  int iRobot = env->GetRobotIndex();
  double incrBound = env->GetMultiBody(iRobot)->GetMaxAxisRange();

  //step out along each ray and record first cfg in collision:
  for(i=0; i<contact_points.size(); i++) {

        int n_ticks;
	Cfg tick = origin;
        Cfg incr = origin.FindIncrement(directions[i], &n_ticks, positionRes, orientationRes);
        double incrSize = incr.OrientationMagnitude() + incr.PositionMagnitude(); //step size
        
        int stateChangedFlag = false;
        
        while(!stateChangedFlag) {
            
            tick.Increment(incr);
            bool bCurrentState = tick.isCollision(env, cd, cdsetid, cdInfo);
            //double currentDist;
            
            // if state was changed or this cfg is out of bounding box
            if( (bCurrentState != bInitState) || !(tick.InBoundingBox(env)) ){
	        //contact_points[i] = tick; //first cfg in collision
	        contact_points[i] = tick - incr; //last free cfg
                stateChangedFlag = true;
            }

            //if increment still less than a max bound
            if(incrSize < incrBound){
                incr = incr*2;
                incrSize *= 2;
            }
        }//end while
  }

  return contact_points;
}


bool Cfg::GenerateOverlapCfg(Environment *env,  // although env and robot is not used here,
                             int robot,         // they are needed in other Cfg classes.
                             Vector3D robot_start,
                             Vector3D robot_goal,
                             Cfg *resultCfg)
{
    return CfgHelper->GenerateOverlapCfg(env, robot, robot_start, robot_goal, resultCfg);
}


//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
// Guang Song 08/24/99
//===================================================================
vector<Cfg>
Cfg::GenSurfaceCfgs4ObstNORMAL
(Environment * env,CollisionDetection* cd, 
 int obstacle, int nCfgs, SID _cdsetid, CDInfo& _cdInfo)
{
    
    return CfgHelper->GenSurfaceCfgs4ObstNORMAL(env, cd, obstacle, nCfgs, _cdsetid,_cdInfo);
}

// return a configuration(conformation)'s potential.
double Cfg::Potential(Environment * env) const 
{
    return CfgHelper->GetPotential(*this, env);
}
