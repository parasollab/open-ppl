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
    clearance = 1e10;
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
    clearance = 1e10;
    direction = _direction;
    checkOneDirection = _checkOneDirection;
}

ClearanceInfo::~ClearanceInfo() {
    
}

CfgManager * Cfg::cfgType = new Cfg_free();

const char* Cfg::GetName() {
    //return CfgHelper->GetName();
    return "Cfg_free_rigid";
}   


// Normalize the orientation to the some range.
void Cfg::Normalize_orientation(int index) {
    cfgType->Normalize_orientation(*this, index);
}

void Cfg::createCfgHelper() {
   CfgHelper = cfgType->create();
   assert(CfgHelper);
}

Cfg::Cfg() {
    createCfgHelper();
    for(int i=0; i<cfgType->GetDOF(); ++i)
        v.push_back(0.0);
}

Cfg::Cfg(const vector<double> &v2)
{
    createCfgHelper();
    v = v2;
    Normalize_orientation();
}

Cfg::Cfg(const Vector6<double> &v2)
{
    createCfgHelper();
    for(int i=0; i<6; ++i)
        v.push_back(v2[i]);
    Normalize_orientation();
}


Cfg::Cfg(
         double x,double y,double z,
         double roll,double pitch,double yaw) {
    
    createCfgHelper();
    Vector6<double> tmp(x,y,z,roll,pitch,yaw);
    for(int i=0; i<6; ++i)
        v.push_back(tmp[i]);
    Normalize_orientation();
}

Cfg::Cfg(double x,double y,double z) {
    
    createCfgHelper();
    Vector3D tmp(x,y,z);
    for(int i=0; i<3; ++i)
        v.push_back(tmp[i]);
    Normalize_orientation();
}

Cfg::Cfg(const Cfg &c) {
    CfgHelper = c.CfgHelper->clone();
    v = c.v;
    info = c.info;
}

Cfg::~Cfg(){
    delete CfgHelper;
}

Cfg & Cfg::operator= (const Cfg&c) {
    if( this != &c) { // avoid self assignment
       delete CfgHelper;
       CfgHelper = c.CfgHelper->clone();
       v = c.v;
       info = c.info;
    } 
    return *this;
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
    return cfgType->AlmostEqual(*this, tmp);
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
    return cfgType->AlmostEqual(*this, _c);
}

bool Cfg::isWithinResolution(const Cfg&c, double positionRes,double orientationRes){
    return cfgType->isWithinResolution(*this, c, positionRes, orientationRes);
}

Cfg Cfg::InvalidData(){
    return cfgType->InvalidData();
}

const vector<double>& Cfg::GetData() const {
    return v;
}

Vector3D Cfg::GetRobotCenterPosition(){
    return cfgType->GetRobotCenterPosition(*this);
}


// Return the number of degrees of freedom for the configuration class
int Cfg::DOFs() {
    return cfgType->GetDOF();
}

// Return the range of a single parameter of the configuration (i.e., range of x)
// param = the parameter to get the range for
// In the future, this function should get the range for x,y,z by the bounding box
// Currently it assumes the range for a position parameter to be -10000 to 10000
// and the range for an orientation parameter to be 0 to 1, which should
// also be changed to reflect any self collision in a linked robot
pair<double,double> Cfg::SingleParamRange(int param) {
    return cfgType->SingleParamRange(param);
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
    return cfgType->GetRandomCfg(R,rStep);
}

// generates random configuration where workspace robot's CENTER OF MASS
// is guaranteed to lie within the environment specified bounding box
Cfg Cfg::GetRandomCfg_CenterOfMass(double *boundingBox) {
    return cfgType->GetRandomCfg_CenterOfMass(boundingBox);
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

// generates random configuration and then pushes it to the medial axis of
// the free c-space
Cfg Cfg::GetMedialAxisCfg(Environment *_env, CollisionDetection *_cd,
                          SID _cdsetid, CDInfo &_cdInfo, DistanceMetric *_dm,
                          SID _dmsetid, int n) {
    Cfg maprmCfg = GetRandomCfg(_env);
    
    if (maprmCfg.isCollision(_env, _cd, _cdsetid, _cdInfo)) {
        maprmCfg = MAPRMcollision(maprmCfg,_env,_cd,_cdsetid,_cdInfo,n); 
        if (!(maprmCfg.isCollision(_env,_cd,_cdsetid,_cdInfo))) {
            maprmCfg = MAPRMfree(maprmCfg,_env,_cd,_cdsetid,_cdInfo,_dm,_dmsetid,n);
        }
    } else {
        maprmCfg = MAPRMfree(maprmCfg,_env,_cd,_cdsetid,_cdInfo,_dm,_dmsetid,n);
    }
    
    return maprmCfg;
}

// pushes free node towards medial axis
Cfg Cfg::MAPRMfree(Cfg cfg, Environment *_env, CollisionDetection *cd,
                   SID cdsetid, CDInfo &cdInfo, DistanceMetric *dm,
                   SID dmsetid, int n) {
    Cfg newCfg, oldCfg, dir;
    
    ClearanceInfo clearInfo;
    clearInfo = cfg.ApproxCSpaceClearance2(_env,cd,cdsetid,cdInfo,dm,dmsetid,n);
    cfg.info.clearance = clearInfo.getClearance();
    dir = *(clearInfo.getDirection());
    
    double stepSize = cfg.info.clearance;
    int maxNumSteps = 200;
    
    oldCfg = cfg;
    newCfg = oldCfg;
    
    /// find max. clearance point by stepping out:
    int i=0;
    
    while ((newCfg.info.clearance >= oldCfg.info.clearance) && (newCfg.InBoundingBox(_env))) {
        oldCfg = newCfg;
        newCfg = c1_towards_c2(newCfg, dir, stepSize*-1);
        newCfg.info.clearance = newCfg.ApproxCSpaceClearance(_env,cd,cdsetid,cdInfo,dm,dmsetid,n);

        stepSize = newCfg.info.clearance;
        i++;
    }
    
    if (newCfg.InBoundingBox(_env)) {
        /// binary search betwen oldCfg and newCfg to find max clearance:
        Cfg midCfg;
        double minDistance = 0.1;
        
        do {
            midCfg = (newCfg + oldCfg) / 2; //midpoint between newCfg and oldCfg
            midCfg.info.clearance = midCfg.ApproxCSpaceClearance(_env,cd,cdsetid,cdInfo,dm,dmsetid,n);

            if (midCfg.info.clearance > oldCfg.info.clearance) {
                oldCfg = midCfg;
            } else {
                newCfg = midCfg;
            }
            
            i++;
        } while ((dm->Distance(_env, newCfg, oldCfg, dmsetid) > minDistance) && (i<=maxNumSteps));
        
    }
    
    //free direction memory, allocated in ApproxCSpaceClearance2
    delete clearInfo.getDirection();
    return oldCfg;
}

// pushes colliding node towards free space
Cfg Cfg::MAPRMcollision(Cfg cfg, Environment *_env, CollisionDetection *cd,
                        SID cdsetid, CDInfo& cdInfo, int n) {
    double stepSize = 0.5;
    
    ///pick n random directions:
    vector <Cfg> directions;
    vector <Cfg> steps;
    for (int i=0; i<n; i++) {
        directions.push_back(GetRandomCfg(_env));
        steps.push_back(cfg);
    }  
    
    if (directions.size()==0)
        return cfg;
    
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
    
    return steps[found];
}

Cfg Cfg::GetFreeRandomCfg(Environment *env, CollisionDetection *cd, SID _cdsetid,
                          CDInfo& _cdInfo){
    return cfgType->GetFreeRandomCfg(env, cd, _cdsetid, _cdInfo);
}


void Cfg::GetNFreeRandomCfgs(vector<Cfg> &nodes, Environment *env,
                 CollisionDetection* cd,SID _cdsetid, CDInfo& _cdInfo, int num) {
      cfgType->GetNFreeRandomCfgs(nodes, env, cd, _cdsetid, _cdInfo, num);
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
    return cfgType->GetRandomRay(incr);
}



void Cfg::IncrementTowardsGoal(
                               const Cfg &goal,
                               const Cfg &increment)
{
    cfgType->IncrementTowardsGoal(*this, goal, increment);
}


vector<Cfg> Cfg::FindNeighbors(
                               Environment *_env, const Cfg &increment,
                               CollisionDetection *cd,
                               int noNeighbors,
                               SID  _cdsetid, CDInfo& _cdInfo){
    
    return cfgType->FindNeighbors(*this, 
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
    
    return cfgTypecfgTypecfgType->FindNeighbors(*this, 
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
    return cfgType->FindIncrement(*this, _goal, n_ticks, positionRes, orientationRes);
}

Cfg Cfg::FindIncrement(
                       const Cfg& _goal,
                       int  n_ticks)
{
    return cfgType->FindIncrement(*this, _goal, n_ticks);
}


void Cfg::Increment(const Cfg &_increment)
{
    for(int i=0; i<v.size(); ++i)
        v[i] += _increment.v[i];
    Normalize_orientation();
    
}


vector <double> Cfg::GetOrientation()
{
    return cfgType->GetOrientation(*this);
}

vector <double> Cfg::GetPosition()
{
    return cfgType->GetPosition(*this);
}

double  Cfg::OrientationMagnitude()
{
    return cfgType->OrientationMagnitude(*this);
}


double  Cfg::PositionMagnitude()
{
    return cfgType->PositionMagnitude(*this);
}


vector<Cfg> Cfg::GetMovingSequenceNodes(const Cfg &other, double s) const {
    return cfgType->GetMovingSequenceNodes(*this, other, s);
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
    cfgType->print_preamble_to_file(env, _fp, numofCfg);
    
}

void Cfg::printLinkConfigurations(Environment *env, vector<Vector6D> &cfigs) const {
    cfgType->printLinkConfigurations(*this, env, cfigs);
}


bool Cfg::ConfigEnvironment(Environment *env) {
    return cfgType->ConfigEnvironment(*this, env);
}


bool Cfg::isCollision(Environment *env,CollisionDetection *cd, SID _cdsetid,
                      CDInfo& _cdInfo){
    return cfgType->isCollision(*this, env, cd, _cdsetid, _cdInfo);
}

bool Cfg::isCollision(Environment *env, CollisionDetection *cd,
                      int robot, int obs, SID _cdsetid, CDInfo& _cdInfo){
    return cfgType->isCollision(*this, env, cd, robot, obs, _cdsetid, _cdInfo);
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
    ClearanceInfo clearInfo = 
        ApproxCSpaceClearance2(env, cd, cdsetid, cdInfo, dm, dmsetid, n,bComputePenetration);
    delete clearInfo.getDirection(); //free direction memory, allocated in ApproxCSpaceClearance2
    return clearInfo.getClearance();
}

//Approximate C-Space Clearance
ClearanceInfo 
Cfg::ApproxCSpaceClearance2(Environment *env, 
                            CollisionDetection *cd, 
                            SID cdsetid, 
                            CDInfo& cdInfo,
                            DistanceMetric * dm, 
                            SID dmsetid, int n,
                            bool bComputePenetration) {
    ClearanceInfo clearInfo;
    return ApproxCSpaceClearance2(env,cd,cdsetid,cdInfo,dm,dmsetid,n,clearInfo,bComputePenetration);
}

//Approximate C-Space Clearance
ClearanceInfo 
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
    
    double positionRes = env->GetPositionRes();
    double orientationRes = env->GetOrientationRes();
    
    //if collide, set to true. Otherwise, set to false
    bool bInitState = cfg.isCollision(env,cd,cdsetid,cdInfo);
    if( bComputePenetration==false && bInitState==true) //don't need to comput penetration.
        return clearInfo;

    Cfg dir;
    if ( clearInfo.getCheckOneDirection() ) { //only want the c-space clearance in one specific direction
        n = 1;
        dir = *clearInfo.getDirection();
    }

    //Get information about robot
    int iRobot=env->GetRobotIndex();
    double incrBound = env->GetMultiBody(iRobot)->GetMaxAxisRange(); //max step size
    
    //shot n rays
    for(int i = 0 ; i < n ; i++){
        
        if ( !clearInfo.getCheckOneDirection() ) {
            dir = GetRandomCfg(env);
        }
        
        int n_ticks;
        Cfg tick = cfg;
        Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);
        double incrSize = incr.OrientationMagnitude()+incr.PositionMagnitude();//step size
        
        // this flag will be set if state changed. i.e. from collide to free or
        // from free to collide
        int stateChangedFlag = false;
        
        while(!stateChangedFlag && 
            (dm->Distance(env,cfg,tick,dmsetid)<clearInfo.getClearance()) ){
            
            tick.Increment(incr);
            bool bCurrentState = tick.isCollision(env,cd,cdsetid,cdInfo);
            double currentDist;
            
            // if state was changed or this cfg is out of bounding box
            if( (bCurrentState!=bInitState) || !(tick.InBoundingBox(env)) ){
                currentDist = dm->Distance(env, cfg, tick, dmsetid);
                if( currentDist < clearInfo.getClearance() ) {
                    clearInfo.setClearance( currentDist );
                    Cfg * pTmpDir = new Cfg();
                    *pTmpDir = tick-incr/2;
                    clearInfo.setDirection( pTmpDir );
                }
                stateChangedFlag = true;
            }

            //if increment still less than a max bound.
            if( incrSize<incrBound ){
                incr = incr*2;
                incrSize *= 2;
            }
        }
    }
    
    // if this cfg is free (state = false) then return smallestDistance (clearance)
    // if this cfg is not free (state = true) then return -smallestDistance (penetration)
    clearInfo.
        setClearance( (bInitState==false)?clearInfo.getClearance():-clearInfo.getClearance() );
    
    return clearInfo;
}

bool Cfg::GenerateOverlapCfg(Environment *env,  // although env and robot is not used here,
                             int robot,         // they are needed in other Cfg classes.
                             Vector3D robot_start,
                             Vector3D robot_goal,
                             Cfg *resultCfg)
{
    return cfgType->GenerateOverlapCfg(env, robot, robot_start, robot_goal, resultCfg);
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
    
    return cfgType->GenSurfaceCfgs4ObstNORMAL(env, cd, obstacle, nCfgs, _cdsetid,_cdInfo);
}

// return a configuration(conformation)'s potential.
double Cfg::Potential(Environment * env) const 
{
    return cfgType->GetPotential(*this, env);
}
