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
//Include OBPRM headers
#include "Cfg.h"
#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetrics.h"
#include "CollisionDetection.h"
#include "util.h"

#define EQ(a,b)  (fabs(a-b)<0.0001)

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
  delete direction;
}


////////////////////////////////////////////////////////////////////

bool Cfg::operator==(const Cfg &tmp) const {
  //return v.size() == tmp.v.size() &&
  //    equal(v.begin(), v.end(), tmp.v.begin());
  return AlmostEqual(tmp);
}


bool Cfg::operator!=( const Cfg& tmp) const {
  return !(*this==tmp);
}


bool Cfg::AlmostEqual(const Cfg &_c) const {
  for(int i=0; i<dof; ++i) {
    if(i<posDof) {
      if(!EQ(v[i], _c.v[i]))
	return false;
    } else {
      if(!EQ(DirectedAngularDistance(v[i], _c.v[i]), 0.0)) 
	return false;
    }
  }
  
  return true;
}


void Cfg::add(const Cfg& c1, const Cfg& c2) {
  vector<double> _v;
  for(int i=0; i<c1.v.size(); ++i)
    _v.push_back(c1.v[i]+c2.v[i]);
  v = _v;
  Normalize_orientation();
}


void Cfg::subtract(const Cfg& c1, const Cfg& c2) {
  vector<double> _v;
  for(int i=0; i<c1.v.size(); ++i)
    _v.push_back(c1.v[i]-c2.v[i]);
  v = _v;
  Normalize_orientation();
}


void Cfg::negative(const Cfg& c) {
  vector<double> _v;    
  for(int i=0; i<c.v.size(); ++i)
    _v.push_back(-c.v[i]);
  v = _v;
  Normalize_orientation();
}


void Cfg::multiply(const Cfg& c, double s) {
  vector<double> _v;
  for(int i=0; i<c.v.size(); ++i)
    _v.push_back(c.v[i]*s);
  v = _v;
  Normalize_orientation();
}


void Cfg::divide(const Cfg& c, double s) {
  vector<double> _v;
  for(int i=0; i<c.v.size(); ++i)
    _v.push_back(c.v[i]/s);
  v = _v;
  Normalize_orientation();
}


void Cfg::WeightedSum(const Cfg& first, const Cfg& second, double weight) {
  vector<double> _v;
  for(int i=0; i<first.v.size(); ++i)
    _v.push_back(first.v[i]*(1.-weight) + second.v[i]*weight);
  v = _v;
  Normalize_orientation();
}


bool Cfg::isWithinResolution(const Cfg& c, double positionRes, double orientationRes) const {
    Cfg* diff = this->CreateNewCfg();
    diff->subtract(*this, c);
    
    bool result = ((diff->PositionMagnitude() <= positionRes) &&
		   (diff->OrientationMagnitude() <= orientationRes));
    delete diff;
    return result;
}


//---------------------------------------------
// Input/Output operators for Cfg
//---------------------------------------------
istream& operator>> (istream& s, Cfg& pt) {
  pt.Read(s);
  pt.ReadInfo(s);
  return s;
}


ostream& operator<< (ostream& s, const Cfg& pt) {
  pt.Write(s);
  pt.WriteInfo(s);
  return s;
}


void Cfg::Write(ostream &os) const {
  for(int i=0; i<v.size(); ++i) {
    os << setw(4)<<v[i]<<' ';
  }
}


void Cfg::WriteInfo(ostream &os) const {
  os << obst << " ";
  os << tag << " ";
  os << clearance << " ";
}


void Cfg::Read(istream &is) {
  for(int i=0; i<v.size(); ++i) {
    is >> v[i];
  }
}


void Cfg::ReadInfo(istream &is) {
  is >> obst;
  is >> tag;
  is >> clearance;
}


void Cfg::printLinkConfigurations(Environment* env, vector<Vector6D>& cfigs) const {
  ConfigEnvironment(env);
  int robot = env->GetRobotIndex();
  int numofLink = env->GetMultiBody(robot)->GetFreeBodyCount();
  
  cfigs.erase(cfigs.begin(), cfigs.end());
  for(int i=0; i<numofLink; i++) {
    Transformation tmp = env->GetMultiBody(robot)->GetFreeBody(i)
      ->WorldTransformation();
    Orientation ori = tmp.orientation;
    ori.ConvertType(Orientation::FixedXYZ);
    Vector6D v6 = Vector6D(tmp.position[0], tmp.position[1], tmp.position[2],
			   ori.gamma/6.2832, ori.beta/6.2832, ori.alpha/6.2832);
    cfigs.push_back(v6);		    
  }
}


void Cfg::print_preamble_to_file(Environment* env, FILE* _fp, int numofCfg) {
  int pathVersion = env->GetPathVersion();
  fprintf(_fp,"VIZMO_PATH_FILE   Path Version %d\n", pathVersion);
  
  if(pathVersion<=PATHVER_20001022) {
    int numofLink = env->GetMultiBody(env->GetRobotIndex())->GetFreeBodyCount();
    fprintf(_fp,"%d\n", numofLink);
  }
  else fprintf(_fp,"1\n");
  
  fprintf(_fp,"%d ",numofCfg);    
}


const vector<double>& Cfg::GetData() const {
  return v;
}


int Cfg::DOF() const {
  return dof;
}


int Cfg::posDOF() const {
  return posDof;
}


// Return the range of a single parameter of the configuration (i.e., range of x)
// param = the parameter to get the range for
// In the future, this function should get the range for x,y,z by the bounding box
// Currently it assumes the range for a position parameter to be -10000 to 10000
// and the range for an orientation parameter to be 0 to 1, which should
// also be changed to reflect any self collision in a linked robot
pair<double,double> Cfg::SingleParamRange(int param) {
  pair<double,double> range;
  
  if ((param>=0) && (param<dof)) {
    if ((param>=0) && (param<posDof)) {
      range.first=-10000;
      range.second=10000;
    } else {
      range.first=0;
      range.second=1;
    }
  } else {
    cout << "Error in Cfg::SingleParamRange, out of range! \n";
    exit(1);
  }
  return range;
}


// Set a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to set the parameter as
int Cfg::SetSingleParam(int param, double value) {    
  if ((param>=0) && (param<dof)) {
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
  if ((param>=0) && (param<dof)) {
    v[param]+=value;
    Normalize_orientation(param);
    return 1;
  } else {
    return 0;
  } 
}


// Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to retreive
double Cfg::GetSingleParam(int param) const {
  if ((param>=0) && (param<dof)) {
    return v[param];
  } else {
    return 0;
  }
}


vector<double> Cfg::GetPosition() const {
  vector<double> ret;  
  for(int i=0; i<posDof; ++i) {
    ret.push_back(v[i]);
  }
  return ret;
}


vector<double> Cfg::GetOrientation() const {
  vector<double> ret;
  for(int i=posDof; i<dof; ++i) {
    ret.push_back(v[i]);
  }     
  return ret;
}


double Cfg::PositionMagnitude() const {
  double result = 0.0;
  for(int i=0; i<posDof; ++i) 
    result += sqr(v[i]);
  return sqrt(result);
}


double Cfg::OrientationMagnitude() const {
  double result = 0.0;
  for(int i=posDof; i<dof; ++i) {
    result += v[i] > 0.5 ? sqr(1.0 - v[i]) : sqr(v[i]);
  }
  return sqrt(result);
}


void Cfg::InvalidData() {
  v.clear();
  for(int i=0; i<dof; ++i) {
    if(i<posDof)
      v.push_back(MAXFLOAT);
    else
      v.push_back(0.0);
  }
}


// tests whether or not robot in this configuration has every vertex inside
// the environment specified bounding box
bool Cfg::InBoundingBox(Environment *env) const {
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


void Cfg::c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d) {
 
  Cfg* tmp = CreateNewCfg();
  tmp->subtract(cfg2, cfg1);
  tmp->divide(*tmp, tmp->PositionMagnitude());
  tmp->multiply(*tmp, d);
  tmp->add(cfg1, *tmp);
  this->equals(*tmp);
  delete tmp;
}


void Cfg::GetResolutionCfg(Environment* env) {
  v.clear();
  double posRes = env->GetPositionRes();
  double oriRes = env->GetOrientationRes();
  
  for(int i=0;i<dof;i++)
    if(i<posDof) 
      v.push_back(posRes);
    else 
      v.push_back(oriRes);

  Normalize_orientation();
}


void Cfg::GetPositionOrientationFrom2Cfg(const Cfg& c1,
					 const Cfg& c2) {
  vector<double> _v;
  for(int i=0; i<dof; ++i) {
    if(i<posDof)
      _v.push_back(c1.v[i]);
    else
      _v.push_back(c2.v[i]);
  }
  v = _v;
  Normalize_orientation();
}


void Cfg::GetMovingSequenceNodes(const Cfg& other, double s, vector<Cfg*>& result) const {
  Cfg* tmp = this->CreateNewCfg();
  tmp->WeightedSum(*this, other, s);
  
  Cfg* s1 = tmp->CreateNewCfg();
  s1->GetPositionOrientationFrom2Cfg(*tmp, *this);
  
  Cfg* s2 = tmp->CreateNewCfg();
  s2->GetPositionOrientationFrom2Cfg(*tmp, other);
  
  tmp->equals(*this); 
  result.push_back(tmp);
  result.push_back(s1);
  result.push_back(s2);
  tmp = other.CreateNewCfg();
  result.push_back(tmp);
}


// pt1 & pt2 are two endpts of a line segment
// find the closest point to the current cfg on that line segment
// it could be one of the two endpoints of course
void Cfg::ClosestPtOnLineSegment(const Cfg& current, const Cfg& pt1, const Cfg& pt2) {
    
  Cfg* B = pt2.CreateNewCfg();
  B->subtract(pt2, pt1);

  Cfg* C = current.CreateNewCfg();
  C->subtract(current, pt1);

  double B_dot_C = 0;
  double B_squared = 0;
    
  ///Modified for VC
#if defined(_WIN32)
  using namespace std;
#endif
    
  vector<double>::const_iterator b, c;
  for (b = B->v.begin(), c = C->v.begin(); b < B->v.end(); ++b, ++c) {
    B_dot_C += (*b)*(*c);
    B_squared += (*b)*(*b);
  }
  
  if (B_dot_C <= 0) {
    //return pt1;
    this->equals(pt1);
  } else if (B_dot_C >= B_squared) {
    //return pt2;
    this->equals(pt2);
  } else {
    //return pt1 + B*(B_dot_C/B_squared);
    this->multiply(*B, B_dot_C/B_squared);
    this->add(pt1, *this);
  }

  delete B;
  delete C;
}


void Cfg::FindNeighbors(Environment* _env, Stat_Class& Stats,
			const Cfg& increment,
			CollisionDetection* cd, int noNeighbors,
			CDInfo& _cdInfo, 
			vector<Cfg*>& ret) {  
  vector<Cfg*> nList;
  vector<double> posOnly, oriOnly;

  std::string Callee(GetName());
  {std::string Method("Cfg::FindNeighbors(1)");Callee=Callee+Method;}
  
  /////////////////////////////////////////////////////////////////////
  //Push 2 cfgs into nList whose position or orientation is the same 
  //as increment
  Cfg* tmp = increment.CreateNewCfg();
  nList.push_back(tmp); 
  int i;
  for(i=0; i<dof; ++i) {
    if(i<posDof) {
      posOnly.push_back(increment.v[i]);
      oriOnly.push_back(0.0);
    } else {
      posOnly.push_back(0.0);
      oriOnly.push_back(increment.v[i]);
    }
  }
  Cfg* posCfg = this->CreateNewCfg(posOnly);
  Cfg* oriCfg = this->CreateNewCfg(oriOnly);
  nList.push_back(posCfg);
  nList.push_back(oriCfg);
  
  /////////////////////////////////////////////////////////////////////
  //Push dof cfgs into nList whose value in each dimension is the same
  //as or complement of increment
  
  // find close neighbour in every dimension.
  vector<double> oneDim;
  for(i=0; i< dof; i++)
    oneDim.push_back(0.0);
  
  Cfg* oneDimCfg, *oneDimCfgNegative;
  for(i=0; i< dof; i++) { 
    oneDim[i] = increment.v[i];

    oneDimCfg = this->CreateNewCfg(oneDim);
    nList.push_back(oneDimCfg);

    oneDimCfgNegative = this->CreateNewCfg();
    oneDimCfgNegative->negative(*oneDimCfg);
    nList.push_back(oneDimCfgNegative);

    oneDim[i] = 0.0;  // reset to 0.0
  }
  
  /////////////////////////////////////////////////////////////////////
  //Validate Neighbors
  if(noNeighbors > nList.size()) 
    noNeighbors = nList.size();
  for(i=0;i<(noNeighbors);i++) {
    Cfg* tmp = this->CreateNewCfg();
    tmp->add(*this, *(nList[i]));
    
    if(!this->AlmostEqual(*tmp) && 
       !tmp->isCollision(_env,Stats,cd,_cdInfo,true,&(Callee)) ) 
      ret.push_back(tmp);
    else
      delete tmp;
  }
}


void Cfg::FindNeighbors(Environment* _env, Stat_Class& Stats,
			const Cfg& goal, const Cfg& increment,
			CollisionDetection* cd, int noNeighbors,
			CDInfo& _cdInfo,
			vector<Cfg*>& ret) {
   vector<Cfg*> nList;  
   vector<double> posOnly, oriOnly;

  std::string Callee(GetName());
  {std::string Method("Cfg::FindNeighbors(2)");Callee=Callee+Method;}
  
   /////////////////////////////////////////////////////////////////////
   //Push 2 cfgs into nList whose position or orientation is the same 
   //as increment
   Cfg* tmp = increment.CreateNewCfg();
   nList.push_back(tmp);
   int i;
   for(i=0; i<dof; ++i) {
     if(i<posDof) {
       posOnly.push_back(increment.v[i]);
       oriOnly.push_back(0.0);
     } else {
       posOnly.push_back(0.0);
       oriOnly.push_back(increment.v[i]);
     }
   }
   Cfg* posCfg = this->CreateNewCfg(posOnly);
   Cfg* oriCfg = this->CreateNewCfg(oriOnly);
   nList.push_back(posCfg);
   nList.push_back(oriCfg);

   /////////////////////////////////////////////////////////////////////
   //Push dof cfgs into nList whose value in each dimension is the same
   //as or complement of increment

   // find close neighbour in every dimension.
   vector<double> oneDim;
   for(i=0; i< dof; i++)
     oneDim.push_back(0.0);
   Cfg* oneDimCfg, *oneDimCfgNegative;
   for(i=0; i< dof; i++) {
     oneDim[i] = increment.v[i];

     oneDimCfg = this->CreateNewCfg(oneDim);
     nList.push_back(oneDimCfg);
     
     oneDimCfgNegative = this->CreateNewCfg();
     oneDimCfgNegative->negative(*oneDimCfg);
     nList.push_back(oneDimCfgNegative);

     oneDim[i] = 0.0;  // reset to 0.0
   }

   /////////////////////////////////////////////////////////////////////
   //Validate Neighbors

   /* Need to modify following code for the future cfgs */
   if(noNeighbors > nList.size()) 
     noNeighbors = nList.size();
   for(i=0;i<noNeighbors;++i) {
     Cfg* tmp = this->CreateNewCfg();
     
     tmp->IncrementTowardsGoal(goal, *nList[i]); //The only difference~
     if(!this->AlmostEqual(*tmp) && 
        !tmp->isCollision(_env,Stats,cd,_cdInfo,true,&(Callee)) ) {
       ret.push_back(tmp);
     } else 
       delete tmp;
   }

   for(i=0; i<nList.size(); i++)
     delete nList[i];
}


void Cfg::Increment(const Cfg& _increment) {
  for(int i=0; i<v.size(); ++i)
    v[i] += _increment.v[i];
  Normalize_orientation();
  obst = -1;
  tag = -1;
  clearance = -1;
}


void Cfg::IncrementTowardsGoal(const Cfg &goal,
                               const Cfg &increment) {
  double tmp;
  int i;

  ///For Posiotn
  for(i=0; i<posDof; ++i) {
    //If the diff between goal and c is smaller than increment
    if( fabs(goal.v[i]-v[i]) < fabs(increment.v[i]))
      v[i] = goal.v[i];
    else
      v[i] += increment.v[i];
  }

  ///For Oirentation
  for(i=posDof; i<dof; ++i) {
    if(v[i] != goal.v[i]) {
      double orientationIncr = increment.v[i] < 0.5 ? increment.v[i] : 1-increment.v[i];
      tmp = DirectedAngularDistance(v[i], goal.v[i]);
      if(fabs(tmp) < orientationIncr) {
	v[i]=goal.v[i];
      } else {
	v[i] += increment.v[i];
	v[i] = v[i] - floor(v[i]);
      }
    }
  }
}


void Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal,
			int* n_ticks, double positionRes,
			double orientationRes) {
  Cfg* diff = _goal.CreateNewCfg();
  diff->subtract(_goal, _start);

  // adding two basically makes this a rough ceiling...
  *n_ticks = (int)max(diff->PositionMagnitude()/positionRes, 
		      diff->OrientationMagnitude()/orientationRes) + 2;
  delete diff;

  this->FindIncrement(_start, _goal, *n_ticks);
}


void Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal,
			int n_ticks) {
  vector<double> incr;
  for(int i=0; i<dof; ++i) {
    if(i<posDof) 
      incr.push_back((_goal.v[i] - _start.v[i])/n_ticks);
    else
      incr.push_back(DirectedAngularDistance(_start.v[i], _goal.v[i])/n_ticks);
  }
  
  v = incr;
  Normalize_orientation();
}


// generates random configuration where workspace robot's EVERY VERTEX
// is guaranteed to lie within the environment specified bounding box
void Cfg::GetRandomCfg(Environment* env, int maxTries) {
  
  // Probably should do something smarter than 3 strikes and exit.
  // eg, if it fails once, check size of bounding box vs robot radius
  // and see if user has an impossibly small (for this robot) bounding
  // box specified
  
  // Since the argument of GetRandomCfg_CenterOfMass is changed, we do
    // not need define boudingBox here     //dawenx
    // double *bb = env->GetBoundingBox();
 
  obst = -1;
  tag = -1;
  clearance = -1;

  while (maxTries-- > 0) {
    this->GetRandomCfg_CenterOfMass(env);
   
    if (this->InBoundingBox(env))
      return;
  }//endwhile
  
  
  // Print error message and some helpful (I hope!) statistics and exit...
  cout << "\n\nERROR: GetRandomCfg not able to find anything in bounding box."
       <<   "\n       robot radius is "
       << env->GetMultiBody(env->GetRobotIndex())->GetBoundingSphereRadius();
  env->DisplayBoundingBox(cout);
  exit(-1);
}



// ditto, but with a default number of tries (10)
void Cfg::GetRandomCfg(Environment* env) {
  int default_maxTries = 100;
  this->GetRandomCfg(env, default_maxTries);

  obst = -1;
  tag = -1;
  clearance = -1;
}


// generates a random cfg with a given length 
void Cfg::GetRandomCfg(Environment* env, DistanceMetric* dm, double length) {
  vector<double> originVector;
  for(int i=0; i<dof; i++)
    originVector.push_back(0.0);
  Cfg* origin = this->CreateNewCfg(originVector);
  Cfg* outsideCfg = this->CreateNewCfg();
  outsideCfg->GetRandomCfg(env,1000);
  
  // first find an outsite configuration with sufficient size
  for(; dm->Distance(env, *origin, *outsideCfg) < 2*length; outsideCfg->multiply(*outsideCfg,2)); 
  
  // now, using binary search  find a configuration with the approximate
  // length
  Cfg* aboveCfg = outsideCfg->CreateNewCfg();
  Cfg* belowCfg = origin->CreateNewCfg();
  Cfg* currentCfg = this->CreateNewCfg();
  while (1) {
    currentCfg->add(*aboveCfg, *belowCfg);
    currentCfg->multiply(*currentCfg, 0.5);
    /*cout <<"Above " << aboveCfg << endl;
      cout <<"Current " << currentCfg << endl;
      cout <<"Below " << belowCfg << endl; */
    double magnitude = dm->Distance(env, *origin, *currentCfg);
    double diff = dm->Distance(env, *aboveCfg, *belowCfg);
    cout << "Current magnitude is \n" << magnitude << " " << length
	 << " " << diff << endl;
    if( (magnitude >= length*0.9) && (magnitude <= length*1.1)) {
      break;
    }
    if(magnitude>length) {
      aboveCfg->equals(*currentCfg);
    } else { 
      belowCfg->equals(*currentCfg); 
    }
  }
  v = currentCfg->v;
  delete currentCfg;
  delete origin;
  delete outsideCfg;
  delete aboveCfg;
  delete belowCfg;

  obst = -1;
  tag = -1;
  clearance = -1;
}


void Cfg::GetFreeRandomCfg(Environment* env, Stat_Class& Stats,
			   CollisionDetection* cd, 
			   CDInfo& _cdInfo) {

  std::string Callee(GetName());
  {std::string Method("Cfg::GetFreeRandomCfg");Callee=Callee+Method;}
  do {
    this->GetRandomCfg(env);
  } while ( this->isCollision(env, Stats, cd, _cdInfo,true,&Callee) );
  
}


void Cfg::GetNFreeRandomCfgs(vector<Cfg*>& nodes, Environment* env,
			     Stat_Class& Stats, CollisionDetection* cd,  
			     CDInfo& _cdInfo, int num) const {
  Cfg* tmp;
  std::string Callee(GetName());
  {std::string Method("Cfg::GetNFreeRandomCfgs");Callee=Callee+Method;}
  for(int i=0; i<num; ++i) {
    tmp = this->CreateNewCfg();
    do {
      tmp->GetRandomCfg(env);
    } while ( tmp->isCollision(env, Stats, cd, _cdInfo,true,&(Callee)) );
    nodes.push_back(tmp);
   }
};


// generates random configuration and then pushes it to the medial axis of
// the free c-space
void Cfg::GetMedialAxisCfg(Environment* _env, Stat_Class& Stats,
			   CollisionDetection* _cd,
			   CDInfo& _cdInfo, DistanceMetric* _dm,
			   int clearance_n, int penetration_n) { 
    this->GetRandomCfg(_env);
    this->PushToMedialAxis(_env, Stats, _cd, _cdInfo, _dm, 
			   clearance_n, penetration_n);
}


// pushes node towards c-space medial axis
void Cfg::PushToMedialAxis(Environment *_env, Stat_Class& Stats,
			   CollisionDetection *cd,
			     CDInfo& cdInfo, DistanceMetric *dm,
			     int clearance_n, int penetration_n) {
  std::string Callee(GetName()),CallCnt("1");
  {std::string Method("-Cfg::PushToMedialAxis");Callee=Callee+Method;}
    if(this->isCollision(_env, Stats, cd, cdInfo,true, &(Callee))) {
      ClearanceInfo clearInfo;
      this->ApproxCSpaceClearance2(_env, Stats, cd, cdInfo, dm, 
				   penetration_n, clearInfo, 1);
      this->v = (clearInfo.getDirection())->v;
      //delete clearInfo.getDirection();
    }
    CallCnt="2";
    std::string tmpStr = Callee+CallCnt;
    if(!(this->isCollision(_env, Stats, cd, cdInfo,true,&tmpStr))) {
      this->MAPRMfree(_env, Stats, cd, cdInfo, dm, clearance_n);
    }
}


// pushes free node towards c-space medial axis
void Cfg::MAPRMfree(Environment* _env, Stat_Class& Stats, 
		    CollisionDetection* cd,
		    CDInfo& cdInfo, DistanceMetric* dm,
		    int n) {
  Cfg* cfg = this;
  Cfg* newCfg, *oldCfg, *dir;
    
  ClearanceInfo clearInfo;
  cfg->ApproxCSpaceClearance2(_env, Stats, cd, cdInfo, dm,
			      n, clearInfo, 0);
  cfg->clearance = clearInfo.getClearance();
  dir = clearInfo.getDirection();
  //delete clearInfo.getDirection();
    
  int i = 0;
  double stepSize = cfg->clearance;
  
  //oldCfg = cfg;
  oldCfg = cfg->CreateNewCfg();
  //newCfg = oldCfg;
  newCfg = oldCfg->CreateNewCfg();

  /// find max. clearance point by stepping out:
  while ((newCfg->clearance >= oldCfg->clearance) && (newCfg->InBoundingBox(_env))) {    
    delete oldCfg;
    oldCfg = newCfg->CreateNewCfg();
    newCfg->c1_towards_c2(*newCfg, *dir, stepSize*-1);
    newCfg->clearance = newCfg->ApproxCSpaceClearance(_env,Stats,cd,cdInfo,dm,n,0);
    
    stepSize = newCfg->clearance;
   
    i++;
  }
  
  if (newCfg->InBoundingBox(_env)) {
    /// binary search betwen oldCfg and newCfg to find max clearance:
    Cfg* midCfg = this->CreateNewCfg();
    double minDistance = _env->GetPositionRes() + _env->GetOrientationRes();
    int maxNumSteps = 200; //arbitrary
    
    do {
      //midpoint between newCfg and oldCfg
      midCfg->add(*newCfg, *oldCfg);
      midCfg->divide(*midCfg, 2);
      midCfg->clearance = midCfg->ApproxCSpaceClearance(_env,Stats,cd,cdInfo,dm,n,0);

      if (midCfg->clearance > oldCfg->clearance) {
	oldCfg->equals(*midCfg);
      } else {
	newCfg->equals(*midCfg);
      }
      
      i++;
    } while ((dm->Distance(_env, *newCfg, *oldCfg) > minDistance) && (i <= maxNumSteps));
    delete midCfg;
  }
  
  this->equals(*oldCfg);

  delete oldCfg;
  delete newCfg;
}


// pushes colliding node towards free space
void Cfg::MAPRMcollision(Environment* _env, Stat_Class& Stats,
			 CollisionDetection* cd,
			 CDInfo& cdInfo, int n) {
  Cfg* cfg = this;
  double stepSize = 0.5;
  std::string Callee(GetName()),CallCnt("1");
  {std::string Method("-Cfg::MAPRMcollision");Callee=Callee+Method;}
  
  ///pick n random directions:
  vector<Cfg*> directions;
  vector<Cfg*> steps;
  Cfg* tmp, *tmp2;
  for (int i=0; i<n; i++) {
    tmp = this->CreateNewCfg();
    tmp->GetRandomCfg(_env);
    directions.push_back(tmp);

    tmp2 = cfg->CreateNewCfg();
    steps.push_back(tmp2);
  }  
  
  //if (directions.size()==0)
  //  this = cfg;
    
  ///step out along each direction:
  int found = -1;
  while (found < 0) {
    for (int i=0; i<directions.size(); i++) {
      steps[i]->c1_towards_c2(*steps[i], *directions[i], stepSize);
      if (!(steps[i]->isCollision(_env, Stats, cd, cdInfo,true,&(Callee)))) {
	found = i;
	break;
      }
    }
    stepSize = stepSize * 2;
  }
  
  this->equals(*steps[found]);
}


double Cfg::Clearance(Environment *env, Stat_Class& Stats,
		      CollisionDetection *cd ) const {
  if(!ConfigEnvironment(env))
    return -1;
  return cd->Clearance(env, Stats);
}


//Approximate C-Space Clearance
double Cfg::ApproxCSpaceClearance(Environment* env, Stat_Class& Stats,
				  CollisionDetection *cd, CDInfo& cdInfo,
                                  DistanceMetric* dm, 
				  int n, bool bComputePenetration) const {
  ClearanceInfo clearInfo;
  ApproxCSpaceClearance2(env, Stats, cd, cdInfo, dm, n, clearInfo, bComputePenetration);
  //delete clearInfo.getDirection(); //free direction memory, allocated in ApproxCSpaceClearance2
  return clearInfo.getClearance();
}


//Approximate C-Space Clearance
void Cfg::ApproxCSpaceClearance2(Environment* env, Stat_Class& Stats,
				 CollisionDetection* cd, CDInfo& cdInfo,
				 DistanceMetric* dm, 
				 int n, ClearanceInfo& clearInfo,
				 bool bComputePenetration) const {
  Cfg* cfg = this->CreateNewCfg();
  
  vector<Cfg*> directions;
  double dist = 100 * env->GetPositionRes();
  int i;
  Cfg* tmp;
  std::string Callee(GetName()),CallCnt;
  {std::string Method("Cfg::ApproxCSpaceClearance2");Callee=Callee+Method;}

  for (i=0; i<n; i++) {
    tmp = this->CreateNewCfg();
    tmp->GetRandomRay(dist);
    directions.push_back(tmp);
  }
  if (directions.size() == 0) { //unable to generate random directions
    delete tmp;
    delete cfg;
    return;
  }
    
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
    
  //if collide, set to true. Otherwise, set to false
  CallCnt="1";
  std::string tmpStr = Callee+CallCnt;
  bool bInitState = cfg->isCollision( env, Stats, cd, cdInfo,
				      true, &tmpStr );
  
  if( bComputePenetration == false && bInitState == true ) { //don't need to compute clearance
    delete cfg;
    for(i=0; i<directions.size(); i++)
      delete directions[i];
    return;
  }
  if( bComputePenetration == true && bInitState == false ) { //don't need to compute penetration
    delete cfg;
    for(i=0; i<directions.size(); i++)
      delete directions[i];
    return;
  }

  //find max step size:
  int iRobot = env->GetRobotIndex();
  double incrBound = env->GetMultiBody(iRobot)->GetMaxAxisRange(); //max step size    
  
  vector<Cfg*> tick;
  vector<Cfg*> incr;
  vector<double> incrSize;
  vector<bool> ignored;
  int num_ignored = 0;
  int n_ticks;

  Cfg* tmp2;
  for (i=0; i<directions.size(); i++) {
    tmp = cfg->CreateNewCfg();
    tick.push_back(tmp);

    tmp2 = cfg->CreateNewCfg();
    tmp2->FindIncrement(*cfg, *directions[i], &n_ticks, positionRes, orientationRes);
    incr.push_back(tmp2);

    incrSize.push_back(incr[i]->OrientationMagnitude() + incr[i]->PositionMagnitude());

    ignored.push_back(false);
  }

  int stateChangedFlag = false;
  
  //step out along each direction:
  while ( !stateChangedFlag ) {
    for (int i=0; i<directions.size(); i++) {
      tick[i]->Increment(*incr[i]);
      
      if (bComputePenetration) { //finding penetration
        CallCnt="2";
	tmpStr = Callee+CallCnt;
	if (!ignored[i] && 
	    (tick[i]->isCollision(env, Stats, cd, cdInfo,
	                          true,&tmpStr) != bInitState)) {
	  if (!(tick[i]->InBoundingBox(env))) { //ignore this direction
	    ignored[i] = true;
	    num_ignored++;
	    if (num_ignored == directions.size()) { //if no more directions left, exit loop
	      clearInfo.setClearance(10000);
	      
	      Cfg* tmp3 = cfg->CreateNewCfg();
	      clearInfo.setDirection(tmp3);

	      stateChangedFlag = true;
	    }
	  } else {
	    clearInfo.setClearance(dm->Distance(env, *tick[i], *cfg));
	    
	    Cfg* tmp3 = tick[i]->CreateNewCfg();
	    clearInfo.setDirection(tmp3);
	    
	    stateChangedFlag = true;
	  }
	}
      } else { //finding clearance
        CallCnt="3";
	tmpStr = Callee+CallCnt;
	if ( (tick[i]->isCollision(env, Stats, cd, cdInfo,true, &tmpStr) != bInitState) 
	     || !(tick[i]->InBoundingBox(env)) ) {
	  clearInfo.setClearance(dm->Distance(env, *tick[i], *cfg));
	  
	  Cfg* tmp3 = tick[i]->CreateNewCfg();
	  clearInfo.setDirection(tmp3);

	  stateChangedFlag = true;
	}
      }
      
      if (stateChangedFlag) {
	break; //exit for loop
      }
      
      //if increment still less than a max bound
      if (incrSize[i] < incrBound) {
	incr[i]->multiply(*incr[i], 2);
	incrSize[i] = incrSize[i] * 2;
      }	
      
    }//end for
  }//end while
  
  // if this cfg is free (state = false) then return smallestDistance (clearance)
  // if this cfg is not free (state = true) then return -smallestDistance (penetration)
  // clearInfo.setClearance((bInitState==false)?clearInfo.getClearance():-clearInfo.getClearance());
  clearInfo.setClearance(clearInfo.getClearance());

  delete cfg;
  for(i=0; i<directions.size(); i++)
    delete directions[i];
  for(i=0; i<tick.size(); i++)
    delete tick[i];
  for(i=0; i<incr.size(); i++)
    delete incr[i];

  return;
}


void Cfg::ApproxCSpaceContactPoints(vector<Cfg*>& directions, Environment* env,
				    Stat_Class& Stats, CollisionDetection* cd, 
				    CDInfo& cdInfo, vector<Cfg*>& contact_points) const {

  //initialize:
  int i;
  Cfg* origin;
  vector<double> originVector;
  std::string Callee(GetName()),CallCnt;
  {std::string Method("-Cfg::ApproxCSpanceContactPoints");Callee=Callee+Method;}
  
  for(i=0; i<dof; i++)
    originVector.push_back(0.0);
  for(i=0; i<directions.size(); i++) {
    origin = this->CreateNewCfg(originVector);
    contact_points.push_back(origin);
  }
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  CallCnt="1";
  std::string tmpStr = Callee+CallCnt;
  bool bInitState = origin->isCollision(env, Stats, cd, cdInfo,true,&tmpStr);
  
  //find max step size:
  int iRobot = env->GetRobotIndex();
  double incrBound = env->GetMultiBody(iRobot)->GetMaxAxisRange();
  
  //step out along each ray and record first cfg in collision:
  for(i=0; i<contact_points.size(); i++) {
    
    int n_ticks;
    Cfg* tick = origin->CreateNewCfg();
    Cfg* incr = this->CreateNewCfg();
    incr->FindIncrement(*origin, *directions[i], &n_ticks, positionRes, orientationRes);
    double incrSize = incr->OrientationMagnitude() + incr->PositionMagnitude(); //step size
    
    int stateChangedFlag = false;
    
    while(!stateChangedFlag) {
      
      tick->Increment(*incr);
      CallCnt="2";
      tmpStr = Callee+CallCnt;
      bool bCurrentState = tick->isCollision(env, Stats, cd, cdInfo,true, &tmpStr);
      //double currentDist;
      
      // if state was changed or this cfg is out of bounding box
      if( (bCurrentState != bInitState) || !(tick->InBoundingBox(env)) ) {
	//contact_points[i]->equals(*tick); //first cfg in collision
	contact_points[i]->subtract(*tick, *incr); //last free cfg
	stateChangedFlag = true;
      }
      
      //if increment still less than a max bound
      if(incrSize < incrBound) {
	incr->multiply(*incr, 2);
	incrSize *= 2;
      }
    }//end while
  }
}


bool Cfg::isCollision(Environment* env, Stat_Class& Stats,
		      CollisionDetection* cd, CDInfo& _cdInfo, 
		      bool enablePenetration, std::string *pCallName) {
  if(!this->ConfigEnvironment(env))
    return true;
  bool Clear = (pCallName) ? false : true; 
  if( !pCallName )
     pCallName = new std::string("isColl(e,s,cd,cdi,ep)");

  // after updating the environment(multibodies), Ask ENVIRONMENT
  // to check collision! (this is more nature.)
  bool answerFromEnvironment = cd->IsInCollision(env, Stats, _cdInfo, (MultiBody*)NULL, true, pCallName);
  if ( (answerFromEnvironment) && enablePenetration &&
       (cd->penetration>=0)) {
    Cfg* tmp = this->CreateNewCfg();
    bool result = !cd->AcceptablePenetration(*tmp, env, Stats, cd, _cdInfo);
    delete tmp;
    if( Clear ) delete pCallName;
    return result;
  }
  if( Clear ) delete pCallName;
  return answerFromEnvironment;
}


bool Cfg::isCollision(Environment* env, Stat_Class& Stats,
		      CollisionDetection* cd,
                      int robot, int obs, CDInfo& _cdInfo,
		      bool enablePenetration, std::string *pCallName) {
  if(!this->ConfigEnvironment(env))
    return true;
  bool Clear = (pCallName) ? false : true; 
  if( !pCallName )
     pCallName = new std::string("isColl(e,s,cd,r,o,cdi,ep)");

  
  // ask CollisionDetection class directly.
  bool answerFromCD = cd->IsInCollision(env, Stats, _cdInfo, robot, obs, pCallName);
  if ( (answerFromCD) && enablePenetration &&
       (cd->penetration>=0)) {
    Cfg* tmp = this->CreateNewCfg();
    bool result = !cd->AcceptablePenetration(*tmp, env, Stats, cd, _cdInfo);
    delete tmp;
    if(Clear) delete pCallName;
    return result;
  }
  if(Clear) delete pCallName;
  return answerFromCD;
}


bool Cfg::isCollision(Environment* env, Stat_Class& Stats,
		      CollisionDetection* cd,
		      CDInfo& _cdInfo, MultiBody* onflyRobot,
		      bool enablePenetration, std::string *pCallName) {
    this->ConfigEnvironment(env);
  bool Clear = (pCallName) ? false : true; 
  if( !pCallName )
     pCallName = new std::string("isColl(e,s,cd,cdi,mb,ep)");

    bool answer = cd->IsInCollision(env, Stats, _cdInfo, onflyRobot, true, pCallName);
    if ( (answer) && enablePenetration && (cd->penetration>=0)) {
      Cfg* tmp = this->CreateNewCfg();
      bool result = !cd->AcceptablePenetration(*tmp, env, Stats, cd, _cdInfo);
      delete tmp;
      if(Clear) delete pCallName;
      return result;
    }
    if(Clear) delete pCallName;
    return answer;
}


// Normalize the orientation to the some range.
void Cfg::Normalize_orientation(int index) {
  if(index == -1) {
    for(int i=posDof; i<v.size(); ++i)
      v[i] = v[i] - floor(v[i]);
  } else if(index >= posDof && index < dof) {  // orientation index
    v[index] = v[index] - floor(v[index]);
  } 
}
