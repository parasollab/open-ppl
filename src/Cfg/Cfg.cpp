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
#include "Stat_Class.h"
#include "util.h"
#include "ValidityChecker.hpp"
#include "CfgTypes.h"
#include "MPProblem.h"

//class Cfg_free;
//typedef Cfg_free CfgType;

int Cfg::NumofJoints;


#ifdef COLLISIONCFG
extern  vector < vector < vector <  double > > > CollisionConfiguration;
#endif

#define EQ(a,b)  (fabs(a-b)<0.0001)

ClearanceInfo::
~ClearanceInfo() {
  if(direction != NULL)
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
  for(size_t i=0; i<c1.v.size(); ++i)
    _v.push_back(c1.v[i]+c2.v[i]);
  v = _v;
  Normalize_orientation();
}


void Cfg::subtract(const Cfg& c1, const Cfg& c2) {
  vector<double> _v;
  for(size_t i=0; i<c1.v.size(); ++i)
    _v.push_back(c1.v[i]-c2.v[i]);
  v = _v;
  Normalize_orientation();
}


void Cfg::negative(const Cfg& c) {
  vector<double> _v;    
  for(vector<double>::const_iterator V = c.v.begin(); V != c.v.end(); ++V)
    _v.push_back(-(*V));
  v = _v;
  Normalize_orientation();
}


void Cfg::multiply(const Cfg& c, double s) {
  vector<double> _v;
  for(vector<double>::const_iterator V = c.v.begin(); V != c.v.end(); ++V)
    _v.push_back((*V)*s);
  v = _v;
  Normalize_orientation();
}


void Cfg::divide(const Cfg& c, double s) {
  vector<double> _v;
  for(vector<double>::const_iterator V = c.v.begin(); V != c.v.end(); ++V)
    _v.push_back((*V)/s);
  v = _v;
  Normalize_orientation();
}


void Cfg::WeightedSum(const Cfg& first, const Cfg& second, double weight) {
  vector<double> _v;
  for(size_t i=0; i<first.v.size(); ++i)
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
  for(vector<double>::const_iterator V = v.begin(); V != v.end(); ++V)
    os << setw(4)<<*V<<' ';
}


void Cfg::WriteInfo(ostream &os) const {
  os << obst << " ";
  os << tag << " ";
  os << clearance << " ";
}


void Cfg::Read(istream &is) {
  for(vector<double>::iterator V = v.begin(); V != v.end(); ++V)
    is >> (*V);
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
  BoundingBox *bb =  env->GetBoundingBox();

  if(!bb->IfSatisfiesConstraints(v)) 
    return false;

  // @todo: if there are multiple robots, this needs to be changed.
  shared_ptr<MultiBody> robot = env->GetMultiBody(env->GetRobotIndex());  

  if (bb->GetClearance(GetRobotCenterPosition()) < robot->GetBoundingSphereRadius()) { //faster, loose check
    // Robot is close to wall, have a strict check.
    ConfigEnvironment(env); // Config the Environment(robot indeed).

    for(int m=0; m<robot->GetFreeBodyCount(); ++m) {
      Transformation &worldTransformation = robot->GetFreeBody(m)->GetWorldTransformation();
      
      GMSPolyhedron &bb_poly = robot->GetFreeBody(m)->GetBoundingBoxPolyhedron();
      bool bbox_check = true;
      for(vector<Vector3D>::const_iterator V = bb_poly.vertexList.begin(); V != bb_poly.vertexList.end(); ++V)
	if(!bb->IfSatisfiesConstraints(worldTransformation * (*V))) {
	  bbox_check = false;
	  break;
	}
      if(bbox_check) 
	continue;
      
      GMSPolyhedron &poly = robot->GetFreeBody(m)->GetPolyhedron();
      for(vector<Vector3D>::const_iterator V = poly.vertexList.begin(); V != poly.vertexList.end(); ++V)
        if(!bb->IfSatisfiesConstraints(worldTransformation * (*V)))
	  return false;      
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

void Cfg::GetMovingSequenceNodes(const Cfg& other, vector<double> s_value, vector<Cfg*>& result) const {
  Cfg* this_copy = this->CreateNewCfg();
  result.push_back(this_copy);

  Cfg* weighted_sum = this->CreateNewCfg();
  double o_weight = 0.0;
  for(vector<double>::const_iterator S = s_value.begin(); S != s_value.end(); ++S) {
    weighted_sum->WeightedSum(*this, other, *S);

    Cfg* s1 = this->CreateNewCfg();
    s1->GetPositionOrientationFrom2Cfg(*weighted_sum, *result.back());
    result.push_back(s1);

    o_weight += 1.0 / s_value.size();
    weighted_sum->WeightedSum(*this, other, o_weight);

    Cfg* s2 = this->CreateNewCfg();
    s2->GetPositionOrientationFrom2Cfg(*s1, *weighted_sum);
    result.push_back(s2);
  }

  Cfg* other_copy = other.CreateNewCfg();
  result.push_back(other_copy);

  delete weighted_sum;
} 

void Cfg::GetMovingSequenceNodes(const Cfg& other, double s, vector<Cfg*>& result) const {
  Cfg* this_copy = this->CreateNewCfg();
  result.push_back(this_copy);

  Cfg* weighted_sum = this->CreateNewCfg();
  weighted_sum->WeightedSum(*this, other, s);

  Cfg* s1 = this->CreateNewCfg();
  s1->GetPositionOrientationFrom2Cfg(*weighted_sum, *this);
  result.push_back(s1);

  Cfg* s2 = this->CreateNewCfg();
  s2->GetPositionOrientationFrom2Cfg(*weighted_sum, other);
  result.push_back(s2);

  Cfg* other_copy = other.CreateNewCfg();
  result.push_back(other_copy);

  delete weighted_sum;
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
  for(int i=0; i<dof; ++i) {
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
  for(int i=0; i< dof; i++)
    oneDim.push_back(0.0);
  
  Cfg* oneDimCfg, *oneDimCfgNegative;
  for(int i=0; i< dof; i++) { 
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
  if(noNeighbors > (int)nList.size()) 
    noNeighbors = nList.size();
  for(int i=0;i<(noNeighbors);i++) {
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
   if(noNeighbors > (int)nList.size()) 
     noNeighbors = nList.size();
   for(int i=0;i<noNeighbors;++i) {
     Cfg* tmp = this->CreateNewCfg();
     
     tmp->IncrementTowardsGoal(goal, *nList[i]); //The only difference~
     if(!this->AlmostEqual(*tmp) && 
        !tmp->isCollision(_env,Stats,cd,_cdInfo,true,&(Callee)) ) {
       ret.push_back(tmp);
     } else 
       delete tmp;
   }

   for(vector<Cfg*>::iterator I = nList.begin(); I != nList.end(); ++I)
     delete (*I);
}


void Cfg::Increment(const Cfg& _increment) {
  for(size_t i=0; i<v.size(); ++i)
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
  (env->GetBoundingBox())->Print(cout);
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
void Cfg::GetRandomCfg(Environment* env, shared_ptr<DistanceMetricMethod>dm, double length) {
  v = vector<double>(dof, 0);
  Cfg* origin = this->CreateNewCfg();

  GetRandomCfg(env,1000);
  dm->ScaleCfg(env, length, *origin, *this);
  
  Normalize_orientation();

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


// Generates random configuration and pushes it to the c-space medial axis
bool 
Cfg::
GetMedialAxisCfg(MPProblem* mp, Environment* _env, Stat_Class& Stats,
		 string _vc, CollisionDetection* _cd,
		 CDInfo& _cdInfo, string _dm,
		 int clearance_n, int penetration_n) { 
  
  //cout << "\nCfg::GetMedialAxisCfg !! NEW SAMPLE !!" << endl;
  // Save tmp to test agaist pushed configuration
  this->GetRandomCfg(_env);
  CfgType tmp = CfgType(*this);
  //cout << "Initial CFG: " << tmp << endl;
  this->PushToMedialAxis(mp, _env, Stats, _vc, _cd, _cdInfo, _dm, 
			 clearance_n, penetration_n);
  //cout << "bout to check if good in getmedialaxiscfg" << endl;
  // If not pushed, fail
  if (tmp == *this) {
    //cout << "***************************************************** FAILED Same Sample" << endl;
    return false;
  }
  //cout << "***************************************************** SUCCESS New Sample" << endl;
  return true;
}

// Pushes verified configuraion towards c-space medial axis
void 
Cfg::
PushToMedialAxis(MPProblem* mp, Environment *_env, Stat_Class& Stats,
		 string vc, CollisionDetection* cd, CDInfo& cdInfo, 
                 string dm, int clearance_n, int penetration_n) {

   // Check if valid
   string callee = "Cfg::PushToMedialAxis";
   bool valid_cfg = mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc), *this, _env, Stats, cdInfo, true, &callee);
   ClearanceInfo clearInfo;
   ClearanceInfo cInfo;
   // Not valid, move to make valid
   if(!valid_cfg) {
      //cout << " Cfg::PushToMedialAxis !! NOT VALID !! Make valid before moving... ";
      bool compPen = true;
      //cout << "Passing " << compPen << endl;
      this->ApproxCSpaceClearance(mp, _env, Stats, vc, cdInfo, dm, penetration_n, cInfo, compPen, 0);
      //cout << " Finished approx cspace..." << endl;
      //cout << " clearInfo: " << *cInfo.getDirection() << endl;
      if(cInfo.getDirection() == NULL)
         return;
      Cfg* tmp = this->CreateNewCfg();
      Cfg* tmp3 = this->CreateNewCfg();
      tmp->subtract(*cInfo.getDirection(), *this);
      this->add(*cInfo.getDirection(), *tmp);
      Cfg* tmp2=this->CreateNewCfg();
      //cout << " Cfg = clearInfo.getDirection()" << endl;
      //cout << "         Cfg: " << *this << endl;
      // Push to medial axis
      this->MAPRMfree(mp, _env, Stats, vc, cd, cdInfo, dm, clearance_n);
      if(*this == *tmp2)
         this->equals(*tmp3);
   } else {
      //cout << " Cfg::PushToMedialAxis Valid" << endl;
      // Push to medial axis
      this->MAPRMfree(mp, _env, Stats, vc, cd, cdInfo, dm, clearance_n);
   }
   //cout << "All Done PUSHTOMEDIALAXIS" << endl;
}


// Pushes configuration towards c-space medial axis
void 
Cfg::
MAPRMfree(MPProblem* mp, Environment* _env, Stat_Class& Stats, 
	  string vc, CollisionDetection* cd,
          CDInfo& cdInfo, string dm, int n) {

   shared_ptr <DistanceMetricMethod> _dm = mp->GetDistanceMetric()->GetDMMethod(dm);
   Cfg* cfg = this->CreateNewCfg();
   string Callee = "Cfg::MAPRMfree";

   // Approximate clearance
   ClearanceInfo clearInfo;
   //cout << "Cfg::MAPRMFree Approx \"this\" CFG clearance: " << endl;

   // Get direction towards closest c-obstacle
   Cfg* dir;//clearInfo.getDirection()->CreateNewCfg();
   int colliding_obst;
   this->ApproxCSpaceClearance(mp, _env, Stats, vc, cdInfo, dm, n, clearInfo, false);
   if(clearInfo.getDirection() == NULL) {
      //cout << "!! APPROX RETURNED NULL !!" << endl;
      return;
   }
   this->clearance = clearInfo.getClearance();
   dir = clearInfo.getDirection()->CreateNewCfg();
   colliding_obst = clearInfo.getObstacleId();

   dir->subtract(*dir, *this);
   //cout << "        Dir: " << *dir << endl;

   if (*dir != CfgType())
      _dm->ScaleCfg(_env, 1, *this, *dir);
   else
      return;

   // Find max clearance point by stepping out
   double stepSize = this->clearance;

   double positionRes = _env->GetPositionRes();
   double orientationRes = _env->GetOrientationRes();

   //cout << "pRes and oRes: " << positionRes << " " << orientationRes << endl;

   if (this->clearance < positionRes)
      stepSize = positionRes/10;
   //if (this->clearance < 0.25)
   //  stepSize = 0.25;

   Cfg* oldCfg = this->CreateNewCfg();
   Cfg* newCfg = this->CreateNewCfg();
   Cfg* tmpCfg = dir->CreateNewCfg();
   bool found_gap = false;

   while((stepSize > 0) && 
         (newCfg->clearance >= oldCfg->clearance)) {// && 
      //cout<<"stepsize::"<<stepSize<<endl;
      tmpCfg->multiply(*dir, -1*stepSize);
      tmpCfg->add(*newCfg, *tmpCfg);

      // Test if tmpCfg is valid                                                                                                                                                  
      bool valid_cfg = mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc), *tmpCfg, _env, Stats, cdInfo, true, &Callee);
      //cout << "Cfg::MAPRMFree tmpCfg: " << *tmpCfg << endl;
      //cout << "Valid and in BBX: " << valid_cfg << " " << tmpCfg->InBoundingBox(_env) << endl;
      // If valid, calculate clearance and shift, else reduce step till a valid gap is found                                                                                       
      if (valid_cfg && tmpCfg->InBoundingBox(_env)) {
         found_gap = true;
         tmpCfg->clearance = tmpCfg->ApproxCSpaceClearance(mp, _env, Stats, vc, cdInfo,
               dm, n, false, colliding_obst);

         if (tmpCfg->clearance > stepSize)
            stepSize = tmpCfg->clearance;
         else
            stepSize = stepSize*1.5;// base 1.25
         oldCfg->equals(*newCfg);
         newCfg->equals(*tmpCfg);
      } else {
         if (!found_gap) {
            //cout << "REDUCE STEP SIZE" << endl;
            stepSize = stepSize*0.9;// base 0.8                                                                                                                                      
            if(stepSize<1e-20)stepSize=0;
         }
         else
            stepSize = 0;
      }
      //cout<<"newCfg->clearance::"<<newCfg->clearance<<"\toldCfg->clearance::"<<oldCfg->clearance<<endl;
   }

   bool valid_cfg = mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc), *tmpCfg, _env, Stats, cdInfo, true, &Callee);
   //cout << "FINAL Valid and in BBX: " << valid_cfg << " " << newCfg->InBoundingBox(_env) << endl;

   delete tmpCfg;
   delete dir;

   oldCfg = this->CreateNewCfg();  
   bool no_peak = false;

   if(newCfg->clearance > 0 && newCfg->InBoundingBox(_env)) {

      // Modified binary search between oldCfg and newCfg to find max clearance:
      Cfg* midCfg = this->CreateNewCfg();
      Cfg* omidCfg = this->CreateNewCfg();
      Cfg* nmidCfg = this->CreateNewCfg();
      double minDistance = _env->GetPositionRes() + _env->GetOrientationRes();
      minDistance = 0.001; //arbitrary
      int maxNumSteps = 100; //arbitrary
      int i=0;

      do {
         // Midpoint between newCfg and oldCfg
         midCfg->add(*newCfg, *oldCfg);
         midCfg->divide(*midCfg, 2);
         //cout << "Cfg::MAPRMFree MidCFG: ";
         midCfg->clearance = midCfg->ApproxCSpaceClearance(mp, _env, Stats, vc, 
               cdInfo, dm, n, false);
         // Midpoint between newCfg and midCfg
         nmidCfg->add(*newCfg, *midCfg);
         nmidCfg->divide(*nmidCfg, 2);
         //cout << "Cfg::MAPRMFree MidNew: ";
         nmidCfg->clearance = nmidCfg->ApproxCSpaceClearance(mp, _env, Stats, vc, 
               cdInfo, dm, n, false);
         // Midpoint between oldCfg and midCfg
         omidCfg->add(*oldCfg, *midCfg);
         omidCfg->divide(*omidCfg, 2);
         //cout << "Cfg::MAPRMFree MidOld: ";
         omidCfg->clearance = omidCfg->ApproxCSpaceClearance(mp, _env, Stats, vc, 
               cdInfo, dm, n, false);

         //cout << "OldCFG Clearance = " << oldCfg->clearance  << " Location: " << *oldCfg  << endl;
         //cout << "MidOld Clearance = " << omidCfg->clearance << " Location: " << *omidCfg << endl;
         //cout << "MidCFG Clearance = " << midCfg->clearance  << " Location: " << *midCfg  << endl;
         //cout << "MidNew Clearance = " << nmidCfg->clearance << " Location: " << *nmidCfg << endl;
         //cout << "NewCfg Clearance = " << newCfg->clearance  << " Location: " << *newCfg  << endl;

         // Find slopes/differences in the clearances
         double d1 = omidCfg->clearance - oldCfg->clearance;
         double d2 = midCfg->clearance  - omidCfg->clearance;
         double d3 = nmidCfg->clearance - midCfg->clearance;
         double d4 = newCfg->clearance  - nmidCfg->clearance;

         // Determine which 2 define the peak 
         if (d1 > 0 && d2 < 0) {
            //cout << "Medial Axis Between OldCfg and MidCfg \n";
            newCfg->equals(*midCfg);
         }
         else if (d2 > 0 && d3 < 0) {
            //cout << "Medial Axis Between OldMid and NewMid \n";
            oldCfg->equals(*omidCfg);
            newCfg->equals(*nmidCfg);
         }
         else if (d3 > 0 && d4 < 0) {
            //cout << "Medial Axis Between MidCfg and NewCfg \n";
            oldCfg->equals(*midCfg);
         }
         else {
            //cout << "NO MEDIAL AXIS PEAK FOUND (bad c-space clearnace approx) \n";
            no_peak = true;
            break;
         }

      } while ((_dm->Distance(_env, *newCfg, *oldCfg) > minDistance) &&
            (++i <= maxNumSteps));

      delete midCfg;
      delete omidCfg;
      delete nmidCfg;
   }

   if (!no_peak)
      this->equals(*oldCfg);

   delete oldCfg;
   delete newCfg;

   //cout << "Finished..." << endl;
}


double Cfg::Clearance(Environment *env, Stat_Class& Stats,
      CollisionDetection *cd ) const {
   ConfigEnvironment(env);
   return cd->Clearance(env, Stats);
}


//Approximate C-Space Clearance

double
Cfg::
ApproxCSpaceClearance(MPProblem* mp, Environment* env, Stat_Class& Stats,
		      string vc, CDInfo& cdInfo, string m_dm, int n, 
		      //ClearanceInfo& clearInfo, 
		      bool bComputePenetration,
		      int ignore_obstacle) const {
  ClearanceInfo clearInfo;
  ApproxCSpaceClearance(mp, env, Stats, vc, cdInfo, m_dm, 
			n, clearInfo, bComputePenetration, ignore_obstacle);
  return clearInfo.getClearance();
}


//Approximate C-Space Clearance
void 
Cfg::
ApproxCSpaceClearance(MPProblem* mp, Environment* env, Stat_Class& Stats,
		      string vc, CDInfo& cdInfo, string m_dm, int n, 
		      ClearanceInfo& clearInfo, bool bComputePenetration,
		      int ignore_obstacle) const {

  bool change = false;
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  shared_ptr <DistanceMetricMethod> dm = mp->GetDistanceMetric()->GetDMMethod(m_dm);

  //cout << "In Cfg::ApproxCSpaceClearance ";// \n  Position/Orientation Resolutions: " << positionRes << "/" << orientationRes << endl;
  //cout << "n = " << n << ", m_dm = " << m_dm << " bComputePenetration = " << bComputePenetration << endl;
  //cout << "ClearInfo Stuff " << clearInfo.getClearance() << " " << clearInfo.getDirection() << endl;
  // Generate n random directions at a distance dist away from this
  double dist = 100 * min(positionRes, orientationRes);
  vector<Cfg*> directions;
  vector<Cfg*> cand_in;
  vector<Cfg*> cand_out;
  Cfg* tmp;
  double dist_from = 0.0;  

  for(int i=0; i<n; i++) {
    tmp = this->CreateNewCfg();
    tmp->GetRandomRay(dist, env, dm);
    directions.push_back(tmp);
  }
  
  // If unable to generate random directions, exit
  if (directions.size() == 0) { 
    tmp = NULL;
    delete tmp;
    return;
  }
  
  // Setup strings for tracking cd calls
  string callee = "Cfg::ApproxCSpaceClearance";
  
  // Check validity
  Cfg* cfg = this->CreateNewCfg();
  bool bInitState = mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc), *cfg, env, Stats, cdInfo, true, &callee);
  //cout << " Initial state (1 valid, 0 invalid): " << bInitState << endl;
  //cout << " Initial State     : " << bInitState << endl;
  //cout << " Compute Penetration: " << bComputePenetration << endl;
  
  if( bInitState == false) {
    //cout << "Penetration calculation" << endl;
    if (!bComputePenetration) {
      //cout << "!! DID NOT PASS IN B_COMPUTE_PENETRATION !!" << endl;
      bComputePenetration = true;
    }
  }
  //else
    //cout << "Regular calculation " << endl;

   // Find max step size:
   double incrBound = 
     env->GetMultiBody(env->GetRobotIndex())->GetMaxAxisRange(); //max step size    

   // Setup origin vertex
   Cfg* orig = this->CreateNewCfg();
   orig->subtract( *orig, *orig);

   //cout << "Setup step sizes..." << endl;

   // Setup stepsizes for each direction
   vector<Cfg*> tick;
   vector<pair<Cfg*,double> > incr;
   vector<Cfg*>::iterator I;
   for(I = directions.begin(); I != directions.end(); ++I) {
     tmp = cfg->CreateNewCfg();
     tick.push_back(tmp);
     tmp = cfg->CreateNewCfg();
     int n_ticks;
     tmp->FindIncrement(*cfg, **I, &n_ticks, positionRes, orientationRes);
     incr.push_back(make_pair(tmp,
			      tmp->OrientationMagnitude() +
			      tmp->PositionMagnitude()));
   }
   
   //cout << " Start walking along each direction..." << endl;

   // Step out along each direction:
   vector<bool> ignored(directions.size(), false);
   int stateChangedFlag = false;
   int lastLapIndex = -1;
   int iterations = 0;
   
   // MAJOR LOOP
   while(!stateChangedFlag) {
     iterations++;

     for(size_t i=0; i<directions.size(); ++i) {
       if(ignored[i])
	 continue;
       
       tick[i]->Increment(*incr[i].first);
       
       if(!(tick[i]->InBoundingBox(env))) { // Outside BBX
	 if (lastLapIndex !=i) {
	   Cfg* cand_o = tick[i]->CreateNewCfg();
	   Cfg* cand_i = tick[i]->CreateNewCfg();
	   cand_i->subtract(*cand_i, *incr[i].first);
	   cand_out.push_back(cand_o);
	   cand_in.push_back(cand_i);
	 } else
	   stateChangedFlag = true;
	 if (lastLapIndex == -1) // Set lastLapIndex to first out-of-bbx dir
	   lastLapIndex = i;
	 
       }
       else { // In BBX
	 if(ignore_obstacle != -1)
	   cdInfo.ResetVars();
	 bool bValid = mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc), *tick[i], env, Stats, cdInfo, true, &callee);
	 if((ignore_obstacle != -1) && (cdInfo.colliding_obst_index == ignore_obstacle)) { // Check if need to ignore obs
	   if (lastLapIndex != i) {
	     Cfg* cand_o = tick[i]->CreateNewCfg();
	     Cfg* cand_i = tick[i]->CreateNewCfg();
	     cand_i->subtract(*cand_i, *incr[i].first);
	     cand_out.push_back(cand_o);
	     cand_in.push_back(cand_i);
	   } else
	     stateChangedFlag = true;
	   if (lastLapIndex == -1)
	     lastLapIndex = i;
	   
	 } else {
	   if (bValid != bInitState) { // Validity state changed
	     //cout << "Collision Change (" << iterations << "," << i << ")" << endl;
	     if (lastLapIndex != i) {
	       Cfg* cand_o = tick[i]->CreateNewCfg();
	       Cfg* cand_i = tick[i]->CreateNewCfg();
	       cand_i->subtract(*cand_i, *incr[i].first);
	       cand_out.push_back(cand_o);
	       cand_in.push_back(cand_i);
	     } else
	       stateChangedFlag = true;
	     if (lastLapIndex == -1)
	       lastLapIndex = i;
	   }
	 }
       }

       // Once validity changes, exit loop
       if(stateChangedFlag)
	 break;
       
       // If increment still less than a max bound
       if(incr[i].second < incrBound) {
	 incr[i].first->multiply(*incr[i].first,2);
	 incr[i].second *= 2;
       }
       
     } // End for
   } // End while
   //cout << "Candidates Out/In: " << cand_out.size() << "/" << cand_in.size() << endl << flush;
   
   Cfg* innerCfg =  cand_in[0]->CreateNewCfg();
   Cfg* outerCfg = cand_out[0]->CreateNewCfg();
   Cfg* midCfg = this->CreateNewCfg();
   
   //cout << "Made New Cfgs" << endl;
   
   double minDistance = env->GetPositionRes() +
     env->GetOrientationRes();
   minDistance = 0.0001; //arbitrary                                                                                                                          
   int maxNumSteps = 200; //arbitrary                                                                                                                         
   
   clearInfo.setClearance(10000);
   
   for(size_t i=0; i<cand_out.size(); i++) {
     //cout << "Candidate " << i << endl;
     innerCfg =  cand_in[i]->CreateNewCfg();
     outerCfg = cand_out[i]->CreateNewCfg();
     midCfg   = this->CreateNewCfg();
     //cout << "Starting inn/min/out" << endl;
     //cout << *innerCfg->CreateNewCfg() << endl << *midCfg->CreateNewCfg() << endl << *outerCfg->CreateNewCfg() << endl;

     int j=0;
     do {
       //cout << "j = " << j << " dist = " << dm->Distance(env, *innerCfg, *outerCfg) << endl;
       // Calculate new midpoint                                                                                                                              
       midCfg->add(*innerCfg, *outerCfg);
       midCfg->divide(*midCfg, 2);
       bool midValid = mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc), *midCfg, env, Stats, cdInfo, true, &callee);
       //if (bComputePenetration)
	 //cout << "mid is " << midValid << endl;
       if (bComputePenetration) { // If pushing out of obstacle
	 if(midCfg->InBoundingBox(env)) {
	   if (midValid)
	     outerCfg->equals(*midCfg);
	   else
	     innerCfg->equals(*midCfg);
	 } else
	   outerCfg->equals(*midCfg);
       } else {
	 if(!(midCfg->InBoundingBox(env)) || !midValid) // Outside BBX or invalid
	   outerCfg->equals(*midCfg);
	 else
	   innerCfg->equals(*midCfg);
       }
     } while ((dm->Distance(env, *innerCfg, *outerCfg) > minDistance) &&
	      (++j <= maxNumSteps));
         
     //cout << " Found best of candidate..." << endl;

     // Determine if new clearance is better than existing solution
     //cout << " dist (origin to inner) = " << dm->Distance(env, *innerCfg, *cfg) << endl;
     //cout << " dist (origin to outer) = " << dm->Distance(env, *outerCfg, *cfg) << endl;
     //cout << " clearInfo.getClearance = " << clearInfo.getClearance() << endl;

     Cfg* tempDir;
     if (bComputePenetration) {
       //cout << "Made it into if using outer..." << endl;
       // Look for min of outer
       if (clearInfo.getClearance() > dm->Distance(env, *outerCfg, *cfg )) {
	 clearInfo.setClearance(dm->Distance(env, *outerCfg, *cfg));
	 tempDir = outerCfg->CreateNewCfg();
	 if ( tempDir->InBoundingBox(env) ) {
	   clearInfo.setDirection(tempDir);
	   //cout << "Dir set to tempDir..." << endl;
	 }
	 else {
	   clearInfo.setDirection(NULL);
	   //cout << "Dir set to NULL..." << endl;       
	 }
	 //cout << "Set clearInfo and tempDir..." << endl;
       }
     } else {
       //cout << "Made it into if using inner..." << endl;
       // Look for min of inner
       if (clearInfo.getClearance() > dm->Distance(env, *innerCfg, *cfg )) {
	 clearInfo.setClearance(dm->Distance(env, *innerCfg, *cfg));
	 tempDir = innerCfg->CreateNewCfg();
	 if ( tempDir->InBoundingBox(env) ) {
	   clearInfo.setDirection(tempDir);
	   //cout << "Dir set to tempDir..." << endl;
	 }
	 else {
	   clearInfo.setDirection(NULL);
	   //cout << "Dir set to NULL..." << endl;       
	 }
	 //cout << "Set clearInfo and tempDir..." << endl;
       }
     }
   }
   
   delete innerCfg;
   delete midCfg;
   delete outerCfg;

   // Print Out DIRECTION and CLEARANCE
   //if (clearInfo.getDirection() != NULL)
   //  cfg = clearInfo.getDirection()->CreateNewCfg();
   //cout << "  Final Direction: ";                                                                                                                           
   //cfg->Write(cout);                                                                                                                                        
   //cout << "\n  Clearance: " << clearInfo.getClearance() << endl;                                                                                           
   
   //clean up allocated memory                                                                                                                                
   //delete cfg;
   for(vector<Cfg*>::iterator I = directions.begin(); I != directions.end(); ++I)
     delete *I;
   for(vector<Cfg*>::iterator I = tick.begin(); I != tick.end(); ++I)
     delete *I;
   for(vector<pair<Cfg*, double> >::iterator I = incr.begin(); I != incr.end(); ++I)
     delete I->first;
   for(vector<Cfg*>::iterator I = cand_out.begin(); I != cand_out.end(); ++I)
     delete *I;
   for(vector<Cfg*>::iterator I = cand_in.begin(); I != cand_in.end(); ++I)
     delete *I;
   
   return;
}


void Cfg::ApproxCSpaceContactPoints(vector<Cfg*>& directions, Environment* env,
            Stat_Class& Stats, CollisionDetection* cd, 
            CDInfo& cdInfo, vector<Cfg*>& contact_points) const {

  //initialize:
  Cfg* origin;
  vector<double> originVector;
  std::string Callee(GetName()),CallCnt;
  {std::string Method("-Cfg::ApproxCSpanceContactPoints");Callee=Callee+Method;}
  
  for(int i=0; i<dof; i++)
    originVector.push_back(0.0);
  for(size_t i=0; i<directions.size(); i++) {
    origin = this->CreateNewCfg(originVector);
    contact_points.push_back(origin);
  }
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  CallCnt="1";
  std::string tmpStr = Callee+CallCnt;
  bool bInitState = !origin->InBoundingBox(env) || origin->isCollision(env, Stats, cd, cdInfo,true,&tmpStr);
  
  //find max step size:
  int iRobot = env->GetRobotIndex();
  double incrBound = env->GetMultiBody(iRobot)->GetMaxAxisRange();
  
  //step out along each ray and record first cfg in collision:
  for(size_t i=0; i<contact_points.size(); i++) {
    
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
      bool bCurrentState = !tick->InBoundingBox(env) || tick->isCollision(env, Stats, cd, cdInfo,true, &tmpStr);
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
  Stats.IncCfgIsColl(pCallName);

  ConfigEnvironment(env);
  bool Clear = (pCallName) ? false : true; 
  if( !pCallName )
     pCallName = new std::string("isColl(e,s,cd,cdi,ep)");

  // after updating the environment(multibodies), Ask ENVIRONMENT
  // to check collision! (this is more nature.)
  bool answerFromEnvironment = cd->IsInCollision(env, Stats, _cdInfo, shared_ptr<MultiBody>(), true, pCallName);

#ifdef COLLISIONCFG
  if(answerFromEnvironment)
  {
    CollisionConfiguration[_cdInfo.colliding_obst_index].push_back(v);
  }
#endif

  if ((answerFromEnvironment) && enablePenetration && (cd->penetration>=0))
  {
    Cfg* tmp = this->CreateNewCfg();
    bool result = !cd->AcceptablePenetration(*tmp, env, Stats, cd, _cdInfo);
    delete tmp;
    if( Clear ) 
      delete pCallName;
    { 
      SetLabel("VALID",!result); 
      return result; 
    }
  }
  if( Clear ) 
    delete pCallName;
  {
    SetLabel("VALID",!answerFromEnvironment); 
    return answerFromEnvironment;
  }
}


bool Cfg::isCollision(Environment* env, Stat_Class& Stats,
          CollisionDetection* cd,
                      int robot, int obs, CDInfo& _cdInfo,
          bool enablePenetration, std::string *pCallName) {
  Stats.IncCfgIsColl(pCallName);
  
  ConfigEnvironment(env);
  bool Clear = (pCallName) ? false : true;
  
  if( !pCallName )
     pCallName = new std::string("isColl(e,s,cd,r,o,cdi,ep)");

  
  // ask CollisionDetection class directly.
  bool answerFromCD = cd->IsInCollision(env, Stats, _cdInfo, robot, obs, pCallName);
#ifdef COLLISIONCFG
  if(answerFromCD)
    {
      CollisionConfiguration[_cdInfo.colliding_obst_index].push_back(v);
    }
#endif

  if ( (answerFromCD) && enablePenetration &&
       (cd->penetration>=0)) {
    Cfg* tmp = this->CreateNewCfg();
    bool result = !cd->AcceptablePenetration(*tmp, env, Stats, cd, _cdInfo);
    delete tmp;
    if(Clear) delete pCallName;
    {SetLabel("VALID",!result); return result;}
  }
  if(Clear) delete pCallName;
  {SetLabel("VALID",!answerFromCD); return answerFromCD;}
}


bool Cfg::isCollision(Environment* env, Stat_Class& Stats,
          CollisionDetection* cd,
          CDInfo& _cdInfo, shared_ptr<MultiBody> onflyRobot,
          bool enablePenetration, std::string *pCallName) {

  Stats.IncCfgIsColl(pCallName);
    this->ConfigEnvironment(env);
  bool Clear = (pCallName) ? false : true; 
  if( !pCallName )
     pCallName = new std::string("isColl(e,s,cd,cdi,mb,ep)");

  bool answer = cd->IsInCollision(env, Stats, _cdInfo, onflyRobot, true, pCallName);
#ifdef COLLISIONCFG
  if(answer)
    {
      CollisionConfiguration[_cdInfo.colliding_obst_index].push_back(v);
    }
#endif
    if ( (answer) && enablePenetration && (cd->penetration>=0)) {
      Cfg* tmp = this->CreateNewCfg();
      bool result = !cd->AcceptablePenetration(*tmp, env, Stats, cd, _cdInfo);
      delete tmp;
      if(Clear) delete pCallName;
      {SetLabel("VALID",!result); return result;}
    }
    if(Clear) delete pCallName;
    {SetLabel("VALID",!answer); return answer;}
}


// Normalize the orientation to the some range.
void Cfg::Normalize_orientation(int index) {
  if(index == -1) {
    for(size_t i=posDof; i<v.size(); ++i)
      v[i] = v[i] - floor(v[i]);
  } else if(index >= posDof && index < dof) {  // orientation index
    v[index] = v[index] - floor(v[index]);
  } 
}

int Cfg::getNumofJoints() {
  return NumofJoints;
}

void Cfg::setNumofJoints(int _numofjoints) {
  NumofJoints = _numofjoints;
}

bool Cfg::GetLabel(string in_strLabel) {
  #ifndef _PARALLEL
  if(IsLabel(in_strLabel))
  {
    return m_LabelMap[in_strLabel];
  }
  else
  {
    cout << "Cfg::GetLabel -- I cannot find Label =  " << in_strLabel << endl;
    exit(-1);
  }
  #else 
   cout << "Cfg::GetLabel -- Map define type not implemented yet"<< endl;
   exit(-1);
  #endif
  
}

bool Cfg::IsLabel(string in_strLabel) {
   bool label = false;
   #ifndef _PARALLEL
   if(m_LabelMap.count(in_strLabel) > 0)
    { label= true ; }
   else
    { label= false; }
   #endif
   return label;
}
 
void Cfg::SetLabel(string in_strLabel,bool in_bool) {
  #ifndef _PARALLEL
  m_LabelMap[in_strLabel] = in_bool;
  #endif
}


double Cfg::GetStat(string in_strStat) {
  #ifndef _PARALLEL
  if(IsStat(in_strStat))
  {
    return m_StatMap[in_strStat];
  }
  else
  {
    cout << "Cfg::GetStat -- I cannot find Stat =  " << in_strStat << endl;
    exit(-1);
  }
  #else 
  cout << "Cfg::GetStat -- Map define type not implemented yet"<< endl;
    exit(-1);
  #endif
  
}

bool Cfg::IsStat(string in_strStat) {
   bool stat = false;
   #ifndef _PARALLEL
   if(m_StatMap.count(in_strStat) > 0)
    { stat= true ; }
   else
    { stat=false; }
   #endif
   return stat;
}
 
void Cfg::SetStat(string in_strStat,double in_dStat) {
  #ifndef _PARALLEL
  m_StatMap[in_strStat] = in_dStat;
  #endif  
}

vector<Vector3D> Cfg::PolyApprox(Environment* env) const {
  vector<Vector3D> result;
  ConfigEnvironment(env);
  env->GetMultiBody(env->GetRobotIndex())->PolygonalApproximation(result);
  return result;
}

