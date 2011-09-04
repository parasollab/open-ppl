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
#include "MedialAxisUtility.h"

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
  shared_ptr<BoundingBox> bb =  env->GetBoundingBox();

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


void Cfg::FindNeighbors(MPProblem* mp, Environment* _env, Stat_Class& Stats,
      const Cfg& increment,
      string vc_method,
      int noNeighbors,
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
       mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), *tmp, _env, Stats, _cdInfo, true, &Callee) )
      ret.push_back(tmp);
    else
      delete tmp;
  }
}


void Cfg::FindNeighbors(MPProblem* mp, Environment* _env, Stat_Class& Stats,
      const Cfg& goal, const Cfg& increment,
      string vc_method,
      int noNeighbors,
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
	mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), *tmp, _env, Stats, _cdInfo, true, &Callee) ) {
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


void Cfg::GetFreeRandomCfg(MPProblem* mp, Environment* env, Stat_Class& Stats,
         string vc_method,
         CDInfo& _cdInfo) {

  std::string Callee(GetName());
  {std::string Method("Cfg::GetFreeRandomCfg");Callee=Callee+Method;}
  do {
    this->GetRandomCfg(env);
  } while (!(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), *this, env, Stats, _cdInfo, true, &Callee)));
  
}


void Cfg::GetNFreeRandomCfgs(MPProblem* mp, vector<Cfg*>& nodes, Environment* env,
           Stat_Class& Stats, 
	   string vc_method,
           CDInfo& _cdInfo, int num) const {
  Cfg* tmp;
  std::string Callee(GetName());
  {std::string Method("Cfg::GetNFreeRandomCfgs");Callee=Callee+Method;}
  for(int i=0; i<num; ++i) {
    tmp = this->CreateNewCfg();
    do {
      tmp->GetRandomCfg(env);
    } while (!(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), *tmp, env, Stats, _cdInfo, true, &Callee)));
    nodes.push_back(tmp);
   }
};

void Cfg::GetRandomRayPos(double incr, Environment* env) {
  // Create random positional ray and scale
  double length=abs(incr), dist=0.0;
  v.clear();
  for(int i=0; i<DOF(); ++i)
    if (i < posDOF()) {
      v.push_back( 2.0*OBPRM_drand() - 1.0 );
      dist += pow(v[i],2);
    } else v.push_back( 0.0 );
  this->multiply(*this,length/sqrt(dist));

  obst = -1;
  tag = -1;
  clearance = -1;
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

double Cfg::GetSmoothingValue(MPProblem* mp, Environment *_env,Stat_Class& Stats,
															string vc, CDInfo& cdInfo, string dm,int n, bool bl ){
	GetApproxCollisionInfo(mp,*((CfgType*)this),_env,Stats,cdInfo,vc,dm,n,n,true);
	return cdInfo.min_dist;
}
