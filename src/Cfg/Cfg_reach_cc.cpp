#include "Cfg_reach_cc.h"
#include <numeric>
#include "Environment.h"
#include "MultiBody.h"
#include "FreeBody.h"
#include "boost/lambda/lambda.hpp"
#include "boost/random.hpp"

#define TWO_PI 6.2831853072

int Cfg_reach_cc::NumofJoints;
Link* Cfg_reach_cc::link_tree = NULL;
vector<Link*> Cfg_reach_cc::actual_links;
double Cfg_reach_cc::rdres = 0.05;
double Cfg_reach_cc::gamma = 0.5;
bool Cfg_reach_cc::is_closed_chain = true;


Cfg_reach_cc::
Cfg_reach_cc() : Cfg_free_tree() {
}

Cfg_reach_cc::
Cfg_reach_cc(int _numofjoints) : Cfg_free_tree(_numofjoints) {
}

Cfg_reach_cc::
Cfg_reach_cc(const Vector6D& _v) : Cfg_free_tree(_v) {
}

Cfg_reach_cc::
Cfg_reach_cc(const vector<double>& _v) : Cfg_free_tree(_v) {
}

Cfg_reach_cc::
Cfg_reach_cc(const Cfg& c) : Cfg_free_tree(c) {
  link_lengths = ((Cfg_reach_cc&)c).link_lengths;
  link_orientations = ((Cfg_reach_cc&)c).link_orientations;
}

Cfg_reach_cc::
Cfg_reach_cc(double x, double y, double z, 
	     double roll, double pitch, double yaw) 
  : Cfg_free_tree(x, y, z, roll, pitch, yaw) {
}

Cfg_reach_cc::
Cfg_reach_cc(const Vector6D& base,
	     const vector<double>& len, 
	     const vector<int>& ori) :
  link_lengths(len), link_orientations(ori) {
  v.clear();
  v.push_back(base.getX());
  v.push_back(base.getY());
  v.push_back(base.getZ());
  v.push_back(base.getRoll());
  v.push_back(base.getPitch());
  v.push_back(base.getYaw());
  StoreData();
}

Cfg_reach_cc::
~Cfg_reach_cc() {
}

void
Cfg_reach_cc::
initialize_link_tree(const char* filename) {
  ifstream ifs(filename);
  char strData[256];
  if(!(ifs >> strData)) {
    cerr << "Error while reading link values: can't read num_links\n";
    exit(-1);
  }
  if(strcmp(strData, "numLinks") != 0) {
    cerr << "Error while reading link values: can't read num_links\n";
    exit(-1);
  }
  int num_links = 0;
  if(!(ifs >> num_links)) {
    cerr << "Error while reading link values: can't read num_links\n";
    exit(-1);
  }
  
  for(int i=0; i<num_links; ++i) {
    if(!(ifs >> strData)) {
      cerr << "Error while reading link " << i << " lengths\n";
      exit(-1);
    }
    if(strcmp(strData, "RealLink") != 0) {
      cerr << "Error while reading link " << i << " lengths\n";
      exit(-1);
    }
    int link_id = -1;
    if(!(ifs >> link_id)) {
      cerr << "Error while reading link " << i << " id\n";
      exit(-1);
    }
    double length1 = -1;
    if(!(ifs >> length1)) {
      cerr << "Error while reading link " << i << " min length\n";
      exit(-1);
    }
    double length2 = -1;
    if(!(ifs >> length2)) {
      cerr << "Error while reading link " << i << " max length\n";
      exit(-1);
    }
    actual_links.push_back(new Link(length1, length2));
  }
  ifs.close();

  link_tree = BuildTree(0, actual_links.size()-1, actual_links);
  //link_tree->PrintTree(cout);
}

void 
Cfg_reach_cc::
setNumofJoints(int _numofjoints) { 
  NumofJoints = _numofjoints; 
  Cfg_free_tree::setNumofJoints(_numofjoints);
  Cfg::setNumofJoints(_numofjoints);
}

void 
Cfg_reach_cc::
equals(const Cfg& c) {
  Cfg_free_tree::equals(c);
  link_lengths = ((Cfg_reach_cc&)c).link_lengths;
  link_orientations = ((Cfg_reach_cc&)c).link_orientations;
}

void 
Cfg_reach_cc::
add(const Cfg&, const Cfg&) {
  cerr << "Warning, add not implmeneted yet\n";
}

void 
Cfg_reach_cc::
subtract(const Cfg& c1, const Cfg& c2) {
  //cerr << "Warning, subtract not implemented yet\n";
  vector<double> _v1 = c1.GetData();
  vector<double> _v2 = c2.GetData();
  for(int i=0; i<6; ++i)
    v[i] = _v1[i] - _v2[i];

  link_lengths.clear();
  transform(((Cfg_reach_cc&)c1).link_lengths.begin(), ((Cfg_reach_cc&)c1).link_lengths.end(),
	    ((Cfg_reach_cc&)c2).link_lengths.begin(),
	    back_insert_iterator<vector<double> >(link_lengths),
	    minus<double>());

  //not sure what to do here...used for Increment in lp, so setting ori equal to c1 is probably ok
  link_orientations.clear();
  /*
  transform(((Cfg_reach_cc&)c1).link_orientations.begin(), ((Cfg_reach_cc&)c1).link_orientations.end(),
	    ((Cfg_reach_cc&)c2).link_orientations.begin(),
	    back_insert_iterator<vector<double> >(link_orientations),
	    minus<int>());
  */
  link_orientations = ((Cfg_reach_cc&)c1).link_orientations;


  StoreData();  
}

void 
Cfg_reach_cc::
negative(const Cfg& c) {
  //cerr << "Warning, negative not implemented yet\n";
  vector<double> _v = c.GetData();
  for(int i=0; i<6; ++i)
    v[i] = -1*_v[i];

  link_lengths.clear();
  transform(((Cfg_reach_cc&)c).link_lengths.begin(), ((Cfg_reach_cc&)c).link_lengths.end(),
	    back_insert_iterator<vector<double> >(link_lengths),
	    bind1st(multiplies<double>(), -1));

  //not sure what to do here...used for lp failed path, so setting ori equal ok
  link_orientations = ((Cfg_reach_cc&)c).link_orientations;

  StoreData();
}

void 
Cfg_reach_cc::
multiply(const Cfg&, double) {
  cerr << "Warning, multiply not implemented yet\n";
}

void 
Cfg_reach_cc::
divide(const Cfg&, double) {
  cerr << "Warning, divide not implemented yet\n";
}

void 
Cfg_reach_cc::
WeightedSum(const Cfg&, const Cfg&, double weight) {
  cerr << "Warning, WeightedSum not implmeneted yet\n";
}

void 
Cfg_reach_cc::
c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d) {
  cerr << "Warning, c1_towards_c2 not implemented yet\n";
}

bool 
Cfg_reach_cc::
isWithinResolution(const Cfg &c, 
		   double positionRes, 
		   double orientationRes) const {
  //if orienation difference is 2 or -2, return false
  for(size_t i=0; i<link_orientations.size(); ++i)
    if(abs(link_orientations[i]-((Cfg_reach_cc&)c).link_orientations[i]) > 1)
      return false;

  //true if reachable distance is less than rdres
  return (this->LengthDistance(c) <= rdres);
}


bool 
Cfg_reach_cc::
ConfigEnvironment(Environment* _env) const {
  int robot = _env->GetRobotIndex();
     
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 v[5]*TWOPI, 
						 v[4]*TWOPI, 
						 v[3]*TWOPI), // RPY
				     Vector3D(v[0],v[1],v[2]));
  
  _env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);  // update link 1.
  for(int i=0; i<NumofJoints; i++) {
    _env->GetMultiBody(robot)->GetFreeBody(i+1)
      ->GetBackwardConnection(0).GetDHparameters().theta = v[i+6]*360.0;
  }  // config the robot
  
  //_env->GetMultiBody(robot)->GetFreeBody(_env->GetMultiBody(robot)->GetFreeBodyCount()-1)->GetWorldTransformation();
  _env->GetMultiBody(robot)->GetFreeBody(0)->GetWorldTransformation();
  
  return true;
}

void 
Cfg_reach_cc::
GetRandomCfg(double R, double rStep) {
  cerr << "Warning GetRandomCfg not implemented yet\n";
}

void 
Cfg_reach_cc::
GetRandomCfg_CenterOfMass(Environment* env) {
  for(int i=0; i<6; ++i)
    v[i] = env->GetBoundingBox()->GetRandomValueInParameter(i);
  //fix to xz plane:
  //v[1] = 0;
  //v[3] = 0;
  //v[5] = 0;

  link_tree->ResetTree();
  if(is_closed_chain)
    link_tree->RecursiveSample(0, true, gamma);
  else
    link_tree->RecursiveSample(-1, true, gamma);
  link_lengths.clear();
  link_orientations.clear();
  link_tree->ExportTreeLinkLength(link_lengths, link_orientations);

  StoreData();
  
  obst = -1;
  tag = -1;
  clearance = -1;
}

void 
Cfg_reach_cc::
GetRandomRay(double incr, Environment* env, DistanceMetric* dm) {
  cerr << "Warning GetRandomRay not implemented yet\n";
}

void 
Cfg_reach_cc::
FindNeighbors(Environment* env, Stat_Class& Stats, const Cfg& increment,
	      CollisionDetection* cd, int noNeighbors, CDInfo& _cdInfo,
	      vector<Cfg*>& cfgs) {
  cerr << "Warning, FindNeighbors not implemented yet\n";
}

void 
Cfg_reach_cc::
FindNeighbors(Environment* env, Stat_Class& Stats, const Cfg& goal, 
	      const Cfg& increment, CollisionDetection* cd, int noNeighbors, 
	      CDInfo& _cdInfo, vector<Cfg*>& cfgs) {
  cerr << "Warning, FindNeighbors not implemented yet\n";
}

void 
Cfg_reach_cc::
Increment(const Cfg& _increment) {
  vector<double> _v = _increment.GetData();
  for(int i=0; i<6; ++i)
    v[i] += _v[i];
  transform(link_lengths.begin(), link_lengths.end(),
	    ((Cfg_reach_cc&)_increment).link_lengths.begin(),
	    link_lengths.begin(), 
	    plus<double>());
  link_orientations = ((Cfg_reach_cc&)_increment).link_orientations;
  StoreData();
  obst = -1;
  tag = -1;
  clearance = -1;
}

void 
Cfg_reach_cc::
IncrementTowardsGoal(const Cfg &goal, const Cfg &increment) {
  cerr << "Warning, IncrementTowardsGoal not implemented yet\n";
}

void 
Cfg_reach_cc::
FindIncrement(const Cfg& _start, const Cfg& _goal, int* n_ticks, 
	      double positionRes, double orientationRes) {
  //length_diff = _start.link_lengths - _goal.link_lengths
  vector<double> length_diff;
  transform(((Cfg_reach_cc&)_start).link_lengths.begin(), 
	    ((Cfg_reach_cc&)_start).link_lengths.end(),
	    ((Cfg_reach_cc&)_goal).link_lengths.begin(),
	    back_insert_iterator<vector<double> >(length_diff),
	    minus<double>());
  //mag = sqrt(sum of length_diff sqrs)
  double mag = sqrt(inner_product(length_diff.begin(), length_diff.end(),
				  length_diff.begin(), 0.0,
				  plus<double>(), multiplies<double>()));

  int rd_ticks = (int)(mag/rdres) + 2; //adding two makdes a rough ceiling...

  vector<double> v_g = _goal.GetData();
  Cfg_free g(v_g[0],v_g[1],v_g[2],v_g[3],v_g[4],v_g[5]);
  vector<double> v_s = _start.GetData();
  Cfg_free s(v_s[0],v_s[1],v_s[2],v_s[3],v_s[4],v_s[5]);
  g.subtract(g, s);
  int base_ticks = (int)max(g.PositionMagnitude()/positionRes,
			    g.OrientationMagnitude()/orientationRes) + 2;

  *n_ticks = max(rd_ticks, base_ticks);

  FindIncrement(_start, _goal, *n_ticks);
}

void 
Cfg_reach_cc::
FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks) {
  vector<double> v_start = _start.GetData();
  vector<double> v_goal = _goal.GetData();
  for(int i=0; i<6; ++i)
    v[i] = (v_goal[i]-v_start[i])/n_ticks;

  link_lengths.clear();
  Link::FindIncrement(((Cfg_reach_cc&)_start).link_lengths,
		      ((Cfg_reach_cc&)_goal).link_lengths,
		      link_lengths, n_ticks);

  link_orientations = ((Cfg_reach_cc&)_goal).link_orientations;
  for(size_t i=0; i<link_orientations.size(); ++i) {
    int _start_ori = ((Cfg_reach_cc&)_start).link_orientations[i];
    int _goal_ori = ((Cfg_reach_cc&)_goal).link_orientations[i];
    if(_start_ori != _goal_ori) {
      if((_start_ori != 0) && (_goal_ori != 0)) {
	cerr << "Warning in FindIncrement: orientations too far apart, use GetIntermediate first\n";
	exit(-1);
      }
      if(_start_ori != 0)
	link_orientations[i] = _start_ori;
      else
	link_orientations[i] = _goal_ori;
    }
  }

  StoreData();
}

Cfg* 
Cfg_reach_cc::
CreateNewCfg() const {
  Cfg* tmp = new Cfg_reach_cc(*this);
  return tmp;
}

Cfg* 
Cfg_reach_cc::
CreateNewCfg(vector<double>& _v) const {
  if((int)_v.size() < dof) {
    cout << "\n\nERROR in Cfg_reach_cc::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than dof\n";
    exit(-1);
  }
  vector<double> _data(_v.begin(), _v.begin()+dof);
  Cfg* tmp = new Cfg_reach_cc(_data);
  return tmp;
}

void
Cfg_reach_cc::
StoreData() {
  link_tree->ResetTree();
  link_tree->ImportTreeLinkLength(link_lengths, link_orientations, 0);

  if(link_tree->CanRecursiveClose()) 
  {
    v.resize(6);

    //compute joint angles
    double sumExtAng = 0;
    for(size_t i=1; i<actual_links.size(); ++i) {
      double extAng = PI - Link::CalculateJointAngle(actual_links[i-1], actual_links[i]);
      sumExtAng += extAng;
      v.push_back(extAng/TWO_PI);
    }
    if(is_closed_chain)
      v.push_back((TWO_PI-sumExtAng)/TWO_PI);

  } else {
//     cerr << "\n\n\tWARNING: Loop is broken!\n";
    v.resize(dof, 0);
  }

  Normalize_orientation();
}


bool 
Cfg_reach_cc::
GetIntermediate(const Cfg_reach_cc& c1,
		const Cfg_reach_cc& c2) {
  link_tree->ResetTree();

  vector<Range> ranges;
  vector<double>::const_iterator L2 = c2.link_lengths.begin();
  for(vector<double>::const_iterator L1 = c1.link_lengths.begin();
      L1 != c1.link_lengths.end(); ++L1, ++L2) 
    ranges.push_back(Range(*L1, *L2));
  link_tree->ImportTreeLinkAvailableRange(ranges);

  using boost::lambda::_1;
  using boost::lambda::_2;
  vector<int> avg_orientations;
  transform(c1.link_orientations.begin(), c1.link_orientations.end(),
	    c2.link_orientations.begin(),
	    back_insert_iterator<vector<int> >(avg_orientations),
	    ((_1 + _2)/2));
  link_tree->ImportTreeLinkConvexity(avg_orientations, 0);

  link_tree->RecursiveBuildAvailableRange(true);

  //compute deterministic seed for local planning/sampling
  double len = 0.0;
  for(size_t i=0; i<c1.link_lengths.size(); ++i) 
    len += c1.link_lengths[i]*(i+1)*(i+2) + c2.link_lengths[i]*(i+1)*(i+2);
  uint64_t seed = len +
    accumulate(c1.link_orientations.begin(), c1.link_orientations.end(), 0) + 
    accumulate(c2.link_orientations.begin(), c2.link_orientations.end(), 0);
  boost::rand48 generator(seed);
  boost::uniform_real<> distribution(0,1);
  boost::variate_generator<boost::rand48&, boost::uniform_real<> >
    rand(generator, distribution);
  
  bool can_recursive_sample = true;
  if(is_closed_chain)
    can_recursive_sample = link_tree->RecursiveSample(rand, 0);
  else
    can_recursive_sample = link_tree->RecursiveSample(rand, -1);

  if(!can_recursive_sample)
    return false;
  else {
    link_lengths.clear();
    link_orientations.clear();
    link_tree->ExportTreeLinkLength(link_lengths, link_orientations);
    StoreData();
    return true;
  }
}

double
Cfg_reach_cc::
LengthDistance(const Cfg_reach_cc& c2) const {
  vector<Range> ranges;
  link_tree->ExportTreeLinkReachableRange(ranges);
  /*
  cout << "ranges:";
  for(vector<Range>::const_iterator R = ranges.begin(); R != ranges.end(); ++R) 
    cout << " " << R->Size();
  cout << endl;
  */
  vector<double> length_difference;
  for(size_t i=0; i<link_lengths.size(); ++i) 
    if(ranges[i].Size() == 0)
      length_difference.push_back(0);
    else
      length_difference.push_back(fabs(link_lengths[i]-c2.link_lengths[i])/ranges[i].Size());
  /*
  cout << "length_difference: ";
  copy(length_difference.begin(), length_difference.end(), ostream_iterator<double>(cout, " "));
  cout << endl; 
  */

  return sqrt(inner_product(length_difference.begin(), length_difference.end(),
			    length_difference.begin(),
			    0.0, plus<double>(), multiplies<double>()));
}

double
Cfg_reach_cc::
OrientationDistance(const Cfg_reach_cc& c2) const {
  vector<double> ori_difference;
  for(size_t i=0; i<link_orientations.size(); ++i)
    ori_difference.push_back((double)(abs(link_orientations[i]-c2.link_orientations[i]))/2.0);
  /*
  cout << "ori_difference: ";
  copy(ori_difference.begin(), ori_difference.end(), ostream_iterator<double>(cout, " "));
  cout << endl;
  */

  return sqrt(inner_product(ori_difference.begin(), ori_difference.end(),
			    ori_difference.begin(),
			    0.0, plus<double>(), multiplies<double>()));
}

ostream&
Cfg_reach_cc::
print(ostream& os) const 
{
  os << "\tv: ";
  copy(v.begin(), v.end(), ostream_iterator<double>(os, " "));
  os << endl;

  os << "\tlengths: ";
  copy(link_lengths.begin(), link_lengths.end(), ostream_iterator<double>(os, " "));
  os << endl;

  os << "\torientations: ";
  copy(link_orientations.begin(), link_orientations.end(), ostream_iterator<int>(os, " "));
  os << endl;

  return os;
}



ostream&
Cfg_reach_cc::
print_base(ostream& os) const 
{
  copy(v.begin(), v.begin()+6, ostream_iterator<double>(os, " "));
  return os;
}



ostream&
Cfg_reach_cc::
print_len(ostream& os) const 
{
  copy(link_lengths.begin(), link_lengths.end(), ostream_iterator<double>(os, " "));
  return os;
}



ostream&
Cfg_reach_cc::
print_ori(ostream& os) const 
{
  copy(link_orientations.begin(), link_orientations.end(), ostream_iterator<int>(os, " "));
  return os;
}
