#include "Cfg_reach_cc.h"
#include <numeric>
#include "Environment.h"
#include "MultiBody.h"
#include "FreeBody.h"
#include  "DistanceMetrics.h"
#include "boost/lambda/lambda.hpp"
#include "boost/random.hpp"
#include "DistanceMetricMethod.h"
#include "MPProblem.h"
#include "ValidityChecker.h"

Link* Cfg_reach_cc::link_tree = NULL;
vector<Link*> Cfg_reach_cc::actual_links;
double Cfg_reach_cc::rdres = 0.05;
double Cfg_reach_cc::gamma = 0.5;
bool Cfg_reach_cc::is_closed_chain = true;


Cfg_reach_cc::
Cfg_reach_cc() : Cfg_free_tree() {
}

Cfg_reach_cc::Cfg_reach_cc(const Vector6D& base,
    const vector<double>& len, const vector<int>& ori) :
  link_lengths(len), link_orientations(ori) {
    m_v.clear();
    m_v.push_back(base.getX());
    m_v.push_back(base.getY());
    m_v.push_back(base.getZ());
    m_v.push_back(base.getRoll());
    m_v.push_back(base.getPitch());
    m_v.push_back(base.getYaw());
    StoreData();
  }

Cfg_reach_cc::
Cfg_reach_cc(const Cfg& c) : Cfg_free_tree(c) {
  link_lengths = ((Cfg_reach_cc&)c).link_lengths;
  link_orientations = ((Cfg_reach_cc&)c).link_orientations;
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
  link_tree->PrintTree(cout);
}

Cfg&
Cfg_reach_cc::
operator=(const Cfg& _c) {
  link_lengths = ((Cfg_reach_cc&)_c).link_lengths;
  link_orientations = ((Cfg_reach_cc&)_c).link_orientations;
  return Cfg::operator=(_c);
}

void
Cfg_reach_cc::
add(const Cfg& c1, const Cfg& c2) {

  if(OrientationsDifferent((Cfg_reach_cc&)c1, (Cfg_reach_cc&)c2))
  {

    vector<int>::const_iterator I = ((Cfg_reach_cc&)c1).link_orientations.begin();
    vector<int>::const_iterator J = ((Cfg_reach_cc&)c2).link_orientations.begin();
    for(; I != ((Cfg_reach_cc&)c1).link_orientations.end() && J != ((Cfg_reach_cc&)c2).link_orientations.end(); ++I, ++J)
      if(abs(*I - *J) > 1)
      {
        cerr << "\n\nError in Cfg_reach_cc::add, adding cfgs with too great an orientation difference, exiting.\n";
        cerr << "\tc1 = "; ((Cfg_reach_cc&)c1).print(cerr); cerr << endl;
        cerr << "\tc2 = "; ((Cfg_reach_cc&)c2).print(cerr); cerr << endl;
        exit(-1);
      }
  }

  vector<double> _v1 = c1.GetData();
  vector<double> _v2 = c2.GetData();
  if(m_dof != m_numOfJoints)
    for(int i=0; i<6; ++i)
      m_v[i] = _v1[i] + _v2[i];

  link_lengths.clear();

  transform(((Cfg_reach_cc&)c1).link_lengths.begin(), ((Cfg_reach_cc&)c1).link_lengths.end(),
      ((Cfg_reach_cc&)c2).link_lengths.begin(),
      back_insert_iterator<vector<double> >(link_lengths),
      plus<double>());

  link_orientations.clear();


  for(size_t i=0; i<min(((Cfg_reach_cc&)c1).link_orientations.size(),
        ((Cfg_reach_cc&)c2).link_orientations.size()); ++i)
  {
    if(  ((Cfg_reach_cc&)c1).link_orientations[i] == ((Cfg_reach_cc&)c2).link_orientations[i] )
      link_orientations.push_back(  ( (Cfg_reach_cc&)c1).link_orientations[i] );
    else if ( ((Cfg_reach_cc&)c1).link_orientations[i]!=0)
      link_orientations.push_back(  ( (Cfg_reach_cc&)c1).link_orientations[i] );
    else
      link_orientations.push_back(  ( (Cfg_reach_cc&)c2).link_orientations[i] );
  }

  StoreData();

}
void
Cfg_reach_cc::
subtract(const Cfg& c1, const Cfg& c2) {
  //cerr << "Warning, subtract not implemented yet\n";
  //cout << "DEBUG::Cfg_reach_cc::subtract\n";
  //cout << "\tc1 = "; ((Cfg_reach_cc&)c1).print(cout); cout << endl;
  //cout << "\tc2 = "; ((Cfg_reach_cc&)c2).print(cout); cout << endl;
  if(OrientationsDifferent((Cfg_reach_cc&)c1, (Cfg_reach_cc&)c2))
  {
    //cout << "\t\t-> orientation different\n";
    vector<int>::const_iterator I = ((Cfg_reach_cc&)c1).link_orientations.begin();
    vector<int>::const_iterator J = ((Cfg_reach_cc&)c2).link_orientations.begin();
    for(; I != ((Cfg_reach_cc&)c1).link_orientations.end() && J != ((Cfg_reach_cc&)c2).link_orientations.end(); ++I, ++J)
      if(abs(*I - *J) > 1)
      {
        cerr << "\n\nError in Cfg_reach_cc::subtract, subtracting cfgs with too great an orientation difference, exiting.\n";
        cerr << "\tc1 = "; ((Cfg_reach_cc&)c1).print(cerr); cerr << endl;
        cerr << "\tc2 = "; ((Cfg_reach_cc&)c2).print(cerr); cerr << endl;
        exit(-1);
      }
  }

  vector<double> _v1 = c1.GetData();
  vector<double> _v2 = c2.GetData();
  if(m_dof != m_numOfJoints)
    for(int i=0; i<6; ++i)
      m_v[i] = _v1[i] - _v2[i];

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
  //cout << "DEBUG::Cfg_reach_cc::negative\n";
  //cout << "\this = "; print(cout); cout << endl;
  //cout << "\tc = "; ((Cfg_reach_cc&)c).print(cout); cout << endl;
  if(OrientationsDifferent(*this, (Cfg_reach_cc&)c))
  {
    //cout << "\t\t-> orientation different\n";
    vector<int>::const_iterator I = link_orientations.begin();
    vector<int>::const_iterator J = ((Cfg_reach_cc&)c).link_orientations.begin();
    for(; I != link_orientations.end() && J != ((Cfg_reach_cc&)c).link_orientations.end(); ++I, ++J)
      if(abs(*I - *J) > 1)
      {
        cerr << "\n\nError in Cfg_reach_cc::negative, negating cfgs with too great an orientation difference, exiting.\n";
        cerr << "\tthis = "; print(cerr); cerr << endl;
        cerr << "\tc = "; ((Cfg_reach_cc&)c).print(cerr); cerr << endl;
        exit(-1);
      }
  }

  vector<double> _v = c.GetData();
  if(m_dof != m_numOfJoints)
    for(int i=0; i<6; ++i)
      m_v[i] = -1*_v[i];

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
multiply(const Cfg&, double, bool _norm) {
  cerr << "Warning, multiply not implemented yet\n";
  exit(-1);
}

void
Cfg_reach_cc::
divide(const Cfg&, double) {
  cerr << "Warning, divide not implemented yet\n";
  exit(-1);
}

void
Cfg_reach_cc::
WeightedSum(const Cfg& c1, const Cfg& c2, double weight) {
  //check if the input weight parameter is between 0 and 1
  if(!((weight >= 0) && (weight <= 1)))
  {
    cerr << "\n\n Error in Cfg_reach_cc::WeightedSum, weight is not feasible, exiting.\n";
    exit(-1);
  }
  //check if link orientations differ by more than concave/convec <-> flat
  if(OrientationsDifferent((Cfg_reach_cc&)c1, (Cfg_reach_cc&)c2))
  {
    vector<int>::const_iterator I = ((Cfg_reach_cc&)c1).link_orientations.begin();
    vector<int>::const_iterator J = ((Cfg_reach_cc&)c2).link_orientations.begin();
    for(; I != ((Cfg_reach_cc&)c1).link_orientations.end() && J != ((Cfg_reach_cc&)c2).link_orientations.end(); ++I, ++J)
    {
      if(abs(*I - *J) > 1)
      {
        cerr << "\n\nError in Cfg_reach_cc::WeightedSum, weighting cfgs with too great an orientation difference, exiting.\n";
        cerr << "\tc1 = "; ((Cfg_reach_cc&)c1).print(cerr); cerr << endl;
        cerr << "\tc2 = "; ((Cfg_reach_cc&)c2).print(cerr); cerr << endl;
        exit(-1);
      }
    }
  }

  vector<double> _v1 = c1.GetData();
  vector<double> _v2 = c2.GetData();
  if(m_dof != m_numOfJoints)
    for(int i=0; i<6; ++i)
      m_v[i] = _v1[i]*(1-weight) + _v2[i]*weight;

  //compute link lengths
  link_lengths.clear();
  vector<double>::const_iterator I = ((Cfg_reach_cc&)c1).link_lengths.begin();
  vector<double>::const_iterator J = ((Cfg_reach_cc&)c2).link_lengths.begin();
  for(; I != ((Cfg_reach_cc&)c1).link_lengths.end() && J != ((Cfg_reach_cc&)c2).link_lengths.end(); ++I, ++J)
  {
    link_lengths.push_back((*I)*(1-weight) + (*J)*weight);
  }

  //compute link orientations
  link_orientations.clear();

  for(size_t i=0; i<min((((Cfg_reach_cc&)c1).link_orientations.size()), ((Cfg_reach_cc&)c2).link_orientations.size()); ++i)
  {
    if((((Cfg_reach_cc&)c1).link_orientations[i]) == (((Cfg_reach_cc&)c2).link_orientations[i]))
      link_orientations.push_back(((Cfg_reach_cc&)c1).link_orientations[i]);
    //take the non-flat value from first cfg or second cfg
    else if (((Cfg_reach_cc&)c1).link_orientations[i] != 0)
      link_orientations.push_back(((Cfg_reach_cc&)c1).link_orientations[i]);
    else
      link_orientations.push_back(((Cfg_reach_cc&)c2).link_orientations[i]);
  }

  //convert lengths and orientations into joint angles
  StoreData();
}

void
Cfg_reach_cc::
c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d) {
  cerr << "Warning, c1_towards_c2 not implemented yet\n";
  exit(-1);
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
ConfigureRobot(Environment* _env) const {
  Cfg_free_tree::ConfigureRobot(_env);
  vector<int> link_ids;
  for(int i=0; i<_env->GetMultiBody(m_robotIndex)->GetFreeBodyCount(); ++i)
    link_ids.push_back( _env->GetMultiBody(m_robotIndex)->GetFreeBodyIndex( _env->GetMultiBody(m_robotIndex)->GetFreeBody(i) ) );
  set<int> visited;
  for(vector<int>::const_iterator L = link_ids.begin(); L != link_ids.end(); ++L)
    _env->GetMultiBody(m_robotIndex)->GetFreeBody(*L)->ComputeWorldTransformation(visited);
  return true;
}

void
Cfg_reach_cc::GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb) {
  if(m_dof != m_numOfJoints) {
    Point3d p = _bb->GetRandomPoint();
    for(int i=0 ;i<3;i++){
      m_v[i] = p[i];
    }
    for(int i=3; i<6; ++i)
      m_v[i] = _bb->GetRandomValueInParameter(i-3);
  }

  link_tree->ResetTree();
  if(is_closed_chain)
    link_tree->RecursiveSample(0, true, gamma);
  else
    link_tree->RecursiveSample(-1, true, gamma);
  link_lengths.clear();
  link_orientations.clear();
  link_tree->ExportTreeLinkLength(link_lengths, link_orientations);

  StoreData();
}

void
Cfg_reach_cc::
GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm)
{

  int n_ticks= 0;
  double positionRes=env->GetPositionRes();
  double orientationRes=env->GetOrientationRes();

  vector<int>origin_link_orientations;
  Cfg_reach_cc c1;
  c1.GetRandomCfg(env);
  // link_orientations = ((Cfg_reach_cc&)c1).link_orientations;
  Cfg_reach_cc c1_origin;

  //StoreData();
  vector<Range> ranges;
  vector<double>origin_link_lengths;
  link_tree->ExportTreeLinkReachableRange(ranges);

  for(size_t i=0; i<ranges.size(); ++i)
  {
    origin_link_lengths.push_back((ranges[i].min + ranges[i].max)*0.5);
    origin_link_orientations.push_back(0);

  }
  Cfg_reach_cc origin(Vector6D(0,0,0,0,0,0),origin_link_lengths,origin_link_orientations);
  Cfg_reach_cc incr2;

  c1_origin.GetIntermediate(origin,c1);

  incr2.FindIncrement(origin,c1_origin,&n_ticks,positionRes,orientationRes);

  Cfg_reach_cc tick =origin;
  while(dm->Distance(env,origin,tick)< incr)
  {
    // if(dm->Distance(env,origin,tick) > 0.3)
    //{
    //incr2.GetIntermediate(origin,c1);
    //tick.Increment(incr2);
    // }
    //else
    tick.Increment(incr2);

  }

  // dm->ScaleCfg(incr, tick, origin);
  *this=tick;

}


Vector3d Cfg_reach_cc::GetRobotCenterofMass(Environment* env) const {
 cerr << "Warning, GetRobotCenterofMass not implemented yet\n";
  exit(-1);
}

void
Cfg_reach_cc::
Increment(const Cfg& _increment) {
  //cout << "DEBUG::Cfg_reach_cc::Increment\n";
  //cout << "\tthis = "; print(cout); cout << endl;
  //cout << "\t_increment = "; ((Cfg_reach_cc&)_increment).print(cout); cout << endl;
  if(OrientationsDifferent(*this, (Cfg_reach_cc&)_increment))
  {
    //cout << "\t\t-> orientation different\n";
    vector<int>::const_iterator I = link_orientations.begin();
    vector<int>::const_iterator J = ((Cfg_reach_cc&)_increment).link_orientations.begin();
    for(; I != link_orientations.end() && J != ((Cfg_reach_cc&)_increment).link_orientations.end(); ++I, ++J)
      if(abs(*I - *J) > 1)
      {
        cerr << "\n\nError in Cfg_reach_cc::Increment, incrementing cfgs with too great an orientation difference, exiting.\n";
        cerr << "\tthis = "; print(cerr); cerr << endl;
        cerr << "\tc = "; ((Cfg_reach_cc&)_increment).print(cerr); cerr << endl;
        exit(-1);
      }
  }

  vector<double> _v = _increment.GetData();
  if(m_dof != m_numOfJoints)
    for(int i=0; i<6; ++i)
      m_v[i] += _v[i];
  transform(link_lengths.begin(), link_lengths.end(),
      ((Cfg_reach_cc&)_increment).link_lengths.begin(),
      link_lengths.begin(),
      plus<double>());
  link_orientations = ((Cfg_reach_cc&)_increment).link_orientations;

  StoreData();
}

void
Cfg_reach_cc::
IncrementTowardsGoal(const Cfg &goal, const Cfg &increment) {
  cerr << "Warning, IncrementTowardsGoal not implemented yet\n";
  exit(-1);
}

void
Cfg_reach_cc::
FindIncrement(const Cfg& _start, const Cfg& _goal, int* n_ticks,
    double positionRes, double orientationRes, double rd_res) {
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
  int rd_ticks = (int)(mag/rd_res) + 2; //adding two makdes a rough ceiling...

  CfgType g;
  g.subtract(_goal, _start);
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
  if(m_dof != m_numOfJoints)
    for(int i=0; i<6; ++i)
      m_v[i] = (v_goal[i]-v_start[i])/n_ticks;

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

void
Cfg_reach_cc::
StoreData() {
  link_tree->ResetTree();
  link_tree->ImportTreeLinkLength(link_lengths, link_orientations, 0);

  if(link_tree->CanRecursiveClose())
  {
    if(m_dof != m_numOfJoints)
      m_v.resize(6);
    else
      m_v.resize(0);
    //compute joint angles
    double sumExtAng = 0;
    for(size_t i=1; i<actual_links.size(); ++i) {
      double extAng = PI - Link::CalculateJointAngle(actual_links[i-1], actual_links[i]);
      sumExtAng += extAng;
      double a = extAng/TWO_PI;
      a= a - floor(a);
      if(a>=0.5)a-=1.0;
      m_v.push_back(a);
    }
    if(is_closed_chain)
      m_v.push_back((TWO_PI-sumExtAng)/TWO_PI);

  } else {
    //  cerr << "\n\n\tWARNING: Loop is broken!\n";
    m_v.resize(m_dof, 0);
  }

  NormalizeOrientation();
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
    vector<double> v1 = c1.GetData();
    vector<double> v2 = c2.GetData();
    if(m_dof != m_numOfJoints)
      for(int i=0; i<6; ++i)
        m_v[i] = (v1[i] + v2[i]) / 2;
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
  //cout<<"in lenght dist "<<c2.link_lengths.size()<<"__"<<endl;
  vector<Range> ranges;

  link_tree->ExportTreeLinkReachableRange(ranges);


  /*cout << "ranges:";
    for(vector<Range>::const_iterator R = ranges.begin(); R != ranges.end(); ++R)
    cout << " " << R->Size();
    cout << endl;*/

  if(link_lengths.size() != c2.link_lengths.size())
  {
    cerr << "\n\nERROR in Cfg_reach_cc::LengthDistance, link_lengths.size (" << link_lengths.size() << ") != c2.link_lengths.size (" << c2.link_lengths.size() << ", exiting.\n";
    exit(-1);
  }

  vector<double> length_difference;
  for(size_t i=0; i<link_lengths.size(); ++i) {

    if(ranges[i].Size() == 0){

      length_difference.push_back(0);
    }else{

      length_difference.push_back(fabs(link_lengths[i]-c2.link_lengths[i])/ranges[i].Size());
    }

  }
  /*cout << "length_difference: ";
    copy(length_difference.begin(), length_difference.end(), ostream_iterator<double>(cout, " "));
    cout << endl; */


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

  return sqrt(inner_product(ori_difference.begin(), ori_difference.end(),
        ori_difference.begin(),
        0.0, plus<double>(), multiplies<double>()));
}

ostream&
Cfg_reach_cc::
print(ostream& os) const
{
  os << "\tv: ";
  copy(m_v.begin(), m_v.end(), ostream_iterator<double>(os, " "));
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
  copy(m_v.begin(), m_v.begin()+6, ostream_iterator<double>(os, " "));
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






/**********************************
 *
 *
 * Code extracted from old ClosedChainStrategy that handles sampling of multiple loops.
 * To do: add this functionality back to reachable cfg class here
 *
 *
void ClosedChainStrategy::Solve(){
  /////
  // From ClosedChainStrategy::ParseXML, setps up relevant variables
  is_gamma_random=false;
  num_iterations  = in_Node.Read("iterations", false, int(10), int(1), int(100000), "number of iterations");
  gamma = in_Node.Read("gamma", false, double(0.0), double(0.0), double(1.0), "gamma used by reachable code");
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  CfgType::gamma = gamma;
#endif
  ////

  cout<<"in ClosedChainStrategy::Solve()"<<endl;
  //num_iterations=10;//debugging code
  cout << "Sampling " << num_iterations << " configurations...\n";
  double sampling_time = 0.0, collision_time;
  int attempts = 0;
  //MultiBody mb;
  //boost::shared_ptr<MultiBody> subsetOfRobot;
  for(int count = 0; count < num_iterations; ++count){
    bool colliding = true;
    while(colliding)
    {
      if(is_gamma_random)
        gamma = drand48();
      //cout << "\tgamma = " << gamma << "...\n";

      //sample closed cfg
      bool bSampleSucceeded = false;
      Clock_Class GenClock;
      GenClock.StartClock("Node generation");
      while(!bSampleSucceeded)
      {
        attempts++;

        //re-initialize cfg
        for(size_t i=0; i<CCProblem->g_loopRoots.size(); ++i)
          CCProblem->g_loopRoots[i]->ResetTree();
        CCProblem->ConfigBase(CCProblem->GetEnvironment());
        //ConfigureRobot(&env);

        bSampleSucceeded = true;
        for(size_t i=0; i< CCProblem->g_loopRoots.size(); ++i)
        {
          //sample closed loop

          //set ear constraint
          double ear_length = 0;
          if(!CCProblem->g_non_ears[i].empty())
          {
            //compute ear required length
            triple<int,int,int> ear_joint1 = *(find_if(CCProblem->g_earJoints.begin(), CCProblem->g_earJoints.end(), first_is<int, triple<int,int,int> >(CCProblem->g_ears[i].front())));
            triple<int,int,int> ear_joint2 = *(find_if(CCProblem->g_earJoints.begin(), CCProblem->g_earJoints.end(), first_is<int, triple<int,int,int> >(CCProblem->g_ears[i].back())));
            // *
            cout << "setting ear constraint for loop " << i << endl;
            cout << "\tear_joint1: " << ear_joint1.first << "," << ear_joint1.second << "," << ear_joint1.third << endl;
            cout << "\tear_joint2: " << ear_joint2.first << "," << ear_joint2.second << "," << ear_joint2.third << endl;
            * //

            Vector3d joint1, joint2;
            {
            //note, following assumes link orientated along x-axis for longest length
            //get world coords for joint1.second
	      GMSPolyhedron& joint1_bbox1 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint1.second)->GetWorldBoundingBox();
            //cout << "\tjoint1_bbox1:\n";
            //for_each(joint1_bbox1.vertexList, joint1_bbox1.vertexList + joint1_bbox1.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");

            Vector3d joint1_bbox1_endpoint1;
            for(int j=0; j<4; ++j)
              joint1_bbox1_endpoint1 = joint1_bbox1_endpoint1 + joint1_bbox1.vertexList[j];
            joint1_bbox1_endpoint1 = joint1_bbox1_endpoint1 / 4;
            //cout << "\tjoint1_bbox1_endpoint1: " << joint1_bbox1_endpoint1 << endl;

            Vector3d joint1_bbox1_endpoint2;
            for(int j=4; j<8; ++j)
              joint1_bbox1_endpoint2 = joint1_bbox1_endpoint2 + joint1_bbox1.vertexList[j];
            joint1_bbox1_endpoint2 = joint1_bbox1_endpoint2 / 4;
            //cout << "\tjoint1_bbox1_endpoint2: " << joint1_bbox1_endpoint2 << endl;

            //get world coords for joint1.third
            GMSPolyhedron& joint1_bbox2 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint1.third)->GetWorldBoundingBox();
            //cout << "\tjoint1_bbox2:\n";
            //for_each(joint1_bbox2.vertexList, joint1_bbox2.vertexList + joint1_bbox2.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");

            Vector3d joint1_bbox2_endpoint1;
            for(int j=0; j<4; ++j)
              joint1_bbox2_endpoint1 = joint1_bbox2_endpoint1 + joint1_bbox2.vertexList[j];
            joint1_bbox2_endpoint1 = joint1_bbox2_endpoint1 / 4;
            //cout << "\tjoint1_bbox2_endpoint1: " << joint1_bbox2_endpoint1 << endl;

            Vector3d joint1_bbox2_endpoint2;
            for(int j=4; j<8; ++j)
              joint1_bbox2_endpoint2 = joint1_bbox2_endpoint2 + joint1_bbox2.vertexList[j];
            joint1_bbox2_endpoint2 = joint1_bbox2_endpoint2 / 4;
            //cout << "\tjoint1_bbox2_endpoint2: " << joint1_bbox2_endpoint2 << endl;

            //find intersection coord
            Vector3d a = joint1_bbox1_endpoint1 - joint1_bbox2_endpoint1;
            Vector3d b = joint1_bbox1_endpoint1 - joint1_bbox2_endpoint2;
            Vector3d c = joint1_bbox1_endpoint2 - joint1_bbox2_endpoint1;
            Vector3d d = joint1_bbox1_endpoint2 - joint1_bbox2_endpoint2;
            //cout << "\t\ta: " << a << "\tb: " << b << "\tc: " << c << "\td: " << d << endl;

            if(a.magnitude() <= b.magnitude() && a.magnitude() <= c.magnitude() && a.magnitude() <= d.magnitude())
              joint1 = (joint1_bbox1_endpoint1 + joint1_bbox2_endpoint1)/2;
            else if(b.magnitude() <= a.magnitude() && b.magnitude() <= c.magnitude() && b.magnitude() <= d.magnitude())
              joint1 = (joint1_bbox1_endpoint1 + joint1_bbox2_endpoint2)/2;
            else if(c.magnitude() <= a.magnitude() && c.magnitude() <= b.magnitude() && c.magnitude() <= d.magnitude())
              joint1 = (joint1_bbox1_endpoint2 + joint1_bbox2_endpoint1)/2;
            else
              joint1 = (joint1_bbox1_endpoint2 + joint1_bbox2_endpoint2)/2;

            //cout << "\tIntersection point for joint1: " << joint1 << endl;
            }

            {
            //repeat for joint2
            //get world coords for joint2.second
	      GMSPolyhedron& joint2_bbox1 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint2.second)->GetWorldBoundingBox();
            //cout << "\tjoint2_bbox1:\n";
            //for_each(joint2_bbox1.vertexList, joint2_bbox1.vertexList + joint2_bbox1.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");

            Vector3d joint2_bbox1_endpoint1;
            for(int j=0; j<4; ++j)
              joint2_bbox1_endpoint1 = joint2_bbox1_endpoint1 + joint2_bbox1.vertexList[j];
            joint2_bbox1_endpoint1 = joint2_bbox1_endpoint1 / 4;
            //cout << "\tjoint2_bbox1_endpoint1: " << joint2_bbox1_endpoint1 << endl;

            Vector3d joint2_bbox1_endpoint2;
            for(int j=4; j<8; ++j)
              joint2_bbox1_endpoint2 = joint2_bbox1_endpoint2 + joint2_bbox1.vertexList[j];
            joint2_bbox1_endpoint2 = joint2_bbox1_endpoint2 / 4;
            //cout << "\tjoint2_bbox1_endpoint2: " << joint2_bbox1_endpoint2 << endl;

            //get world coords for joint2.third
            GMSPolyhedron& joint2_bbox2 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint2.third)->GetWorldBoundingBox();
            //cout << "\tjoint2_bbox2:\n";
            //for_each(joint2_bbox2.vertexList, joint2_bbox2.vertexList + joint2_bbox2.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");

            Vector3d joint2_bbox2_endpoint1;
            for(int j=0; j<4; ++j)
              joint2_bbox2_endpoint1 = joint2_bbox2_endpoint1 + joint2_bbox2.vertexList[j];
            joint2_bbox2_endpoint1 = joint2_bbox2_endpoint1 / 4;
            //cout << "\tjoint2_bbox2_endpoint1: " << joint2_bbox2_endpoint1 << endl;

            Vector3d joint2_bbox2_endpoint2;
            for(int j=4; j<8; ++j)
              joint2_bbox2_endpoint2 = joint2_bbox2_endpoint2 + joint2_bbox2.vertexList[j];
            joint2_bbox2_endpoint2 = joint2_bbox2_endpoint2 / 4;
            //cout << "\tjoint2_bbox2_endpoint2: " << joint2_bbox2_endpoint2 << endl;

            //find intersection coord
            Vector3d a = joint2_bbox1_endpoint1 - joint2_bbox2_endpoint1;
            Vector3d b = joint2_bbox1_endpoint1 - joint2_bbox2_endpoint2;
            Vector3d c = joint2_bbox1_endpoint2 - joint2_bbox2_endpoint1;
            Vector3d d = joint2_bbox1_endpoint2 - joint2_bbox2_endpoint2;
            //cout << "\t\ta: " << a << "\tb: " << b << "\tc: " << c << "\td: " << d << endl;

            if(a.magnitude() <= b.magnitude() && a.magnitude() <= c.magnitude() && a.magnitude() <= d.magnitude())
              joint2 = (joint2_bbox1_endpoint1 + joint2_bbox2_endpoint1)/2;
            else if(b.magnitude() <= a.magnitude() && b.magnitude() <= c.magnitude() && b.magnitude() <= d.magnitude())
              joint2 = (joint2_bbox1_endpoint1 + joint2_bbox2_endpoint2)/2;
            else if(c.magnitude() <= a.magnitude() && c.magnitude() <= b.magnitude() && c.magnitude() <= d.magnitude())
              joint2 = (joint2_bbox1_endpoint2 + joint2_bbox2_endpoint1)/2;
            else
              joint2 = (joint2_bbox1_endpoint2 + joint2_bbox2_endpoint2)/2;

            //cout << "\tIntersection point for joint2: " << joint2 << endl;
            }

            //subtract to get lengtth
            ear_length = (joint1 - joint2).magnitude();

          }
          //cout << "\tsetting length: " << ear_length << "\tfor link " << g_ear_roots[i]->GetID() << endl;
          //g_ear_roots[i]->SetAvailableRange(Range(ear_length, ear_length));

          //cout << "Attempting to sample loop rooted at " << g_loopRoots[i]->GetID() << "...\n";
          //cout << "Attempting to sample loop rooted at " << g_ear_roots[i]->GetID() << "...\n";
          //if(!g_loopRoots[i]->RecursiveSample(0, true, gamma))
          if(!CCProblem->g_ear_roots[i]->RecursiveSample(ear_length, true, gamma))
          {
            cout << "\tCan't close loop " << i << "!\n";
            bSampleSucceeded = false;
            break;
          } else {
            //ConfigEar(&env, g_loopRoots[i]);
            //cout << "configuring successfully sampled ear\n";
            CCProblem->ConfigEar(CCProblem->GetEnvironment(), CCProblem->g_ear_roots[i], CCProblem->g_loopRoots[i]);

            // *
            {
              for(int e=0; e<=i; ++e)
              {
                cout << "Joint Coords for ear " << e << ":\n";
                PrintEarJointCoords(&env, g_ear_roots[e]);
              }
            }
            * //
            Clock_Class CollisionClock;
            CollisionClock.StartClock("Collision check");
            string CallName = "RandomSample";
            // WARNING: DANGEROUS CollisionDetection Optimization....
            //adjust num free bodies
            int num_bodies = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBodyCount();
            vector<int> actual_ear_links;
	    get_actual_links_of(CCProblem->g_ear_roots[i], actual_ear_links);
	    boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody());
	    //MultiBody mb = *(CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex()).get());
	    //boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody);
	    //for(vector<int>::iterator iter = actual_ear_links.begin(); iter<actual_ear_links.end(); iter++){
	    //cout<<"here "<<actual_ear_links.back()<<"_"<<<num_bodies<<endl;
	    for(size_t l=0; l<actual_ear_links.back();  ++l){

	      shared_ptr<FreeBody> freeBody = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex()).get()->GetFreeBody(l);
	      subsetOfRobot.get()->AddBody(freeBody);
	    }
            get_actual_links_of(CCProblem->g_ear_roots[i], actual_ear_links);
	    //CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->SetFreeBodyCount(actual_ear_links.back());
	    //colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo, boost::shared_ptr<MultiBody>((MultiBody*)NULL), true, &CallName);
	    colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo, subsetOfRobot, true, &CallName);
	    //colliding=false;//debugging code
            //reset num free bodies

	    // *
	    if(i<CCProblem->g_loopRoots.size()-1){
	      colliding=false;
	    }
	    * //

            //CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->SetFreeBodyCount(num_bodies);

            CollisionClock.StopClock();
            collision_time += CollisionClock.GetClock_SEC();
	    //if(i<CCProblem->g_loopRoots.size()){
	    //  colliding=false;//debugging code
	    //}


            if(colliding){
              bSampleSucceeded = false;
              break;
            }

          }
	  //cout<<"end for loop"<<endl;
        }
	//cout<<"out for loop"<<endl;
      }
      //cout<<"sample succeded"<<endl;
      // *
      //inport cfg
      for(size_t i=0; i<g_loopRoots.size(); ++i)
      {
        g_loopRoots[i]->ImportTreeLinkLength(input_lengths[count][i], input_convexities[count][i]);
      }
      * //

      //export lengths/convexities
      vector<vector<double> > lengths;
      vector<vector<int> >  convexities;
      for(size_t i=0; i<CCProblem->g_loopRoots.size(); ++i)
      {
        vector<double> l;
        vector<int> c;
        CCProblem->g_loopRoots[i]->ExportTreeLinkLength(l, c);
        lengths.push_back(l);
        convexities.push_back(c);
      }
      GenClock.StopClock();
      sampling_time += GenClock.GetClock_SEC();

      //output sampled cfg
      // *
      for(size_t i=0; i<g_loopRoots.size(); ++i)
      {
        cout << "Tree " << i << endl;
        cout << "length of tree "<< i <<  ":" << endl;
        copy(lengths[i].begin(), lengths[i].end(), ostream_iterator<double>(cout, " "));
        cout << endl;
        cout << "convexities of tree "<< i <<  ":" << endl;
        copy(convexities[i].begin(), convexities[i].end(), ostream_iterator<double>(cout, " "));
        cout << endl;
      }
      PrintConfiguration(&env, cout); cout << endl;
      * //

      //compute collision
      Clock_Class CollisionClock;
      CollisionClock.StartClock("Collision check");
      //ConfigureRobot(&env);
      string CallName = "RandomSample";

      //colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo,  boost::shared_ptr<MultiBody>((MultiBody*)NULL), true, &CallName);
      //for(vector<int>::iterator iter = actual_ear_links.begin(); iter<actual_ear_links.end(); iter++){
      boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody());
      for(int i=0; i<CCProblem->g_loopRoots.size(); ++i){
	//cout<<"ear link ="<<*iter<<endl;
	shared_ptr<FreeBody> freeBody = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex()).get()->GetFreeBody(i);
	subsetOfRobot.get()->AddBody(freeBody);
      }
      colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo, subsetOfRobot, true, &CallName);

      colliding=false;
      CollisionClock.StopClock();
      collision_time += CollisionClock.GetClock_SEC();


    }
    // *
    //output sampled cfg
    for(size_t i=0; i<g_loopRoots.size(); ++i)
    {
      vector<double> l;
      vector<int> c;
      g_loopRoots[i]->ExportTreeLinkLength(l, c);

      cout << "Tree " << i << endl;
      cout << "length of tree "<< i <<  ":" << endl;
      copy(l.begin(), l.end(), ostream_iterator<double>(cout, " "));
      cout << endl;
      cout << "convexities of tree "<< i <<  ":" << endl;
      copy(c.begin(), c.end(), ostream_iterator<double>(cout, " "));
      cout << endl;
    }
    * //
    CCProblem->PrintConfiguration(CCProblem->GetEnvironment(), cout);

  }
  cout << "Total sampling time: " << sampling_time << endl;
  cout << "Total collision time: " << collision_time << endl;
  cout << "Total time: " << (sampling_time + collision_time) << endl;
  cout << "Attempts: " << attempts << endl;
  //end move to solve function
}
*/
