#include "Cfg_reach_cc.h"
#include <numeric>
#include "MultiBody.h"
#include "Environment.h"
#include "FreeBody.h"
#include "CollisionDetection.h"
#include "Stat_Class.h"
#include "boost/lambda/lambda.hpp"
#include "boost/random.hpp"

#define TWO_PI 6.2831853072

int Cfg_reach_cc::NumofJoints;
Link* Cfg_reach_cc::link_tree = NULL;
vector<Link*> Cfg_reach_cc::actual_links;
double Cfg_reach_cc::rdres = 0.05;
double Cfg_reach_cc::gamma = 0.5;

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

//file for initializing tree
void
Cfg_reach_cc::
initialize_link_tree(const char* filename) {
  ifstream ifs(filename);
  int num_links;
  if(!(ifs >> num_links)) {
    cerr << "Error while reading link values: can't read num_links\n";
    exit(-1);
  }
  for(int i=0; i<num_links; ++i) {
    double length;
    if(!(ifs >> length)) {
      cerr << "Error while reading link " << i << " length\n";
      exit(-1);
    }
    actual_links.push_back(new Link(length, length));
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
  cerr << "Warning, subtract not implemented yet\n";
  /*
  for(int i=0; i<6; ++i)
    v[i] = c1.v[i] - c2.v[i];

  link_lengths.clear();
  transform(c1.link_lengths.begin(), c1.link_lengths.end(),
	    c2.link_lengths.begin(),
	    back_insert_iterator<vector<double> >(link_lengths),
	    minus<double>());

  //not sure what to do here...
  link_orientations.clear();
  transform(c1.link_orientations.begin(), c1.link_orientations.end(),
	    c2.link_orientations.begin(),
	    back_insert_iterator<vector<double> >(link_orientations),
	    minus<int>());

  StoreData();  
  */
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
GetRandomCfg(Environment*env,Stat_Class& Stats,CollisionDetection*cd , CDInfo& _cdInfo){

std::string Callee(GetName());
{std::string Method("Cfg_reach_cc::GetFreeRandomCfg");Callee=Callee+Method;}
 do {
   getReachableCfg(env,cd,Stats, _cdInfo);
    }while (this->isCollision(env,Stats,cd,_cdInfo,true,&Callee) );
     


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
  link_tree->RecursiveSample(0, true, gamma);
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
      double extAng = PI - Link::CalculateJointAngle(actual_links[i-1], 
					       actual_links[i]);
      sumExtAng += extAng;
      v.push_back(extAng/TWO_PI);
    }
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
  
  if(!link_tree->RecursiveSample(rand, 0))
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


double Cfg_reach_cc::MyCalculateJointAngle(Environment* env, Link* link1, Link* link2){
  for(vector<Link*>::iterator E = g_ear_roots.begin(); E != g_ear_roots.end(); ++E)
    {
      vector<int> actual_links;
      get_actual_links_of(*E, actual_links);

      if(find(actual_links.begin(), actual_links.end(), link1->GetID()) != actual_links.end() &&
	 find(actual_links.begin(), actual_links.end(), link2->GetID()) != actual_links.end())
	{
	  //cout << "\tjoint angle w/in an ear, calling old method\n";
	  return Link::CalculateJointAngle(link1, link2);
	}
    }
  //cout << "\tjoint angle not w/in an ear\n";
  Link* ear_root_link = NULL;
  Link* tree_root_link = NULL;
  for(size_t i=0; i<min(g_loopRoots.size(), g_ear_roots.size()); ++i)
    {
      vector<int> actual_links;
      get_actual_links_of(g_ear_roots[i], actual_links);

      //assume ear link is link2
      if(find(actual_links.begin(), actual_links.end(), link2->GetID()) != actual_links.end())
	{
	  ear_root_link = g_ear_roots[i];
	  tree_root_link = g_loopRoots[i];
	  break;
	}
    }
  if(ear_root_link == NULL)
    {
      cerr << "Error, did not find link " << link2->GetID() << " in an ear\n";
      exit(-1);
    }

  double angle = PI;
  GMSPolyhedron& link1_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(link1->GetID())->GetWorldPolyhedron();
  Vector3D joint1; //end of link1_poly
  for(int i=0; i<4; ++i)
    joint1 = joint1 + link1_poly.vertexList[i];
  joint1 = joint1 / 4;
  vector<int> actual_ear_links;
  get_actual_links_of(ear_root_link, actual_ear_links);
  vector<int> actual_loop_links;
  get_actual_links_of(tree_root_link, actual_loop_links);

  int link3_id = actual_ear_links.back();
  GMSPolyhedron& link3_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(link3_id)->GetWorldPolyhedron();
  Vector3D end_link3; //end of link3_poly
  for(int i=0; i<4; ++i)
    end_link3 = end_link3 + link3_poly.vertexList[i];
  end_link3 = end_link3 / 4;
  //cout << "\t\tend_link3 = " << end_link3 << endl;

  int link4_id = -1;
  for(vector<int>::iterator L = actual_loop_links.begin(); L != actual_loop_links.end(); ++L)
    if(find(actual_ear_links.begin(), actual_ear_links.end(), *L) == actual_ear_links.end())
      {
	link4_id = *L;
	break;
      }
  if(link4_id == -1)
    {
      cerr << "Error in MyCalculateJointAngles: did not find link for ear connection\n";
      exit(-1);
    }
  //cout << "\t\tlink4_id = " << link4_id << endl;
  GMSPolyhedron& link4_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(link4_id)->GetWorldPolyhedron();
  Vector3D joint2; //start of link4_poly
  for(int i=4; i<8; ++i)
    joint2 = joint2 + link4_poly.vertexList[i];
  joint2 = joint2 / 4;
  //cout << "\t\tjoint2 = " << joint2 << endl;

  //cout << "\t\t\tear_length = " << (end_link3 - joint1).magnitude() << endl;

  //beta = angle between (joint2-joint1) and (end_link3 - joint1)
  Vector3D V1 = joint2 - joint1;
  V1.normalize();
  Vector3D V2 = end_link3 - joint1;
  V2.normalize();

  /*
  cout << "\nMyCalculateJointAngle(" << link1->GetID() << "," << link2->GetID() << ")" << endl;
  cout << "\tjoint1 = " << joint1 << endl;
  cout << "\tjoint2 = " << joint2 << endl;
  cout << "\tend_link3 = " << end_link3 << endl;
  */




  double beta = atan2(V2.getY(), V2.getX()) - atan2(V1.getY(), V1.getX());
  //cout << "\tbeta = " << beta << endl;

  //return angle + beta;
  angle += beta;
  while(angle < 0)
    angle += TWO_PI;
  while(angle > TWO_PI)
    angle -= TWO_PI;

  ConfigEar(env, ear_root_link, tree_root_link, angle);

  return angle;
}



void Cfg_reach_cc::ParseXML(XMLNodeReader& in_Node) { 
  
  LOG_DEBUG_MSG("ClosedChainProblem::ParseXML()");

  in_Node.verifyName("MPProblem");

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "links_file") {
      string filename = citr->stringXMLParameter(string("filename"), true, string(""),string("Links File Name"));
      cout<<"filename="<<filename<<endl;
      cout<<"parsing file"<<endl;
      ParseLinksFile(filename.c_str());
      cout<<"file "<<filename<<" has been parsed"<<endl;
    } else {
      citr->warnUnknownNode();
    }
    /*
      else  if(citr->getName() == "distance_metrics") {
      m_pDistanceMetric = new DistanceMetric(*citr, this);
    } else  if(citr->getName() == "collision_detection") {
      m_pCollisionDetection = new CollisionDetection(*citr, this);
    } 
      else  if(citr->getName() == "validity_test") {
      m_pCollisionDetection = new CollisionDetection(*citr, this);
      m_pValidityChecker = new ValidityChecker<CfgType>(*citr, this);
    } else  if(citr->getName() == "MPRegions") {
      ///\Todo Parse MPRegions
    } else  if(citr->getName() == "NeighborhoodFinder") {
      m_pNeighborhoodFinder = new NeighborhoodFinder(*citr,this);
    
    }else {
      citr->warnUnknownNode();
    }
    */
    
  }
}

/*
void MPProblem::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("MPProblem::ParseXML()");

  in_Node.verifyName("MPProblem");

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "environment") {
      m_pEnvironment = new Environment(*citr, this);
    } else  if(citr->getName() == "distance_metrics") {
      m_pDistanceMetric = new DistanceMetric(*citr, this);
    } else  if(citr->getName() == "collision_detection") {
      m_pCollisionDetection = new CollisionDetection(*citr, this);
    }
    else  if(citr->getName() == "validity_test") {
      m_pCollisionDetection = new CollisionDetection(*citr, this);
      m_pValidityChecker = new ValidityChecker<CfgType>(*citr, this);
    } else  if(citr->getName() == "MPRegions") {
      ///\Todo Parse MPRegions
    } else  if(citr->getName() == "NeighborhoodFinder") {
      m_pNeighborhoodFinder = new NeighborhoodFinder(*citr,this);
    }else {
      citr->warnUnknownNode();
    }
  }

  vector<cd_predefined> cdtypes = m_pCollisionDetection->GetSelectedCDTypes();
  for(vector<cd_predefined>::iterator C = cdtypes.begin(); C != cdtypes.end(); ++C)
    m_pEnvironment->buildCDstructure(*C, 1);
  LOG_DEBUG_MSG("~MPProblem::ParseXML()");
}
*/

bool Cfg_reach_cc::ParseRealLink(ifstream &fin)
  {
    int myID = -1;
    fin >> myID;

    double minRange = -1, maxRange = -1;
    fin >> minRange;
    fin >> maxRange;

    cout << "myID:" << myID << " minRange:" << minRange << " maxRange:" << maxRange << endl;

    if(g_baseLinks[myID] == NULL)
      {
	g_baseLinks[myID] = new Link(myID, minRange, maxRange);
      }
    else
      {
	cerr << "ERROR: duplicate definition of link " << myID << endl;
	exit(1);
      }

    return true;
  }


bool Cfg_reach_cc::ParseVirtualLink(ifstream &fin)
{
  int myID = -1, lcID = -1, rcID = -1;
  fin >> myID;
  fin >> lcID;
  fin >> rcID;
  cout << "myID: " << myID << " lcID:" << lcID << " rcID:" << rcID << endl;

  if(g_baseLinks[myID] == NULL)
    {
      if(g_baseLinks[lcID] && g_baseLinks[rcID])
	{
	  g_baseLinks[myID] = new Link(myID, g_baseLinks[lcID], g_baseLinks[rcID]);
	}
      else
	{
	  cerr << "ERROR: " << myID << endl;
	  exit(1);
	}
    }
  else
    {
      cerr << "ERROR: duplicate definition of link " << myID << endl;
      exit(1);
    }

  return true;
}




bool Cfg_reach_cc::ParseLoop(ifstream &fin)
{
  int loopSize = 0;
  fin >> loopSize;
  
  vector<int> loopLinksID;
  vector<Link *> loopLinks;
  for(int i=0; i<loopSize; ++i)
  {
    int linkID = -1;
    fin >> linkID;      
    loopLinksID.push_back(linkID);
    loopLinks.push_back(g_baseLinks[linkID]);
  }
  cout << "Loop: "; 
  copy(loopLinksID.begin(), loopLinksID.end(), ostream_iterator<int>(cout, " "));
  cout << endl;    
    
  g_loopIDs.push_back(loopLinksID);
  g_loopLinks.push_back(loopLinks);

  Link *newTree = BuildTree(0, loopLinks.size()-1, loopLinks);
  g_loopRoots.push_back(newTree);

  return true;
}



bool Cfg_reach_cc::ParseCfgJoints(ifstream &fin)
{
 int numAngles = 0;
 fin >> numAngles;

 vector<pair<int, int> > jointLinkIDs;
 for(int i=0; i<numAngles; ++i)
 {
   int linkID1 = -1, linkID2 = -1;
   fin >> linkID1;
   fin >> linkID2;
   jointLinkIDs.push_back(pair<int, int>(linkID1, linkID2));
   g_cfgJoints.push_back(pair<Link*, Link *>(g_baseLinks[linkID1], g_baseLinks[linkID2]));
 }
      
 cout << "reading joints: " << endl;
 for(size_t i=0; i<jointLinkIDs.size(); ++i)
   cout << jointLinkIDs[i].first << "," << jointLinkIDs[i].second << endl;
      
 return true;
}


bool Cfg_reach_cc::ParseEarJoints(ifstream &fin)
{
 int numJoints = 0;
 fin >> numJoints;

 for(int i=0; i<numJoints; ++i)
 {
   int linkID1 = -1, linkID2 = -1, linkID3 = -1;
   fin >> linkID1;
   fin >> linkID2;
   fin >> linkID3;
   g_earJoints.push_back(triple<int, int, int>(linkID1, linkID2, linkID3));
 }
      
 cout << "reading ear joints: " << endl;
 for(size_t i=0; i<g_earJoints.size(); ++i)
   cout << g_earJoints[i].first << "," << g_earJoints[i].second << "," << g_earJoints[i].third << endl;
      
 return true;
}



//void ClosedChainProblem::ConfigBase(Environment* env, const vector<double>& v = vector<double>(6, 0))
void Cfg_reach_cc::ConfigBase(Environment* env, const vector<double>& v)
{
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, v[5]*TWOPI, v[4]*TWOPI, v[3]*TWOPI), Vector3D(v[0], v[1], v[2]));
  env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(0)->Configure(T1);
}



void Cfg_reach_cc::ConfigEar(Environment* env, Link* ear_root, vector<int>& actual_ear_links, int base_link_id, double base_link_angle){
  int robot = env->GetRobotIndex();

  if(find(actual_ear_links.begin(), actual_ear_links.end(), base_link_id) != actual_ear_links.end())
    base_link_id = -1;
  vector<double> angles;
  for(vector<pair<Link*, Link*> >::iterator J = g_cfgJoints.begin(); J != g_cfgJoints.end(); ++J)
    {
      if((J->first->GetID() == base_link_id || find(actual_ear_links.begin(), actual_ear_links.end(), J->first->GetID()) !=
          actual_ear_links.end()) 
         && (J->second->GetID() == base_link_id || find(actual_ear_links.begin(), actual_ear_links.end(), J->second->GetID()) != actual_ear_links.end()))
        {
	  double angle = 0;
          if(J->first->GetID() == base_link_id || J->second->GetID() == base_link_id)
            angle = (PI - base_link_angle) / TWO_PI;
          else
            angle = (PI - MyCalculateJointAngle(env, J->first, J->second)) / TWO_PI;
          while(angle < 0)
            angle += 1;

	  //find appropriate backward connection...
          FreeBody* link2 = env->GetMultiBody(robot)->GetFreeBody(J->second->GetID()).get();
          bool found_connection = false;
          for(int i=0; i<link2->BackwardConnectionCount(); ++i) {
            FreeBody* link1 = env->GetMultiBody(robot)->GetFreeBody(J->first->GetID()).get();
            Connection conn = link2->GetBackwardConnection(i);
	    boost::shared_ptr<Body> *boostPtr = new boost::shared_ptr<Body>(link1);
            if(conn.IsFirstBody(*boostPtr))
              {
                found_connection = true;
                conn.GetDHparameters().theta = angle * 360.0;
                link2->GetBackwardConnection(i).GetDHparameters().theta = angle * 360.0;
                //cout << "setting connection theta of link " << J->first->GetID() << endl;
                break;
              }
          }
          if(!found_connection)
            {
              cout<< "Error in ConfigEar: couldn't find connection "<<endl;
              cerr << "Error in ConfigEar: couldn't find connection " << J->first->GetID() << " -> " << J->second->GetID() << endl;
              exit(-1);
            }
        }
    }
    set<int> transformed;
    //mark base
    transformed.insert(0);
    for(vector<Link*>::iterator E = g_ear_roots.begin(); E != g_ear_roots.end() && (*E) != ear_root; ++E){
	vector<int> _actual_links;
	get_actual_links_of(*E, _actual_links);
	transformed.insert(_actual_links.begin(), _actual_links.end());
    }
    for(int i=0; i<env->GetMultiBody(robot)->GetFreeBodyCount(); ++i)
      if(find(actual_ear_links.begin(), actual_ear_links.end(), i) != actual_ear_links.end()){
	  //cout << "calling ComputeWorldTransformation for link " << i << endl;
	  env->GetMultiBody(robot)->GetFreeBody(i)->ComputeWorldTransformation(transformed);
      }
}


void Cfg_reach_cc::ConfigEar(Environment* env, Link* ear_root, Link* loop_root){
  vector<int> actual_ear_links;
  get_actual_links_of(ear_root, actual_ear_links);

  vector<int> actual_loop_links;
  get_actual_links_of(loop_root, actual_loop_links);
  int base_link_id = actual_loop_links.back(); //or actual_non_ear_links.back()
  double base_link_angle = MyCalculateJointAngle(env, g_baseLinks[base_link_id], g_baseLinks[actual_ear_links.front()]);
  ConfigEar(env, ear_root, actual_ear_links, base_link_id, base_link_angle);
}


void Cfg_reach_cc::ConfigEar(Environment*env, Link* ear_root, Link* loop_root, double base_link_angle){
  vector<int> actual_loop_links;
  get_actual_links_of(loop_root, actual_loop_links);

  vector<int> actual_ear_links;
  get_actual_links_of(ear_root, actual_ear_links);

  int base_link_id = actual_ear_links.front();
  if(!actual_loop_links.empty())
    base_link_id = actual_loop_links.back();

  ConfigEar(env, ear_root, actual_ear_links, base_link_id, base_link_angle);
}

bool Cfg_reach_cc::ParseLinksFile(const char* linksFileName)
{
  ifstream fin(linksFileName, ios::in);
  if(!fin.good())
  {
    cerr << " Can not open " << linksFileName << endl;
    exit(1);
  }

  int numLinks = 0;
  
  char strData[256];
  fin >> strData;
  if(strcmp(strData, "numLinks") != 0)
    return false;
  else 
    fin >> numLinks;

  for(int i=0; i<numLinks; ++i)
    g_baseLinks.push_back(NULL);
    
  while(!fin.eof())
  {
    strData[0] = '\0';
    fin >> strData;
    cout << "strData:" << strData << endl;

    if(strcmp(strData, "RealLink") == 0)
    {
      if(!ParseRealLink(fin)) 
        return false;
    }
    else if(strcmp(strData, "VirtualLink") == 0)
    {
      if(!ParseVirtualLink(fin))
        return false;
    }
    else if(strcmp(strData, "Loop") == 0)
    {
      if(!ParseLoop(fin))
        return false;
    }
    else if(strcmp(strData, "CfgJoints") == 0)
    {
      if(!ParseCfgJoints(fin))
        return false;
    }
    else if(strcmp(strData, "EarJoints") == 0)
    {
      if(!ParseEarJoints(fin))
        return false;
    }
  }

  fin.close();

  /*
  //look for implied constraints:
  cout << "Implied constraints found:\n";
  set<Link*> seen;
  for(size_t i=0; i<loopRoots.size(); ++i)
  {
    cout << "\tLoop " << i << ":";
    set<Link*> implied;
    seen_both_children(loopRoots[i], seen, implied);
    for(set<Link*>::iterator I = implied.begin(); I != implied.end(); ++I)
      cout << " " << (*I)->GetID();
    cout << endl;
  }
  */

  //find ears
  cout << "Partitioning loops into (unseen) (seen):\n";
  g_ears.clear();
  g_non_ears.clear();
  set<Link*> links_seen;
  for(size_t i=0; i<g_loopRoots.size(); ++i)
  {
    vector<int> seen, unseen;
    partition_loop(g_loopRoots[i], links_seen, seen, unseen);
    g_ears.push_back(unseen);
    g_non_ears.push_back(seen);
    cout << "\tLoop " << i << ": (";
    for_each(unseen.begin(), unseen.end(), cout << boost::lambda::_1 << " ");
    cout << ") (";
    for_each(seen.begin(), seen.end(), cout << boost::lambda::_1 << " ");
    cout << ")\n";
  }

  //find ear virtual link roots
  g_ear_roots.clear();
  g_ear_rootIDs.clear();
  for(size_t i=0; i<g_loopRoots.size(); ++i)
  {
    Link* parent = get_parent_of(g_loopRoots[i], g_ears[i]);
    if(parent == NULL)
    {
      cerr << "Error, parent link not found for ear " << i << "in loop " << i << endl;
      exit(-1);
    } 
    else
    {
      g_ear_roots.push_back(parent);
      g_ear_rootIDs.push_back(parent->GetID());
    }
  }
  cout << "Ear root ids:\n";
  for(size_t i=0; i<g_ear_rootIDs.size(); ++i)
    cout << "\tEar " << i << ": " << g_ear_rootIDs[i] << endl;

  return true;
}


vector<double> Cfg_reach_cc::GetConfigurationVector(Environment* env){
  //ofPath << "0 0 0 0 0 0 ";
  vector<double> joint_angles;
  for(size_t i=0; i<g_cfgJoints.size(); ++i)
    {
      double extAng = (PI - MyCalculateJointAngle(env, g_cfgJoints[i].first, g_cfgJoints[i].second)) / TWO_PI;
      while(extAng < 0)
        extAng += 1;
      while(extAng > 1)
        extAng -= 1;
      joint_angles.push_back(extAng);
      //ofPath << extAng << " ";
    }
  //ofPath << endl;
  return joint_angles;
}



vector<double> Cfg_reach_cc::getReachableCfg(Environment* env, CollisionDetection* cd, Stat_Class& Stats, CDInfo &_cdInfo, bool is_gamma_random){
     if(is_gamma_random)
        gamma = drand48();
      //cout << "\tgamma = " << gamma << "...\n";

      //sample closed cfg  
      bool bSampleSucceeded = false;  
      //Clock_Class GenClock;  
      //GenClock.StartClock("Node generation");  
      while(!bSampleSucceeded)  
      {  
        //attempts++;

        //re-initialize cfg  
        for(size_t i=0; i<g_loopRoots.size(); ++i)  
          g_loopRoots[i]->ResetTree();  
        ConfigBase(env);
        //ConfigEnvironment(&env);
         
        bSampleSucceeded = true;  
        for(size_t i=0; i<g_loopRoots.size(); ++i)  
        {      
          //sample closed loop 

          //set ear constraint
          double ear_length = 0;
          if(!g_non_ears[i].empty())
          {
            //compute ear required length
            triple<int,int,int> ear_joint1 = *(find_if(g_earJoints.begin(), g_earJoints.end(), first_is<int, triple<int,int,int> >(g_ears[i].front())));
            triple<int,int,int> ear_joint2 = *(find_if(g_earJoints.begin(), g_earJoints.end(), first_is<int, triple<int,int,int> >(g_ears[i].back())));
            /*
            cout << "setting ear constraint for loop " << i << endl;
            cout << "\tear_joint1: " << ear_joint1.first << "," << ear_joint1.second << "," << ear_joint1.third << endl;
            cout << "\tear_joint2: " << ear_joint2.first << "," << ear_joint2.second << "," << ear_joint2.third << endl;
            */

            Vector3D joint1, joint2;
            {
            //note, following assumes link orientated along x-axis for longest length
            //get world coords for joint1.second
	      GMSPolyhedron& joint1_bbox1 = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(ear_joint1.second)->GetWorldBoundingBox();
            //cout << "\tjoint1_bbox1:\n";
            //for_each(joint1_bbox1.vertexList, joint1_bbox1.vertexList + joint1_bbox1.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");

            Vector3D joint1_bbox1_endpoint1;
            for(int j=0; j<4; ++j)
              joint1_bbox1_endpoint1 = joint1_bbox1_endpoint1 + joint1_bbox1.vertexList[j];
            joint1_bbox1_endpoint1 = joint1_bbox1_endpoint1 / 4;
            //cout << "\tjoint1_bbox1_endpoint1: " << joint1_bbox1_endpoint1 << endl;

            Vector3D joint1_bbox1_endpoint2;
            for(int j=4; j<8; ++j)
              joint1_bbox1_endpoint2 = joint1_bbox1_endpoint2 + joint1_bbox1.vertexList[j];
            joint1_bbox1_endpoint2 = joint1_bbox1_endpoint2 / 4;
            //cout << "\tjoint1_bbox1_endpoint2: " << joint1_bbox1_endpoint2 << endl;

            //get world coords for joint1.third
            GMSPolyhedron& joint1_bbox2 = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(ear_joint1.third)->GetWorldBoundingBox();
            //cout << "\tjoint1_bbox2:\n";
            //for_each(joint1_bbox2.vertexList, joint1_bbox2.vertexList + joint1_bbox2.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");
            
            Vector3D joint1_bbox2_endpoint1;
            for(int j=0; j<4; ++j)
              joint1_bbox2_endpoint1 = joint1_bbox2_endpoint1 + joint1_bbox2.vertexList[j];
            joint1_bbox2_endpoint1 = joint1_bbox2_endpoint1 / 4;
            //cout << "\tjoint1_bbox2_endpoint1: " << joint1_bbox2_endpoint1 << endl;

            Vector3D joint1_bbox2_endpoint2;
            for(int j=4; j<8; ++j)
              joint1_bbox2_endpoint2 = joint1_bbox2_endpoint2 + joint1_bbox2.vertexList[j];
            joint1_bbox2_endpoint2 = joint1_bbox2_endpoint2 / 4;
            //cout << "\tjoint1_bbox2_endpoint2: " << joint1_bbox2_endpoint2 << endl;

            //find intersection coord
            Vector3D a = joint1_bbox1_endpoint1 - joint1_bbox2_endpoint1;
            Vector3D b = joint1_bbox1_endpoint1 - joint1_bbox2_endpoint2;
            Vector3D c = joint1_bbox1_endpoint2 - joint1_bbox2_endpoint1;
            Vector3D d = joint1_bbox1_endpoint2 - joint1_bbox2_endpoint2;
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
	      GMSPolyhedron& joint2_bbox1 = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(ear_joint2.second)->GetWorldBoundingBox();
            //cout << "\tjoint2_bbox1:\n";
            //for_each(joint2_bbox1.vertexList, joint2_bbox1.vertexList + joint2_bbox1.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");
            
            Vector3D joint2_bbox1_endpoint1;
            for(int j=0; j<4; ++j)
              joint2_bbox1_endpoint1 = joint2_bbox1_endpoint1 + joint2_bbox1.vertexList[j];
            joint2_bbox1_endpoint1 = joint2_bbox1_endpoint1 / 4;
            //cout << "\tjoint2_bbox1_endpoint1: " << joint2_bbox1_endpoint1 << endl;

            Vector3D joint2_bbox1_endpoint2;
            for(int j=4; j<8; ++j)
              joint2_bbox1_endpoint2 = joint2_bbox1_endpoint2 + joint2_bbox1.vertexList[j];
            joint2_bbox1_endpoint2 = joint2_bbox1_endpoint2 / 4;
            //cout << "\tjoint2_bbox1_endpoint2: " << joint2_bbox1_endpoint2 << endl;

            //get world coords for joint2.third
            GMSPolyhedron& joint2_bbox2 = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(ear_joint2.third)->GetWorldBoundingBox();
            //cout << "\tjoint2_bbox2:\n";
            //for_each(joint2_bbox2.vertexList, joint2_bbox2.vertexList + joint2_bbox2.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");
            
            Vector3D joint2_bbox2_endpoint1;
            for(int j=0; j<4; ++j)
              joint2_bbox2_endpoint1 = joint2_bbox2_endpoint1 + joint2_bbox2.vertexList[j];
            joint2_bbox2_endpoint1 = joint2_bbox2_endpoint1 / 4;
            //cout << "\tjoint2_bbox2_endpoint1: " << joint2_bbox2_endpoint1 << endl;

            Vector3D joint2_bbox2_endpoint2;
            for(int j=4; j<8; ++j)
              joint2_bbox2_endpoint2 = joint2_bbox2_endpoint2 + joint2_bbox2.vertexList[j];
            joint2_bbox2_endpoint2 = joint2_bbox2_endpoint2 / 4;
            //cout << "\tjoint2_bbox2_endpoint2: " << joint2_bbox2_endpoint2 << endl;

            //find intersection coord
            Vector3D a = joint2_bbox1_endpoint1 - joint2_bbox2_endpoint1;
            Vector3D b = joint2_bbox1_endpoint1 - joint2_bbox2_endpoint2;
            Vector3D c = joint2_bbox1_endpoint2 - joint2_bbox2_endpoint1;
            Vector3D d = joint2_bbox1_endpoint2 - joint2_bbox2_endpoint2;
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
          if(!g_ear_roots[i]->RecursiveSample(ear_length, true, gamma))
          {  
            cout << "\tCan't close loop " << i << "!\n";  
            bSampleSucceeded = false;  
            break;  
          } else {
            //ConfigEar(&env, g_loopRoots[i]);
            //cout << "configuring successfully sampled ear\n";
            ConfigEar(env, g_ear_roots[i], g_loopRoots[i]);
	  
            /*
            {
              for(int e=0; e<=i; ++e)
              {
                cout << "Joint Coords for ear " << e << ":\n";
                PrintEarJointCoords(&env, g_ear_roots[e]);
              }
            }
            */
            //Clock_Class CollisionClock;
            //CollisionClock.StartClock("Collision check");
            string CallName = "RandomSample";
            // WARNING: DANGEROUS CollisionDetection Optimization....
            //adjust num free bodies
            int num_bodies = env->GetMultiBody(env->GetRobotIndex())->GetFreeBodyCount();
            vector<int> actual_ear_links;
	    get_actual_links_of(g_ear_roots[i], actual_ear_links);
	    boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody());
	    //MultiBody mb = *(CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex()).get());
	    //boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody);
	    //for(vector<int>::iterator iter = actual_ear_links.begin(); iter<actual_ear_links.end(); iter++){
	    //cout<<"here "<<actual_ear_links.back()<<"_"<<<num_bodies<<endl;
	    for(size_t l=0; l<actual_ear_links.back();  ++l){
	   
	      shared_ptr<FreeBody> freeBody = env->GetMultiBody(env->GetRobotIndex()).get()->GetFreeBody(l);
	      subsetOfRobot.get()->AddBody(freeBody);
	    }
            get_actual_links_of(g_ear_roots[i], actual_ear_links);
	    //CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->SetFreeBodyCount(actual_ear_links.back());
	    //colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo, boost::shared_ptr<MultiBody>((MultiBody*)NULL), true, &CallName);
	    bool colliding = cd->IsInCollision(env, Stats, _cdInfo, subsetOfRobot, true, &CallName);
	    //colliding=false;//debugging code
            //reset num free bodies
      
	    /*
	    if(i<CCProblem->g_loopRoots.size()-1){
	      colliding=false;
	    }
	    */
	       
            //CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->SetFreeBodyCount(num_bodies);

            //CollisionClock.StopClock();
            //collision_time += CollisionClock.GetClock_SEC();
	    //if(i<CCProblem->g_loopRoots.size()){
	    //  colliding=false;//debugging code
	    //}
       

            if(colliding){
              bSampleSucceeded = false;
              break;
            }
           
          }
	}
      }
      return GetConfigurationVector(env);
}


vector<double> Cfg_reach_cc::getReachableCfg(Environment* env, CollisionDetection* cd, Stat_Class& Stats, CDInfo& _cdInfo){
  getReachableCfg(env, cd, Stats, _cdInfo, true);
}
