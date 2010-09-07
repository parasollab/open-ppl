#ifndef Cfg_reach_cc_h
#define Cfg_reach_cc_h

#include "Cfg_free_tree.h"
#include "VirtualLink.h"
#include "CollisionDetection.h"
#include "Stat_Class.h"

template <typename T, typename U, typename V>
struct triple
{
  triple() {}
  triple(const T& t, const U& u, const V& v) : first(t), second(u), third(v) {}
  ~triple() {}

  T first;
  U second;
  V third;
};

template<typename F, typename T>
struct first_is : public unary_function<T, bool>
{
  first_is(const F& f) : first(f) {}
  ~first_is() {}
  bool operator()(const T& t) const 
  {
    return first == t.first;
  }

  F first;
};


class Cfg_reach_cc : public Cfg_free_tree {
 public:
  Cfg_reach_cc();
  Cfg_reach_cc(int _numofjoints);
  Cfg_reach_cc(const Vector6D& _v);
  Cfg_reach_cc(const vector<double>& _v);
  Cfg_reach_cc(const Cfg&c);
  Cfg_reach_cc(double x, double y, double z, 
	       double roll, double pitch, double yaw);
  Cfg_reach_cc(const Vector6D& base,
	       const vector<double>& len, const vector<int>& ori);
  virtual ~Cfg_reach_cc();

  static int getNumofJoints() { return NumofJoints; }
  static void setNumofJoints(int _numofjoints);
  static void initialize_link_tree(const char* filename);

  virtual const char* GetName() const { return "Cfg_reach_cc"; }

  virtual void equals(const Cfg&);

  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double);
  virtual void divide(const Cfg&, double);
  virtual void WeightedSum(const Cfg&, const Cfg&, double weight = 0.5); 
  virtual void c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d);

  virtual bool isWithinResolution(const Cfg &c, 
				  double positionRes, 
				  double orientationRes) const;

  virtual bool ConfigEnvironment(Environment*) const;

  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* env) {
    Cfg_free_tree::GetRandomCfg(env);
  }
  /*
  virtual void GetRandomCfg(Environment* env, DistanceMetric* _dm, 
                            double length);
  */
  virtual void GetRandomCfg_CenterOfMass(Environment* env);
  virtual void GetRandomRay(double incr, Environment* env, DistanceMetric* dm);

  /*
  virtual void MAPRMfree(Environment* _env, Stat_Class& Stats,
			 CollisionDetection* cd, CDInfo& cdInfo, 
			 DistanceMetric* dm, int n);

  virtual bool GenerateOverlapCfg(Environment* env, int robot,
				  Vector3D robot_start, Vector3D robot_goal, 
				  Cfg* resultCfg);
  virtual void GenSurfaceCfgs4ObstNORMAL(Environment* env, Stat_Class& Stats,
					 CollisionDetection*,
					 int obstacle, int nCfgs,
					 CDInfo& _cdInfo,
					 vector<Cfg*>& nodes) const;
  */

  virtual void FindNeighbors(Environment* env, Stat_Class& Stats,
			     const Cfg& increment,
			     CollisionDetection*,
			     int noNeighbors, 
			     CDInfo& _cdInfo,
			     vector<Cfg*>& cfgs);
  virtual void FindNeighbors(Environment* env, Stat_Class& Stats,
			     const Cfg& goal, const Cfg& increment, 
			     CollisionDetection*,
			     int noNeighbors, 
			     CDInfo& _cdInfo,
			     vector<Cfg*>& cfgs);

  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, 
			     int* n_ticks, 
			     double positionRes, double orientationRes);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks);

  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;

  ostream& print(ostream& os) const;
  ostream& print_base(ostream& os) const;
  ostream& print_len(ostream& os) const;
  ostream& print_ori(ostream& os) const;

  static void print_link_tree(ostream& os) {
    if(link_tree != NULL)
      link_tree->PrintTree(os);
  }

  static double rdres;
  static double gamma;

  bool GetIntermediate(const Cfg_reach_cc& c1,
		       const Cfg_reach_cc& c2);

  static bool OrientationsDifferent(const Cfg_reach_cc& c1,
				    const Cfg_reach_cc& c2) {
    return c1.link_orientations != c2.link_orientations;
  }

  double LengthDistance(const Cfg_reach_cc& c2) const;
  double OrientationDistance(const Cfg_reach_cc& c2) const;


  void ParseXML(XMLNodeReader& in_Node);
  bool ParseRealLink(ifstream &fin);
  bool ParseVirtualLink(ifstream &fin);
  bool ParseLoop(ifstream &fin);
  bool ParseCfgJoints(ifstream &fin);
  bool ParseEarJoints(ifstream &fin);


  double MyCalculateJointAngle(Environment* env, Link* link1, Link* link2);
  bool ParseLinksFile(const char* linksFileName);

  void ConfigBase(Environment* env, const vector<double>& v);
  void ConfigBase(Environment* env){ConfigBase(env, vector<double>(6, 0));}

  void ConfigEar(Environment* env, Link* ear_root, vector<int>& actual_ear_links, int base_link_id, double base_link_angle);
  void ConfigEar(Environment* env, Link* ear_root, Link* loop_root);
  void ConfigEar(Environment* env, Link* ear_root, Link* loop_root, double base_link_angle);
  void getReachableCfg(Environment* env, CollisionDetection* cd, bool is_gamma_random);
  void getReachableCfg(Environment* env, CollisionDetection* cd);



 protected:
  void StoreData();

  static   int NumofJoints;

  vector<double> link_lengths;
  vector<int> link_orientations;
  static Link* link_tree;
  static vector<Link*> actual_links;

  Stat_Class Stats;
  CDInfo _cdInfo;
  
  vector<Link *> g_baseLinks;
  vector<Link *> g_loopRoots;
  vector<vector<int> > g_loopIDs;
  vector<vector<Link *> > g_loopLinks;
  vector<pair<Link *, Link *> > g_cfgJoints;
  vector<triple<int,int,int> > g_earJoints;
  vector<Link*> g_ear_roots;
  vector<int> g_ear_rootIDs;
  vector<vector<int> > g_ears;
  vector<vector<int> > g_non_ears;
  
};

#endif
