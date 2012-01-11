#ifndef Cfg_reach_cc_h
#define Cfg_reach_cc_h

#include "Cfg_free_tree.h"
#include "VirtualLink.h"

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
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg_free_tree::define_type(t);
      t.member(link_lengths);
      t.member(link_orientations);
      
    }
 #endif

  static int GetNumOfJoints() { return m_numOfJoints; }
  static void SetNumOfJoints(int _numofjoints);
  static void initialize_link_tree(const char* filename);

  virtual const char* GetName() const { return "Cfg_reach_cc"; }

  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double);
  virtual void divide(const Cfg&, double);
  virtual Cfg& operator=(const Cfg&);
  virtual void WeightedSum(const Cfg&, const Cfg&, double weight = 0.5); 
  virtual void c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d);

  virtual bool isWithinResolution(const Cfg &c, 
				  double positionRes, 
				  double orientationRes) const;

  virtual bool ConfigEnvironment(Environment*) const;

  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* _env,shared_ptr<BoundingBox> _bb) {
    Cfg_free_tree::GetRandomCfg(_env,_bb);
  }
  virtual void GetRandomCfg(Environment* _env){
    Cfg_free_tree::GetRandomCfg(_env);
  } 
  /*
  virtual void GetRandomCfg(Environment* env, DistanceMetric* _dm, 
                            double length);
  */
  virtual void GetRandomCfg_CenterOfMass(Environment* env);
  virtual void GetRandomCfg_CenterOfMass(Environment* env,shared_ptr<BoundingBox> bb);
  virtual void GetRandomRay(double incr, Environment* env,shared_ptr <DistanceMetricMethod> dm);

  /*
  virtual void MAPRMfree(Environment* _env, StatClass& Stats,
			 CollisionDetection* cd, CDInfo& cdInfo, 
			 DistanceMetric* dm, int n);

  virtual bool GenerateOverlapCfg(Environment* env, int robot,
				  Vector3D robot_start, Vector3D robot_goal, 
				  Cfg* resultCfg);
  virtual void GenSurfaceCfgs4ObstNORMAL(Environment* env, StatClass& Stats,
					 CollisionDetection*,
					 int obstacle, int nCfgs,
					 CDInfo& _cdInfo,
					 vector<Cfg*>& nodes) const;
  */

  virtual void FindNeighbors(MPProblem* mp, Environment* env, StatClass& Stats,
			     const Cfg& increment,
			     string vc_method,
			     int noNeighbors, 
			     CDInfo& _cdInfo,
			     vector<Cfg*>& cfgs);
  virtual void FindNeighbors(MPProblem* mp, Environment* env, StatClass& Stats,
			     const Cfg& goal, const Cfg& increment, 
			     string vc_method,
			     int noNeighbors, 
			     CDInfo& _cdInfo,
			     vector<Cfg*>& cfgs);

  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, 
			     int* n_ticks, 
			     double positionRes, double orientationRes, double rd_res = .05);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks);

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

 protected:
  void StoreData();

  static int m_numOfJoints;

  vector<double> link_lengths;
  vector<int> link_orientations;
  static Link* link_tree;
  static vector<Link*> actual_links;

 public:
  static bool is_closed_chain;
};

#endif
