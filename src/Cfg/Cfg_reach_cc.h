#ifndef Cfg_reach_cc_h
#define Cfg_reach_cc_h

#include "Cfg_free_tree.h"
#include "VirtualLink.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableCfgs
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class Cfg_reach_cc : public Cfg_free_tree {
 public:
  Cfg_reach_cc();
  Cfg_reach_cc(const Vector6D& base, const vector<double>& len, const vector<int>& ori);
  Cfg_reach_cc(const Cfg&c);
  virtual ~Cfg_reach_cc();

  #ifdef _PARALLEL
    void define_type(stapl::typer &t)
    {
      Cfg_free_tree::define_type(t);
      t.member(link_lengths);
      t.member(link_orientations);

    }
 #endif

  static void initialize_link_tree(const char* filename);

  virtual const string GetName() const { return "Cfg_reach_cc"; }
  static size_t GetNumOfJoints() { return m_dof-6; }
  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double, bool _norm=true);
  virtual void divide(const Cfg&, double);
  virtual Cfg& operator=(const Cfg&);
  virtual void WeightedSum(const Cfg&, const Cfg&, double weight = 0.5);
  virtual void c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d);

  virtual bool isWithinResolution(const Cfg &c,
				  double positionRes,
				  double orientationRes) const;

  virtual bool ConfigureRobot(Environment*) const;

  virtual void GetRandomCfgCenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  virtual void GetRandomRay(double incr, Environment* env,shared_ptr <DistanceMetricMethod> dm, bool _norm=true);
   virtual Vector3d GetRobotCenterofMass(Environment*) const;

  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal,
			     int* n_ticks,
			     double positionRes, double orientationRes, double rd_res = .05);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks);

  ostream& print(ostream& os) const;
  virtual ostream& print_base(ostream& os) const;
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

  vector<double> link_lengths;
  vector<int> link_orientations;
  static Link* link_tree;
  static vector<Link*> actual_links;

 public:
  static bool is_closed_chain;
};

#endif
