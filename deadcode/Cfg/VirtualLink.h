#ifndef VIRTUAL_LINK_H_
#define VIRTUAL_LINK_H_

#include <vector>
#include <list>
#include <set>
using namespace std;

#include "boost/random.hpp"

#define EPS_ZERO 0.00001
#ifndef PI
#define PI      3.1415926536
#endif
#define TWO_PI  6.2831853072

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
struct Range
{
  double min, max;

  Range(double m1 = 0, double m2 = 0)
  {
    if (m1 > m2)
    {
      max = m1;
      min = m2;
    }
    else
    {
      min = m1;
      max = m2;
    }
  }

  double Size() const { return max - min; }

  bool HasIntersection(const Range &r) { return !(r.min > max || r.max < min); }

  friend ostream& operator<<(ostream &o, const Range& r);
};


class Link;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
struct FamilyInfo
{
  Link *parent, *sibling;
  double angLeft, angRight, angJoint;

  FamilyInfo(Link *p, Link *s) : parent(p), sibling (s), angLeft(0), angRight(0), angJoint(0) {}
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class Link
{
  static int IDCount;
  int ID;
  int convexity; //1 convex, 0 flat, -1 concave

  //Link *leftChild, *rightChild;

  Range reachableRange, availableRange;

  double length;
  pair<double,double> coord;

  vector<FamilyInfo> families;

public:
  Link *leftChild, *rightChild; //todo: make private and fix rest of code to treat as private
  Link(double rmin, double rmax);
  Link(int myID, double rmin, double rmax);

  Link(Link* c0, Link* c1);
  Link(int myID, Link* c0, Link* c1);

  ~Link();

  void InitializeFromRange(double rmin, double rmax);
  void InitializeFromChildren(Link* c0, Link* c1);

  void UpdateARange(Link *parent, Link *sibling, bool bSampleConvex =false);

  double SampleLength(bool bSampleConvex, double);
  double SampleLength(boost::variate_generator<boost::rand48&,
		                               boost::uniform_real<>
		                              >& rand);

  void ExportTreeLinkLength(vector<double> &lengths);
  int  ImportTreeLinkLength(const vector<double> &lengths, int index = 0);

  void ExportTreeLinkReachableRange(vector<Range>& ranges) const;

  void ExportTreeLinkAvailableRange(vector<Range> &ranges);
  int  ImportTreeLinkAvailableRange(const vector<Range> &ranges, int index = 0);

  void ExportTreeLinkConvexity(vector<int> &convexities);
  int  ImportTreeLinkConvexity(const vector<int> &convexities, int index = 0);

  void ExportTreeLinkLength(vector<double> &lengths, vector<int> &convexities);
  int  ImportTreeLinkLength(const vector<double> &lengths, const vector<int> &convexities,  int index = 0);

  static void FindRootPathways(Link *leaf, vector<Link*> &curPathway, vector<vector<Link*> > &pathwayEnsemble);

  static bool MatchPathways(const vector<vector<Link *> > &pathEnsembleLeft, const vector<vector<Link *> > &pathEnsembleRight,
                            vector<Link *> &leftPath, vector<Link *> &rightPath);

  static double CalculateJointAngle(Link *prev, Link *next);

  bool RecursiveSample(double l = -1, bool bSampleConvex = true, double gama = 0.5);
  bool RecursiveSample(boost::variate_generator<boost::rand48&,
		                                boost::uniform_real<>
		                               >& rand,
		       double l = -1);

  bool SetLength(double l);
  double GetLength(){ return length; }

  int GetID(){ return ID; }

  void PrintLink(ostream& os);
  void PrintTree(ostream& os);

  void ResetTree();

  bool CanClose();
  bool CanRecursiveClose();

  void FindBreachesRecursive(list<Link*> &breaches);

  Range GetAvailableRange()
  {
    cout << "ID: " << ID << availableRange << endl;
    return availableRange;
  }

  void SetAvailableRange(const Range& range) { availableRange = range; }

  static void InterpolateLinkLength(const vector<double> &start,
                                    const vector<double> &increment,
                                    vector<double> &intermediate,
                                    int index);
  static void InterpolateLinkLength(const vector<double> &start,
                                    const vector<double> &goal,
                                    vector<double> &intermediate,
                                    int nSteps,
                                    int index);

  static void FindIncrement(const vector<double> &start,
                            const vector<double> &goal,
                            vector<double> &increment,
                            int nSteps);

  void RecursiveBuildAvailableRange(bool bFixedConvexity = false);

  friend Link *BuildTree(int i, int j, vector<Link*>& baseLinks);

  friend void seen_both_children(Link* link, set<Link*>& seen, set<Link*>& implied);
  friend void partition_loop(Link* link, set<Link*>& links_seen, vector<int>& seen, vector<int>& unseen);
  friend void get_actual_links_of(Link* link, vector<int>& ids);
  friend Link* get_parent_of(Link* link, const vector<int>& ear);
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
Link *BuildTree(int i, int j, vector<Link*>& baseLinks );

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
double CosineAngle(double a, double b, double c);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
Range RangeIntersection(const Range &r1, const Range &r2);
////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
Range RangeUnion(const Range &r1, const Range &r2);
////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
Range RangePlus(const Range &r1, const Range &r2);
////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
Range RangeMinus(const Range &r1, const Range &r2);

#endif
