#ifndef __VIRTUAL_LINK__H___
#define __VIRTUAL_LINK__H___

#include <vector>
#include <list>
using namespace std;

#include "boost/random.hpp"

#define EPS_ZERO 0.00001
#ifndef PI
#define PI      3.1415926536
#endif
#define TWO_PI  6.2831853072


struct Range {
  double min;
  double max;

  Range(){};
  Range(double m1, double m2){
    if (m1 > m2){
      max = m1;
      min = m2;
    }else{
      min = m1;
      max = m2;
    }
  }
  double Size() const{
    return max-min;
  }
  
  bool HasIntersection(const Range &r) {
    if (r.min > max || r.max < min)
      return false;
    else
      return true;
  }
  friend ostream& operator <<(ostream &o, const Range& r);
  
};

class Link {
  int ID;
  Link *parent;
  Link *sibling;

  Link *leftChild;
  Link *rightChild;
  
  Range reachableRange; 
  Range availableRange; 

  double length;
  int convexity; //1 convex, 0 flat, -1 concave

  double angLeft;
  double angRight;
  double angJoint;
  pair<double,double> coord;

  static int IDCount;  
 
public:
  Link(double rmin, double rmax);
  Link(Link* c0, Link* c1);
  ~Link();

  double SampleLength(bool bSampleConvex, double);
  double SampleLength(boost::variate_generator<boost::rand48&,
		                               boost::uniform_real<> 
		                              >& rand);

  void  ExportTreeLinkLength(vector<double> &lengths);
  int  ImportTreeLinkLength(const vector<double> &lengths, int index = 0);

  void ExportTreeLinkReachableRange(vector<Range>& ranges) const;
  
  void  ExportTreeLinkAvailableRange(vector<Range> &ranges);
  int  ImportTreeLinkAvailableRange(const vector<Range> &ranges, int index = 0);
  
  void  ExportTreeLinkConvexity(vector<int> &convexities);
  int  ImportTreeLinkConvexity(const vector<int> &convexities, int index = 0);
  
  void  ExportTreeLinkLength(vector<double> &lengths, vector<int> &convexities);
  int  ImportTreeLinkLength(const vector<double> &lengths, const vector<int> &convexities,  int index = 0);
  
  bool RecursiveSample(double l = -1, bool bSampleConvex = true, double gama=0.5);
  bool RecursiveSample(boost::variate_generator<boost::rand48&,
		                                boost::uniform_real<>
		                               >& rand, 
		       double l = -1);

  bool SetLength(double l);
  
  double GetLength(){return length;}
  int GetID(){return ID;}
  void PrintLink(ostream& os);
  void PrintTree(ostream& os);
  void ResetTree();  
  
  double AngleToParent();
  double AngleToSibling();
  double AngleToLeft();
  double AngleToRight();
  
  bool CanClose();
  bool CanRecursiveClose();
  void FindBreachesRecursive(list<Link*> &breaches);
  
  void CalculateReachableRanges();
  void CalculateAngles();
  Range GetAvailableRange(){
    cout << "ID: " << ID;
    cout << availableRange << endl;
    return availableRange;
  };

  static void
  InterpolateLinkLength(const vector<double> &start, 
                        const vector<double> &increment, 
                        vector<double> &intermediate, 
                        int index);

  static void
  InterpolateLinkLength(const vector<double> &start, 
                        const vector<double> &goal, 
                        vector<double> &intermediate, 
                        int nSteps,
                        int index);
  
  static void
  FindIncrement(const vector<double> &start, 
                const vector<double> &goal, 
                vector<double> &increment, 
                int nSteps);
  
  void RecursiveBuildAvailableRange(bool bFixedConvexity = false);
  
  friend Link *FindLCA(Link *left, Link *right);
  friend Link *BuildTree(int i, int j, vector<Link*>& baseLinks);
  friend double CalculateJointAngle(Link *prev, Link *next);
};

Link *FindLCA(Link *left, Link *right);
double CalculateJointAngle(Link *left, Link *right);
double CosineAngle(double a, double b, double c);
Range RangeIntersection(const Range &r1, const Range &r2);
Range RangeUnion(const Range &r1, const Range &r2);
Range RangePlus(const Range &r1, const Range &r2);
Range RangeMinus(const Range &r1, const Range &r2);
Link *BuildTree(int i, int j, vector<Link*>& baseLinks );

#endif
