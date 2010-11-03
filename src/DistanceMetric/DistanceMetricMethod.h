#ifndef DistanceMetricMethod_h
#define DistanceMetricMethod_h

#include "LabeledObject.h"
#include "Roadmap.h"
#include "Clock_Class.h"
#include "CfgTypes.h"

#include <boost/mpl/list.hpp>
using namespace std;

template <class CFG, class WEIGHT> class LocalPlannerMethod;
class GMSPolyhedron;
class MPProblem;

const double MAX_DIST =  1e10;

const int CS = 0;   ///< Type CS: Configuration space distance metric
const int WS = 1;   ///< Type WS: Workspace distance metric 


/**This is the interface for all distance metric methods(euclidean, 
  *scaledEuclidean, minkowski, manhattan, com, etc.).
  */
class DistanceMetricMethod  : public LabeledObject {
 public:
  DistanceMetricMethod();
  DistanceMetricMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  virtual ~DistanceMetricMethod();

  virtual char* GetName() const = 0;
  char* name() const {return GetName(); }
  virtual void SetDefault() = 0;

  virtual bool operator==(const DistanceMetricMethod& dm) const;

  virtual void PrintOptions(ostream& os) const = 0;

  template <class CFG, class WEIGHT>
  vector<typename RoadmapGraph<CFG, WEIGHT>::VID> RangeQuery(Roadmap<CFG, WEIGHT>* rm,
                   typename RoadmapGraph<CFG, WEIGHT>::VID in_query, double in_radius);
  
  template <class CFG, class WEIGHT>
  vector<typename RoadmapGraph<CFG, WEIGHT>::VID> RangeQuery(Roadmap<CFG, WEIGHT>* rm,
                         CFG in_query, double in_radius); 
  
  virtual DistanceMetricMethod* CreateCopy() = 0;

  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) = 0;
  virtual void ScaleCfg(Environment* env, double length, Cfg& o, Cfg& c);

public:
    double m_distance_time;

 protected:
  int type; ///<WS or CS. Used to classify metrics.
};
ostream& operator<< (ostream& _os, const DistanceMetricMethod& dm);


/**This computes the euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class EuclideanDistance : public DistanceMetricMethod {
 public:
  EuclideanDistance();
  EuclideanDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  virtual ~EuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();
  
  virtual void PrintOptions(ostream& os) const;

  /**This method calculates 
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2....+(c1n-c2n)^2).
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  virtual void ScaleCfg(Environment* env, double length, Cfg& o, Cfg& c);

 protected:
  virtual double ScaledDistance(Environment* env, const Cfg& _c1, const Cfg& _c2, double sValue);
  double _ScaledDistance(Environment* env, const Cfg& _c1, const Cfg& _c2, double sValue);
};


class KnotTheoryDistance : public EuclideanDistance {
 public:
  KnotTheoryDistance();
  KnotTheoryDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
   ~KnotTheoryDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual void PrintOptions(ostream&)const;
  virtual DistanceMetricMethod* CreateCopy();
 
 /**This method calculates 
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2....+(c1n-c2n)^2).
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
 // virtual void ScaleCfg(Environment* env, double length, Cfg& o, Cfg& c);

// protected:
 // virtual double ScaledDistance(Environment* env, const Cfg& _c1, const Cfg& _c2, double sValue);
};

/**This computes the scaled euclidean distance between two cfgs.  This 
  *class is derived off of DistanceMetricMethod.
  */
class ScaledEuclideanDistance : public EuclideanDistance {
 public:
  ScaledEuclideanDistance();
  ScaledEuclideanDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  virtual ~ScaledEuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual bool operator==(const ScaledEuclideanDistance& dm) const;
  virtual void PrintOptions(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *sqrt(s*(Position Magnitude)^ + (1-s)*(Orientation Magnitude)^2)
   
    *Position Magnitude is eulidean distance of position part of dimensions.
    *Position Magnitude is eulidean distance of orientation part of dimensions.
    *Usually first 3 dimensions are positions and rest of them are orientations.
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);

  double GetS() const { return sValue; }

 protected:
  /**Scale for Euludean distance of position part. 
    *Should between [0,1].
    */
  double sValue; ///<For Euclidean distance of position part.
};


/**This computes the euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class UniformEuclideanDistance : public DistanceMetricMethod {
 public:
  UniformEuclideanDistance(bool _useRotational = false);
  virtual ~UniformEuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual void PrintOptions(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *sqrt(
    *      (c11-c21)^2+(c12-c22)^2+(c13-c23)^2
              +   
           MIN((c14-c24)^2, (1-abs(c14-c24))^2)....+MIN((c1n-c2n)^2, (1-(c1n-c2n))^2)
         )
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  
 protected:
  bool useRotational;
};


/**This computes the pure euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class PureEuclideanDistance : public DistanceMetricMethod {
 public:
  PureEuclideanDistance();
  PureEuclideanDistance(string in_strLabel, int _useRotational);
  virtual ~PureEuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual void PrintOptions(ostream& _os) const;
  virtual DistanceMetricMethod *CreateCopy();

  /**This method calculates 
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2....+(c1n-c2n)^2).
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *
    */
    
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
};


/**This computes the minkowski distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class MinkowskiDistance : public DistanceMetricMethod {
 public:
  MinkowskiDistance();
  MinkowskiDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  virtual ~MinkowskiDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  bool operator==(const MinkowskiDistance& dm) const;
  virtual void PrintOptions(ostream& _os) const;
  virtual DistanceMetricMethod *CreateCopy();

  /**This method calculates 
    *pow( (c11-c21)^r1+(c12-c22)^r1+(c13-c23)^r1+(c14-c24)^r2....+(c1n-c2n)^r2 , r3)
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *Usually first 3 dimensions are positions and rest of them are orientations.
    *position part using r1 as power factor, and orientation part using r2 as power factor.
    *usually, r1=r2 and r3=1/r1.
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);

  double GetR1() const { return r1; }
  double GetR2() const { return r2; }
  double GetR3() const { return r3; }

 protected:
  /**Power factors for Minkowski Distace.
    */
  double r1; ///<For position part.
  double r2; ///<For rotation part.
  double r3; ///<For calculating root.
};


/**This computes the manhattan distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class ManhattanDistance : public DistanceMetricMethod {
 public:
  ManhattanDistance();
  ManhattanDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  virtual ~ManhattanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();
  
  virtual void PrintOptions(ostream& os) const;

  /**This method calculates 
    *( |c11-c21|+|c12-c22|+...+|c1n-c2n| ).
    *
    *Here |A| is absolute value of A.
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
};


/**This computes the euclidean distance of only the position part between 
  *two cfgs.  This class is derived off of DistanceMetricMethod.
  */
class CenterOfMassDistance : public DistanceMetricMethod {
 public:
  CenterOfMassDistance();
  CenterOfMassDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  virtual ~CenterOfMassDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();

  virtual void PrintOptions(ostream& os) const;
  
  /**This method calculates
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2).
    *This method only Euclidean Distance of position part and 
    *assumed that the first 3 dimension of Cfg are for position.
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
    return Distance(_c1, _c2);
  }
  virtual double Distance(const Cfg& _c1, const Cfg& _c2);
};


class RmsdDistance : public EuclideanDistance {
 public:
  RmsdDistance();
  RmsdDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  ~RmsdDistance();

  virtual char* GetName() const;
  virtual DistanceMetricMethod* CreateCopy();

  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  virtual vector<Vector3D> GetCoordinatesForRMSD(const Cfg &c, Environment *env);
  double RMSD(vector<Vector3D> x, vector<Vector3D> y, int dim);
};


class LPSweptDistance : public DistanceMetricMethod {
 public:
  LPSweptDistance();
  LPSweptDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  LPSweptDistance(LocalPlannerMethod<CfgType, WeightType>* _lp_method, double pos_res = 0.1, double ori_res = 0.1, bool bbox = false);
  ~LPSweptDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();

  virtual void PrintOptions(ostream& os) const;
  
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  double SweptDistance(Environment* env, const vector<GMSPolyhedron>& poly1, const vector<GMSPolyhedron>& poly2);

 protected:
  LocalPlannerMethod<CfgType, WeightType>* lp_method;
  double positionRes, orientationRes;
  bool use_bbox;
};


class BinaryLPSweptDistance : public DistanceMetricMethod {
 public:
  BinaryLPSweptDistance();
  BinaryLPSweptDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true);
  BinaryLPSweptDistance(LocalPlannerMethod<CfgType, WeightType>* _lp_method, double pos_res = 0.1, double ori_res = 0.1, double tolerance = 0.01, int max_attempts = 100, bool bbox = false);
  ~BinaryLPSweptDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();

  virtual void PrintOptions(ostream& os) const;
  
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  virtual double DistanceCalc(Environment* env, const Cfg& _c1, const Cfg& _c2, double pos_res, double ori_res);
  double SweptDistance(Environment* env, const vector<GMSPolyhedron>& poly1, const vector<GMSPolyhedron>& poly2);

 protected:
  LocalPlannerMethod<CfgType, WeightType>* lp_method;
  double positionRes, orientationRes, tolerance;
  int max_attempts;
  int dist_calls_count;
  bool use_bbox;
};


#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
class ReachableDistance : public DistanceMetricMethod {
 public:
  ReachableDistance() {}
  ReachableDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn = true) : DistanceMetricMethod(in_Node, in_pProblem, warn) {
    type = CS;
  }
  virtual ~ReachableDistance() {}
  virtual char* GetName() const { return "reachable"; }
  virtual void SetDefault() {}
  virtual DistanceMetricMethod* CreateCopy() {
    DistanceMetricMethod* _copy = new ReachableDistance(*this);
    return _copy;
  }
  virtual void PrintOptions(ostream& os) const;
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
};
#endif


template <class CFG, class WEIGHT>
 vector<typename RoadmapGraph<CFG, WEIGHT>::VID> DistanceMetricMethod::
RangeQuery(Roadmap<CFG, WEIGHT>* rm, typename RoadmapGraph<CFG, WEIGHT>::VID in_query, double in_radius) {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  vector<VID> returnVec;
  RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;
  Environment* _env = rm->GetEnvironment();


  Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  vector<VID> vec_vids;
  pMap->GetVerticesVID(vec_vids);
  typename vector<VID>::iterator itr;
  for(itr = vec_vids.begin(); itr != vec_vids.end(); ++itr)
  {
    if(in_query == *itr) continue;
  double dist = this->Distance(_env, (*(pMap->find_vertex(in_query))).property(), (*(pMap->find_vertex(*itr))).property());
    //cout << "Distance = " << dist << " Radius = " << in_radius << endl;
    if( dist< in_radius) {
      returnVec.push_back(*itr);
    }
  }
  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();

  return returnVec;
}


template <class CFG, class WEIGHT>
vector<typename RoadmapGraph<CFG, WEIGHT>::VID> DistanceMetricMethod::
RangeQuery(Roadmap<CFG, WEIGHT>* rm, CFG in_query, double in_radius) {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  vector<VID> returnVec;
  RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;
  Environment* _env = rm->GetEnvironment();
  
  Clock_Class distance_time;
  distance_time.StartClock("distance_time");
  


  vector<VID> vec_vids;
  pMap->GetVerticesVID(vec_vids);
  typename vector<VID>::iterator itr;

  for(itr = vec_vids.begin(); itr != vec_vids.end(); ++itr)
  {
    if(in_query == (*(pMap->find_vertex(*itr))).property()) continue;
    if(this->Distance(_env, in_query, (*(pMap->find_vertex(*itr))).property()) < in_radius) {
      returnVec.push_back(*itr);
    }
  }
  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();

  return returnVec;
}

#endif
