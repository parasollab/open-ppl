/**
 * @file DistanceMetrics.h
 *
 * @author Daniel Vallejo
 * @date 8/21/1998
 */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DistanceMetrics_h
#define DistanceMetrics_h

//////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include <Graph.h> //to get def for VID
#include <GraphAlgo.h>
#include <RoadmapGraph.h>
#include <functional>

#include "OBPRMDef.h"
#include "util.h"
#include "LabeledObject.h"

#include "Clock_Class.h"
#include "PMPL_Container_Base.h"

/////////////////////////////////////////////////////////////////////////////////////////
//using stapl::VID;

const double MAX_DIST =  1e10;

class Cfg;
class MultiBody;
class Environment;
class n_str_param;

template <class CFG, class WEIGHT> class Roadmap;

class DistanceMetricMethod;
class EuclideanDistance;
class ScaledEuclideanDistance;
class MinkowskiDistance;
class ManhattanDistance;
class CenterOfMassDistance;
class RmsdDistance;
class LPSweptDistance;
class BinaryLPSweptDistance;
class KnotTheoryDistance;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
class ReachableDistance;
#endif

template <class CFG, class WEIGHT> class LocalPlannerMethod;
#include "CfgTypes.h"

const int CS = 0;   ///< Type CS: Configuration space distance metric
const int WS = 1;   ///< Type WS: Workspace distance metric 


/**This is the main distance metric class.  It contains two vectors: all 
  *and selected.  all contains all of the different types of distance 
  *metric methods.  selected contains only those selected by the user.
  */
namespace pmpl_detail { //hide DistanceMetricMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
   EuclideanDistance(),
   ScaledEuclideanDistance(),
   MinkowskiDistance(),
   ManhattanDistance(),
   CenterOfMassDistance(),
   RmsdDistance(),
   LPSweptDistance(),
   BinaryLPSweptDistance(),
   #if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
   ReachableDistance(), 
  #endif
   KnotTheoryDistance()
    > DistanceMetricMethodList;
}


class DistanceMetric : private PMPL_Container_Base< DistanceMetricMethod,
                    pmpl_detail::DistanceMetricMethodList>, public MPBaseObject {

private:
  typedef PMPL_Container_Base< DistanceMetricMethod, pmpl_detail::DistanceMetricMethodList> DistanceMetricContainer;

  public:
    typedef DistanceMetricContainer::method_pointer DistanceMetricPointer;

 

 public:
  DistanceMetric();
  DistanceMetric(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true);
  virtual ~DistanceMetric();

  bool ParseXML(MPProblem* in_pProblem, XMLNodeReader::childiterator citr);
  
 DistanceMetricPointer GetDMMethod(string in_strLabel);
 
 // virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  void AddDMMethod(string in_strLabel, DistanceMetricPointer in_ptr);




  void PrintUsage(ostream& _os) const;
  void PrintValues(ostream& _os) const;
  void PrintDefaults(ostream& _os) ;
  void PrintOptions(ostream& _os) const;

  /**Read information about DistanceMetricMethods selected from file.
    *@param _fname filename for data file.
    *@see ReadDMs(istream& _myistream)
    */
  //void ReadDMs(const char* _fname);
  /**Read information about DistanceMetricMethods selected from input stream.
    *@see WriteDMs for data format
    *@note if this method could not be able to understand
    *input file format, then error message will be post to 
    *standard output.
    */
  //void ReadDMs(istream& _myistream);
  /**Ouput information about selected DistanceMetricMethods to file.
    *@param _fname filename for data file.
    *@see WriteDMs(ostream& _myostream)
    */
  //void WriteDMs(const char* _fname) const;
  /**Ouput information about selected DistanceMetricMethods to output stream.
    *@note format: DM_NAME (a string) DM_PARMS (double, int, etc) 
    *for each DistanceMetricMethod.
    *if scaledEuclidean is encountered, then sValue will be printed.
    *if minkowski is encountered, r1, r2, and r3 will be printed.
    */
 // void WriteDMs(ostream& _myostream) const;

  /**Get Distance between two Configurations.
    *The distance is calculated by DistanceMetricMethod in selected vector. 
    *
    *@param _c1 Start Cfg
    *@param _c2 End Cfg
    *@return distance metrics between _c1 and _c2.
    *
    *@note there is the framework for multiple selected distance metrics, 
    *but now it only uses the first one in the selected vector.
    */
/*
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  virtual double Distance(Environment* env, const Cfg* _c1, const Cfg* _c2);
*/
  void ScaleCfg(Environment* env, double length, Cfg& o, Cfg& c);

public:
    double m_distance_time;
    string par_label;
};

/*Compare two distances in DIST_TYPE instances.
 *return (_cc1.second < _cc2.second)*/
 
template <class T>
class DIST_Compare : public binary_function<const pair<pair<T,T>,double>,
              const pair<pair<T,T>,double>,
              bool> {
 public:
  bool operator()(const pair<pair<T,T>,double> _cc1,
      const pair<pair<T,T>,double> _cc2) {
    return (_cc1.second < _cc2.second);
  }
};

template <class CFG>
class CFG_DIST_COMPARE_INDEX : public binary_function<const pair<int,double>, 
						      const pair<int,double>, bool> {
 public:
  bool operator()(const pair<int,double> _cc1,
		  const pair<int,double> _cc2) {
    return (_cc1.second < _cc2.second);
  }
  
};

template <class CFG>
class CFG_DIST_COMPARE : public binary_function<const pair<CFG,double>, 
              const pair<CFG,double>, bool> {
 public:
  bool operator()(const pair<CFG,double> _cc1,
      const pair<CFG,double> _cc2) {
    return (_cc1.second < _cc2.second);
  }
  
};


/**This is the interface for all distance metric methods(euclidean, 
  *scaledEuclidean, minkowski, manhattan, com, etc.).
  */
class DistanceMetricMethod  : public LabeledObject {
 public:
  DistanceMetricMethod(string in_strLabel);
  virtual ~DistanceMetricMethod();

  virtual char* GetName() const = 0;
  virtual void SetDefault() = 0;

  virtual bool operator==(const DistanceMetricMethod& dm) const;


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

//A new distance compare method that doesnt require a precomputed pair<cfg,double>
//proper constructor must be called!
template <class CFG>
class CFG_CFG_DIST_COMPARE : public binary_function<const CFG, const CFG, bool> {

  public:
    CFG_CFG_DIST_COMPARE(CFG& start_cfg, shared_ptr<DistanceMetricMethod> _dm, Environment* _env): 
        m_cfg(start_cfg), m_dm(_dm), m_env(_env) { };
    bool operator()(const CFG _cc1, const CFG _cc2) {
      double dcc1,dcc2;
      dcc1 = m_dm->Distance(m_env, m_cfg, _cc1);
      dcc2 = m_dm->Distance(m_env, m_cfg, _cc2);
      return (dcc1 < dcc2);
  }

  private:
    CFG_CFG_DIST_COMPARE() {};
    Environment* m_env;
  shared_ptr<DistanceMetricMethod> m_dm;
    CFG& m_cfg;
};


/**This computes the euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class EuclideanDistance : public DistanceMetricMethod {
 public:
  EuclideanDistance(string in_strLabel);
  virtual ~EuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
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
  virtual void ScaleCfg(Environment* env, double length, Cfg& o, Cfg& c);

 protected:
  virtual double ScaledDistance(Environment* env, const Cfg& _c1, const Cfg& _c2, double sValue);
  double _ScaledDistance(Environment* env, const Cfg& _c1, const Cfg& _c2, double sValue);
};


class KnotTheoryDistance : public EuclideanDistance {
 public:
  KnotTheoryDistance(string in_strLabel);
   ~KnotTheoryDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual  void PrintUsage(ostream&)const;
  virtual void PrintValues(ostream&)const;
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
  ScaledEuclideanDistance(string in_strLabel);
  ScaledEuclideanDistance(string in_strLabel, double _sValue);
  virtual ~ScaledEuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual bool operator==(const ScaledEuclideanDistance& dm) const;
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
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

  ///////
class UniformEuclideanDistance : public DistanceMetricMethod {
 public:
  UniformEuclideanDistance(string in_strLabel);
  UniformEuclideanDistance(string in_strLabel, int _useRotational);
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
  PureEuclideanDistance(string in_strLabel);
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
  MinkowskiDistance(string in_strLabel);
  virtual ~MinkowskiDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  bool operator==(const MinkowskiDistance& dm) const;
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
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
  ManhattanDistance(string in_strLabel);
  virtual ~ManhattanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();

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
  CenterOfMassDistance(string in_strLabel);
  virtual ~CenterOfMassDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();

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
  RmsdDistance(string in_strLabel);
  ~RmsdDistance();

  virtual char* GetName() const;
  virtual DistanceMetricMethod* CreateCopy();

  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  virtual vector<Vector3D> GetCoordinatesForRMSD(const Cfg &c, Environment *env);
  double RMSD(vector<Vector3D> x, vector<Vector3D> y, int dim);
};


class LPSweptDistance : public DistanceMetricMethod {
 public:
  LPSweptDistance(string in_strLabel);
  LPSweptDistance(string in_strLabel, LocalPlannerMethod<CfgType, WeightType>* _lp_method);
  LPSweptDistance(string in_strLabel, LocalPlannerMethod<CfgType, WeightType>* _lp_method, double pos_res, double ori_res, bool bbox);
  ~LPSweptDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();

  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  double SweptDistance(Environment* env, const vector<GMSPolyhedron>& poly1, const vector<GMSPolyhedron>& poly2);

 protected:
  LocalPlannerMethod<CfgType, WeightType>* lp_method;
  double positionRes, orientationRes;
  bool use_bbox;
};


class BinaryLPSweptDistance : public DistanceMetricMethod {
 public:
  BinaryLPSweptDistance(string in_strLabel);
  BinaryLPSweptDistance(string in_strLabel, LocalPlannerMethod<CfgType, WeightType>* _lp_method);
  BinaryLPSweptDistance(string in_strLabel, LocalPlannerMethod<CfgType, WeightType>* _lp_method, double pos_res, double ori_res, double tolerance, int max_attempts, bool bbox);
  ~BinaryLPSweptDistance();

  virtual char* GetName() const;
  virtual void SetDefault();
  virtual DistanceMetricMethod* CreateCopy();

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
  ReachableDistance(string in_strLabel) : DistanceMetricMethod(in_strLabel) {
    type = CS;
  }
  virtual ~ReachableDistance() {}
  virtual char* GetName() const { return "reachable"; }
  virtual void SetDefault() {}
  virtual DistanceMetricMethod* CreateCopy() {
    DistanceMetricMethod* _copy = new ReachableDistance(*this);
    return _copy;
  }
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
};
#endif


#include "Roadmap.h"
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
