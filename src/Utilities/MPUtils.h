#ifndef MPUTILS_H_
#define MPUTILS_H_

#include <map>
#include <string>

#include <boost/function.hpp>
#include "boost/shared_ptr.hpp"
#include "boost/mpl/list.hpp"
#include "boost/mpl/sort.hpp"
#include "boost/type_traits/is_base_of.hpp"
#include "boost/mpl/begin.hpp"
#include "boost/mpl/end.hpp"
#include "boost/mpl/next_prior.hpp"

using boost::shared_ptr;

#include "Vector.h"
#include "Point.h"
using namespace mathtool;

#include "IOUtils.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

//forward declarations
class StatClass;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Constants
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define TWOPI           (PI*2.0)
#define DEGTORAD (PI/180.0)
#define RADTODEG (180.0/PI)


#define NULL_WT_INFO -999               ///< to pad weight fields for graph conversions
#define INVALID_LP -999                 ///< invalid local planner id
#define INVALID_RNGSEED -999            ///< invalid seed value for Random Number Generator
#define MAX_INT  999999999
#define INVALID_INT -999
#define MAX_DBL  999999999.99999
#define INVALID_DBL -999

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Collision Detection
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//Need to define at least one type of collision detection library
#ifndef USE_VCLIP
#ifndef USE_RAPID
#ifndef USE_PQP
#ifndef USE_SOLID
#ifndef NO_CD_USE
#error You have to specify at least one collision detection library.
#endif
#endif
#endif
#endif
#endif

/// Legal Types of Collision Detecters
enum cd_predefined {
  /// voronoi clip
#ifdef USE_VCLIP
  VCLIP, 
#endif
  /// Robust and Accurate Polygon Interference Detection
#ifdef USE_RAPID
  RAPID,
#endif
  /// Proximity Query Package
#ifdef USE_PQP
  PROXIMITYQUERYPACKAGE,
#endif
  /// SOLID
#ifdef USE_SOLID
  SOLID,
#endif
  INSIDE_SPHERES,
  BOUNDING_SPHERES,
  CD_USER1
};    

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Random Number Generation
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//return non-negative double-prevision floating-point values 
//uniformly distributed over the interval [0.0, 1.0)
double DRand();

//return non-negative long integers uniformly distributed over the interval [0, 2**31)
long LRand();

//return signed long integers uniformly distributed over the interval [-2**31, 2**31)
long MRand();

// normally(gaussian) distributed random number generator.
// when reset is 1, it reset the internal static variable and return 0.0
double GRand(bool _reset = false);

//Same as GRand, but one can specify the mean and stdev of the distribution
double GaussianDistribution(double _mean, double _stdev);

/* use seedval as the seed
*/
long SRand(long _seed = 0x1234ABCD);

/* "baseSeed" is a static variable in this function
   we use baseSeed, methodName and nextNodeIndex to generate a deterministic seed,
   then call seed48()
   when reset is 1, baseSeed will be reset
   */
long SRand(string _methodName, int _nextNodeIndex, long _base = 0x1234ABCD, bool _reset = false);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Simple Utilities (Angular Distance and Compare Second)
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/**Normalize a value into the range [-1,1)
 */
double Normalize(double _a);

/**Calculate the minimum DIRECTED angular distance 
 *between two angles normalized to 1.0 
 */
double DirectedAngularDistance(double _a, double _b);

/* Compare the second of a pair */
template <typename T, typename U>
class CompareSecond {
 public:
  bool operator()(const pair<T, U>& _a, const pair<T, U>& _b) const {
    return _a.second < _b.second;
  }
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Compose Functions
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename InputIterator, typename BinaryOperator, typename UnaryOperator>  
struct Compose {
  bool operator()(InputIterator _first, InputIterator _last, 
      BinaryOperator _binaryOp, UnaryOperator _op) {
    if (_first == _last)
      return false;
    else {
      bool result = _op(*_first++);
      while (_first != _last)
        result = _binaryOp(result, _op(*_first++)); 
      return result;  
    }
  }  
};

template <typename InputIterator, typename UnaryOperator>  
struct Compose<InputIterator, logical_and<bool>, UnaryOperator> {
  bool operator()(InputIterator _first, InputIterator _last, 
      logical_and<bool> _binaryOp, UnaryOperator _op) {
    if (_first == _last) 
      return false;
    else {
      bool result = _op(*_first++);
      if (result == false)
        return result;	
      while (_first != _last) {
        result = _binaryOp(result, _op(*_first++)); 
        if (result == false)
          return result;	
      }
      return result;  
    }
  }  
};

template <typename InputIterator, typename UnaryOperator>  
struct Compose<InputIterator, logical_or<bool>, UnaryOperator> {
  bool operator()(InputIterator _first, InputIterator _last, 
      logical_or<bool> _binaryOp, UnaryOperator _op) {
    if (_first == _last)
      return false;
    else {
      bool result = _op(*_first++);
      if (result == true)
        return result;	
      while (_first != _last) {
        result = _binaryOp(result, _op(*_first++)); 
        if (result == true)
          return result;	
      }
      return result;  
    }
  }  
};

template <typename InputIterator, typename UnaryOperator>  
struct ComposeNegate {
  bool operator()(InputIterator _it, UnaryOperator _op) {
    return !_op(*_it); 
  }  
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Containers
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//MethodSet defines basic method container class to derive from
//  (for classes like DistanceMetric, LocalPlanner, NeighborhoodFinder, Sampler etc)
//
//derived class must specify the Method type and MethodTypeList
//  e.g., NeighborhoodFinder: Method = NeighborhoodFinderMethod
//                            MethodTypeList = boost::mpl::list<BruteForceNF,BandsNF,...>
//  e.g., LocalPlanner: Method = LocalPlannerMethod
//                      MethodTypeList = boost::mpl::list<Straightline,RotateAtS,...>
///////////////////////////////////////////////////////////////////////////////////////////

template<typename MPTraits, typename Method>
struct MethodFactory {
  boost::shared_ptr<Method> operator()(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) const {
    return boost::shared_ptr<Method>(new Method(_problem, _node));
  }
};

template<typename MPTraits, typename Method>
class MethodSet {
  public:
    typedef boost::shared_ptr<Method> MethodPointer;
    typedef typename map<string, MethodPointer>::iterator MIT;
    typedef typename map<string, MethodPointer>::const_iterator CMIT;

    template<typename MethodTypeList>
      MethodSet(const MethodTypeList& _etl, const string& _name) : m_default(""), m_name(_name) {
        AddToUniverse(typename boost::mpl::begin<MethodTypeList>::type(), typename boost::mpl::end<MethodTypeList>::type());
      }

    void ParseXML(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node){
      XMLNodeReader::childiterator citr;
      for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
        if (!AddMethod(_problem, *citr, citr->getName())){
          citr->warnUnknownNode();
        }
      }
    }

    bool AddMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node, const string& _name) {
      if(m_universe.find(_name) != m_universe.end()) {
        MethodPointer e = m_universe[_name](_problem, _node);
        return AddMethod(e, e->GetLabel());
      }
      return false;
    }

    bool AddMethod(MethodPointer _e, const string& _label) {
      if(m_universe.find(_e->GetName()) != m_universe.end()) {
        _e->SetLabel(_label);
        if(m_elements.empty()) 
          m_default = _label;
        if(m_elements.find(_label) == m_elements.end()) 
          m_elements[_label] = _e;
        else 
          cerr << "\nWarning, method list already has a pointer associated with \"" << _label << "\", not added\n";
        return true;
      }
      else{
        cerr << "Error. Method \"" << _e->GetName() << "\" is not contained within the motion planning universe. Exiting." << endl;
        exit(1);
      }
    }

    MethodPointer GetMethod(const string& _label) {
      MethodPointer element;
      if(_label == "") 
        element = m_elements[m_default];
      else 
        element = m_elements[_label];
      if(element.get() == NULL) {
        cerr << "\n\tError, requesting element with name \"" << _label << "\" which does not exist in " << m_name << ".\n";
        for(CMIT mit = Begin(); mit != End(); ++mit)
          if(mit->second.get() != NULL)
            cerr << " \"" << mit->first << "\"";
        cerr << "\n\texiting.\n";
        exit(-1);
      }
      return element;
    }

    void SetMPProblem(typename MPTraits::MPProblemType* _problem){
      for(MIT mit = m_elements.begin(); mit!=m_elements.end(); mit++){
        mit->second->SetMPProblem(_problem);
      }
    }

    void PrintOptions(ostream& _os) const {
      size_t count = 0;
      _os << endl << m_name << " has these methods available::" << endl << endl;
      for(CMIT mit = Begin(); mit != End(); ++mit){
        _os << ++count << ") \"" << mit->first << "\" (" << mit->second->GetName() << ")" << endl;
        mit->second->PrintOptions(_os);
        _os << endl;
      }
      _os << endl;
    }

    CMIT Begin() const { return m_elements.begin(); }
    CMIT End() const { return m_elements.end(); }
    MIT Begin() { return m_elements.begin(); }
    MIT End() { return m_elements.end(); }

  protected:
    typedef boost::function<MethodPointer(typename MPTraits::MPProblemType*, XMLNodeReader&)> FactoryType;
    
    template <typename Last> 
      void AddToUniverse(Last, Last){}

    template <typename First, typename Last>
      void AddToUniverse(First, Last) {
        typename boost::mpl::deref<First>::type first;
        m_universe[first.GetName()] = MethodFactory<MPTraits, typename boost::mpl::deref<First>::type>();
        AddToUniverse(typename boost::mpl::next<First>::type(), Last());
      }

    string m_default, m_name;
    map<string, FactoryType> m_universe;
    map<string, MethodPointer> m_elements;
};



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// MPBaseObject
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
class MPBaseObject {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    MPBaseObject(MPProblemType* _problem = NULL, string _label = "", string _name = "", bool _debug = false) :
      m_problem(_problem), m_label(_label), m_name(_name), m_debug(_debug) {};
    MPBaseObject(MPProblemType* _problem, XMLNodeReader& _node, string _name="") : 
      m_problem(_problem), m_label(""), m_name(_name), m_debug(false) { 
        ParseXML(_node); 
      };

    virtual ~MPBaseObject() {}

    virtual void ParseXML(XMLNodeReader& _node) {
      m_label = _node.stringXMLParameter("label", false, "", "Label Identifier");
      m_debug = _node.boolXMLParameter("debug", false, false, "Run-time debug on(true)/off(false)");
      m_recordKeep = _node.boolXMLParameter("recordKeep", false, false, "Keeping track of algorithmic statistics, on(true)/off(false)");
    };

    MPProblemType* GetMPProblem() const {return m_problem;}
    virtual void SetMPProblem(MPProblemType* _m){m_problem = _m;}
    virtual void PrintOptions(ostream& _os) {};
    string GetLabel() const {return m_label;}
    void SetLabel(string _s) {m_label = _s;}
    string GetName()  const {return m_name;}
    void SetName (string _s) {m_name  = _s;}
    string GetNameAndLabel() const {return m_name + "::" + m_label;}
    bool GetDebug() const {return m_debug;}
    void SetDebug(bool _d) {m_debug = _d;}

  private:
    MPProblemType* m_problem;
    string m_label;

  protected:
    string m_name;
    bool m_debug;
    bool m_recordKeep;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Cfg Utilities
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<class CfgType, class Environment>
bool
IsWithinResolution(const CfgType& _cfg1, const CfgType& _cfg2, Environment* _env) {
  CfgType diff = _cfg1 - _cfg2;
  return diff->PositionMagnitude() <= _env->GetPositionRes() 
    && diff->OrientationMagnitude() <= _env->GetOrientationRes();
}	

/** pt1 & pt2 are two endpts of a line segment
 * find the closest point to the current cfg on that line segment
 * it could be one of the two endpoints of course
 */
template<class CfgType>
CfgType
ClosestPtOnLineSegment(const CfgType& _current, const CfgType& _p1, const CfgType& _p2) {
  CfgType b = _p2 - _p1;
  CfgType c = _current - _p1;  
  
  double bDotC = 0;
  double bSquared = 0;

  vector<double>::const_iterator itb, itc;
  for (itb = b.GetData().begin(), itc = c.GetData().begin(); itb != b.GetData().end(); ++itb, ++itc) {
    bDotC += (*itb)*(*itc);
    bSquared += (*itb)*(*itb);
  }

  if (bDotC <= 0) {
    return _p1;
  } 
  else if (bDotC >= bSquared) {
    return _p2;
  } 
  else {
    CfgType result = b*(bDotC/bSquared) + _p1 ;
    return result;
  }
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// GetCentroid
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<template<class CFG, class WEIGHT> class RDMP, class CFG, class WEIGHT>
CFG
GetCentroid(RDMP<CFG, WEIGHT>* _graph, vector<typename RDMP<CFG, WEIGHT>::VID>& _cc){
  CFG center;
  for(size_t i = 0; i < _cc.size(); i++) {

    CFG& cfg = (*(_graph->find_vertex(_cc[i]))).property();
    center += cfg;
  }
  center /= _cc.size();
  return center;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// RRTExpand
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*Basic utility for "growing" a RRT tree.  Assumed to be given a start node, and he goal node to grow towards.
  Resulting node extended towards the goal is passed by reference and modified.  Returned boolean relays whether the
  growth was succesful or not.

  _mp -> Used to obtain information about the problem at hand
  strVcmethod -> Used for VC calls
  dm_label -> Used to extract DistanceMetricMethod pointer
  start -> Location of Cfg on tree to grow from
  dir -> Direction to grow towards
  new_cfg -> New Cfg to be added to the tree; passed by reference
  delta -> Maximum distance to grow
  obsDist -> Distance away from object the new node neads to at least be
  */

template<class MPTraits>
bool
RRTExpand(typename MPTraits::MPProblemType* _mp, 
    string _vc, string _dm, 
    typename MPTraits::CfgType _start, 
    typename MPTraits::CfgType _dir, 
    typename MPTraits::CfgType& _newCfg, 
    double _delta, int& _weight, CDInfo& _cdInfo,
    double _posRes, double _oriRes){
  
  //Setup...primarily for collision checks that occur later on
  StatClass* stats = _mp->GetStatClass();
  Environment* env = _mp->GetEnvironment();
  typename MPTraits::MPProblemType::DistanceMetricPointer dm = _mp->GetDistanceMetric(_dm);
  typename MPTraits::MPProblemType::ValidityCheckerPointer vc = _mp->GetValidityChecker(_vc);
  string callee("RRTUtility::RRTExpand");

  typename vector<typename MPTraits::CfgType>::iterator startCIterator;
  typename MPTraits::CfgType incr, tick = _start, previous = _start;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick,_dir,&nTicks, _posRes, _oriRes);
  _weight = nTicks;

  //Move out from start towards dir, bounded by number of ticks allowed at a given resolution.  Delta + obsDist are
  //given to the function, and are user defined.
  while(!collision && dm->Distance(env,_start,tick) <= _delta) {
    previous = tick;
    tick += incr; //Increment tick
    if(!(tick.InBoundary(env)) || !(vc->IsValid(tick, env, *stats, _cdInfo, &callee))){
      collision = true; //Found a collision; return previous tick, as it is collision-free
    }
    ++ticker;
    if (ticker == nTicks){ //Have we reached the max amount of increments?
      break;
    }
  }
  if(previous != _start){ //Did we go anywhere?
    _newCfg = previous;//Last Cfg pushed back is the final tick allowed
    return true;     
  }
  //Didn't find a place to go :(
  else
    return false;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// Geometry Utils
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------------
//PtInTriangle: determine if point _P is in triange defined by (_A,_B,_C)
//----------------------------------------------------------------------------
bool PtInTriangle(const Point2d& _A, const Point2d& _B, const Point2d& _C, const Point2d & _P);

//----------------------------------------------------------------------------
// CHECKS IF 2D POINT P IS IN TRIANGLE ABC. RETURNS 1 IF IN, 0 IF OUT
//   uses barycentric coordinated to compute this and return the uv-coords
//   for potential usage later
//----------------------------------------------------------------------------
bool PtInTriangle(const Point2d& _A, const Point2d& _B, const Point2d& _C, const Point2d & _P,
    double& _u, double& _v);


//----------------------------------------------------------------------------
// GetPtFromBarycentricCoords: given triange defined by _A,_B,_C, return the
// point inside triangle defined by barycentric coords. _u,_v
//----------------------------------------------------------------------------
Point3d GetPtFromBarycentricCoords(const Point3d& _A, const Point3d& _B, const Point3d& _C, double _u, double _v); 


//----------------------------------------------------------------------------
//NormalizeTheta: given a value, lock it into the range -PI to PI
//----------------------------------------------------------------------------
double NormalizeTheta(double _theta);

#endif

