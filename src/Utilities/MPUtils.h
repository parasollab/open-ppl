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
//     Region Expand Utilities 
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


// Maintains a vector of size 2 with Min at the front and Max at the end
void PushMinMax(vector<double>& _vec, double _num);

vector<double> GetCartesianCoordinates(vector<double> sphericalCoordinates);

int GetQuadrant(double _radians);

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

    CFG cfg = _graph->GetCfg(_cc[i]);
    center += cfg;
  }
  center /= _cc.size();
  return center;
};




template<class CfgType, class Environment>
CfgType 
SelectDirection(Environment* _env, CfgType dir){
  if(dir == CfgType()){
    dir.GetRandomCfg(_env);
  }else{
    CfgType r;
    r.GetRandomCfg(_env);
    dir.subtract(dir,r);
  }
  return dir;
}


template<class CfgType>
vector<double> 
GetSphericalCoordinates(CfgType& _cfg) {
  vector<double> coordinates(3);
  double rho = 0;   // = sqrt(x^2 + y^2 + z^2)
  double theta = 0; // = arctan(y/x) 
  double phi = 0;   // = arccos(z/rho)
  // Getting cartesian coordinates
  for (size_t j = 0; j < _cfg.PosDOF(); ++j) {
    coordinates[j] = _cfg[j];
    rho += pow(_cfg[j], 2.0);
  }
  // Coordinates = [X,Y,Z]
  rho = sqrt(rho); 
  theta = atan2(coordinates[1],coordinates[0]);
  phi = MAX_INT;
  if(_cfg.PosDOF() == 3)
    phi = acos(coordinates[2] / rho);
  
  // Lets make all angles positive for more accurate comparison between quadrants, since atan2 returns [-2/pi,2/pi]
  while(theta < 0) 
    theta += TWOPI; 
  // from cartesian to polar
  coordinates[0] = rho;
  coordinates[1] = theta;
  coordinates[2] = phi;
  
  return coordinates;
}


// Gets the point that is in between the candidate and the neighbor
template<class CfgType>
CfgType GetMiddlePoint(CfgType _regionCand, CfgType _neighbor, double _radius) {
  CfgType middlePoint;
  middlePoint = _regionCand + _neighbor; 
  middlePoint = middlePoint/2; 
  // Getting middle point across the circumference between candidate and neighbor
  vector<double> midPointCoordinates = GetSphericalCoordinates(middlePoint);
  midPointCoordinates[0] = _radius;
  midPointCoordinates = GetCartesianCoordinates(midPointCoordinates);

  middlePoint.SetData(midPointCoordinates);
  return middlePoint;

}


template<class CfgType>
vector<CfgType> 
GetMiddlePoints(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius) {
  
  vector<CfgType> middlePoints;
  // Getting middle point across the circumference between candidate and neighbor
  for (size_t i = 0; i < _neighbors.size(); ++i) 
    middlePoints.push_back(GetMiddlePoint(_regionCand, _neighbors[i], _radius));

  return middlePoints;

}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Random Node within a region
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
 * Returns a random node within a region characterized by a region candidate
 * and its neighbors
 *
 * */

template<class CfgType>
CfgType 
SelectDirection(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius) {

  SelectDirection(_regionCand, _neighbors, _radius, 0);
}


template<class CfgType>
CfgType 
SelectDirection(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius, double _overlap) {
  CfgType dir = CfgType();
  
  // Store all the angles to get the max and min
  vector<double> thetas;
  vector<double> phis;

  if (dir.PosDOF() > 0) {
    
    vector<double> candCoordinates = GetSphericalCoordinates(_regionCand);
    vector<CfgType> midPoints = GetMiddlePoints(_regionCand, _neighbors, _radius);
    // TODO DEBUG only one neighbor, split region into halves 
    if (midPoints.size() == 1) {
      CfgType point = -(midPoints[0]); 
      midPoints.push_back(point);
    }

    for (size_t i = 0; i < midPoints.size(); ++i) {
      // [0] = Rho, [1] = Theta, [2] = Phi
      vector<double> neighborCoordinates;
      if(_neighbors.size() == 1)
        neighborCoordinates = GetSphericalCoordinates(_neighbors[0]);
      else 
        neighborCoordinates = GetSphericalCoordinates(_neighbors[i]);
      
      vector<double> midPointCoordinates = GetSphericalCoordinates(midPoints[i]);
      vector<double> increments(3);
      
      // FIXME Increments not working, some angles are being shrinked
      // instead of enlarged
      increments[1] = (neighborCoordinates[1] - midPointCoordinates[1] ) * _overlap;
      midPointCoordinates[1] += increments[1];
      double increment = (neighborCoordinates[2] - midPointCoordinates[2]) * _overlap;
      midPointCoordinates[2] += increment;
      
      // We use this function to avoid pushing all angles and sorting them at the end
      PushMinMax(thetas, midPointCoordinates[1]);
      PushMinMax(phis, midPointCoordinates[2]);
    }
 
    // Is the range calculated correct? If cand theta is outside out [min,max] then fix the range to be [max-2PI, min] 
    while (thetas[0] > candCoordinates[1] ) {
      double temp = thetas[0];
      thetas[0] = thetas[1] - TWOPI;
      thetas[1] = temp;
    }
    while (thetas[1] < candCoordinates[1]) {
      double temp = thetas[1];
      thetas[1] = thetas[0] + TWOPI;
      thetas[0] = temp;
    }
    // Randomizing
    // Rho = radius
    vector<double> randCoordinates(3);
    randCoordinates[0] = _radius  * sqrt(DRand()); 
    randCoordinates[1] = (thetas[1] - thetas[0]) * DRand() + thetas[0];
    randCoordinates[2] = MAX_INT;
    if(_regionCand.PosDOF() == 3) 
      randCoordinates[2] = (phis[1] - phis[0]) * DRand() + phis[0];

    randCoordinates = GetCartesianCoordinates(randCoordinates);

    dir.SetData(randCoordinates);

    // Randomizing Rotational DOFs
    for (size_t i = dir.PosDOF(); i < dir.DOF(); i++)
      dir[i] = (2*DRand()-1.0);

    // TODO Fixed-Tree - I am not sure how to divide this space into regions.
  }else {
    for (size_t i=0; i < _regionCand.DOF(); i++) {
      vector<double> minMax;
      for (size_t j=0; j < _neighbors.size(); j++) {
        PushMinMax(minMax, _neighbors[j][i]);
      }
      dir[i] = (minMax.back() - minMax.front()) * DRand() + minMax.front();
    }
  }

  return dir;
}





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
  while(!collision && dm->Distance(env,_start,tick) <= _delta && ticker <= nTicks) {
    previous = tick;
    tick += incr; //Increment tick
    if(!(tick.InBoundary(env)) || !(vc->IsValid(tick, env, *stats, _cdInfo, &callee))){
      collision = true; //Found a collision; return previous tick, as it is collision-free
    }
    ++ticker;
  }
  if(previous != _start){ //Did we go anywhere?
    _newCfg = previous;//Last Cfg pushed back is the final tick allowed
    return true;     
  }
  //Didn't find a place to go :(
  else
    return false;
}

/*
 *  Expand for Lazy RRT where three different values can be returned: OUTOFBOUNDARY, INVALID, VALID
 * */

namespace ExpansionType {
  enum Expansion {
    IN_COLLISION,
    NO_COLLISION,
    OUT_OF_BOUNDARY,
    NO_EXPANSION,
    JUMPED,
  };
 
  string GetExpansionTypeString(Expansion expansion);

}
// TODO Cesar. Better naming
namespace NodeState {
  enum State {
    COLLISION,
    FREE,
  };
}

template<class MPTraits>
ExpansionType::Expansion
BlindRRTExpand(typename MPTraits::MPProblemType* _mp, 
    string _vc, string _dm, string _expansionType, 
    typename MPTraits::CfgType _start, 
    typename MPTraits::CfgType _dir, 
    vector< pair<typename MPTraits::CfgType, int> >& _newCfgs, // pair = Cfg , weight. weight is the distance from the Cfg to the start Cfg 
    double _delta, CDInfo& _cdInfo,
    double _posRes, double _oriRes){
  
  //Setup...primarily for collision checks that occur later on
  StatClass* stats = _mp->GetStatClass();
  string callee("RRTUtility::BlindRRTExpand");
  Environment* env = _mp->GetEnvironment();
  typename MPTraits::MPProblemType::DistanceMetricPointer dm = _mp->GetDistanceMetric(_dm);
  typename MPTraits::MPProblemType::ValidityCheckerPointer vc = _mp->GetValidityChecker(_vc);

  typedef typename MPTraits::CfgType CfgType;
  typename MPTraits::CfgType incr, tick = _start, previous = _start;
  // if any collision is encountered, collision becomes true
  // jump is to know if we should go directly to _delta
  bool collision=false, outOfBoundary=false, jump=false;
  // this tracks collision changes along the expansion
  bool prevCollision = !vc->IsValid(_start, env, *stats, _cdInfo, &callee); 
  int nTicks, ticker = 0;
  pair<CfgType, int> cfgWeight; 
  incr.FindIncrement(tick,_dir,&nTicks, _posRes, _oriRes);

  //Move out from start towards dir, bounded by number of ticks allowed at a given resolution.  Delta + obsDist are
  //given to the function, and are user defined.
  while(!outOfBoundary && dm->Distance(env,_start,tick) <= _delta && ticker <= nTicks) {
    previous = tick;
    tick += incr; //Increment tick
    ++ticker;
    
    if(!(tick.InBoundary(env)) ) {
      outOfBoundary = true; // Expansion is out of boundary, return previous tick
    }
    else if (!(vc->IsValid(tick, env, *stats, _cdInfo, &callee))) { // no need for further checking, get ray and return
      collision = true; //Found a collision, activate flag
      //previous.SetStat("Validity", NodeState::FREE);
      if(!prevCollision) {
        
        cfgWeight = make_pair(previous, ticker - 1); // previous is one tick behind
        _newCfgs.push_back(cfgWeight);
        
        // we track all nodes
        if(_expansionType == "All") {
          //tick.SetStat("Validity", NodeState::COLLISION);
          
          // TODO Cesar do we really want to add in collision nodes???
          cfgWeight = make_pair(tick, ticker);  
          _newCfgs.push_back(cfgWeight);
        
        } else if ("First") {  // we dont need to track every change, lets jump to delta
          jump = true;
          break;
        }
        
        prevCollision = true;
      }
    } else {
      //tick.SetStat("Validity", NodeState::FREE);
      
      if(prevCollision) {
        
        // we track all nodes
        if(_expansionType == "All") {
          //previous.SetStat("Validity", NodeState::COLLISION);
          
          // TODO Cesar do we really want to add in collision nodes???
          cfgWeight = make_pair(previous, ticker - 1); // previous is one tick behind 
          _newCfgs.push_back(cfgWeight);
        
        } 
         
        cfgWeight = make_pair(tick, ticker); 
        _newCfgs.push_back(cfgWeight);
        
        // if we place the previous two lines at the beginning to have a nested-if clause, the order cfgs are added is
        // wrong, so we have to check twice for the expansion label
        if (_expansionType == "First") {
          jump = true;
          break;
        }

        prevCollision = false; 
      }
    }
  }

  if (jump) {
    // we need to jump directly to delta, lets take the direction and scale to delta 
    CfgType origin;
    CfgType dir = _dir - _start;
    dm->ScaleCfg(env, _delta, origin, dir);
    previous = dir + _start;
    vc->IsValid(previous, env, *stats, _cdInfo, &callee); // lets validate previous and see where we expanded 
  } 
  if(previous != _start){ //Did we go anywhere?
    
    cfgWeight = make_pair(previous, ticker); 
    // TODO Cesar: previous might get added in the previous step
    if(previous != _newCfgs.back().first) 
      _newCfgs.push_back(cfgWeight);//Last Cfg pushed back is the final tick allowed
    
    if (collision) {    // encountered collision in the way
      return ExpansionType::IN_COLLISION;
    
    // TODO Cesar: implement laziness level with direct expansion
    //} else if (outOfBoundary) { 
        
    } else {            // No collision and not out of boundary
      return ExpansionType::NO_COLLISION;
    }

  }
  // Didn't find a place to go :(
  else
    return ExpansionType::NO_EXPANSION;     
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

