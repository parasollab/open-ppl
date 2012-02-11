#ifndef MPUTILS_H_
#define MPUTILS_H_

#include <map>
#include <string>
#include "CfgTypes.h"
#include "IOUtils.h"
//boost includes
#include <boost/function.hpp>
#include "boost/shared_ptr.hpp"
#include "boost/mpl/list.hpp"
#include "boost/mpl/sort.hpp"
#include "boost/type_traits/is_base_of.hpp"
#include "boost/mpl/begin.hpp"
#include "boost/mpl/end.hpp"
#include "boost/mpl/next_prior.hpp"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Constants
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifndef PI
#define PI              3.141592653589793
#endif
#define TWOPI           (PI*2.0)

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
  PQP,
#endif
  /// SOLID
#ifdef USE_SOLID
  SOLID,
#endif
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
// Simple Utilities (Angular Distance and Gaussian Distribution)
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/**Calculate the minimum DIRECTED angular distance 
 *between two angles normalized to 1.0 
 */
double DirectedAngularDistance(double _a, double _b);

double GaussianDistribution(double _mean, double _stdev);

/* This function calculated angle _c1_c2_c3, 
   as in the angle between the rays _c2, _c1 and _c2, _c3.
  */
template<typename CFG>
double AngleBetween(CFG _c1, CFG _c2, CFG _c3){
  CFG sub1, sub2;
  sub1.subtract(_c2, _c1);
  sub2.subtract(_c2, _c3);
  vector<double> sub1Data = sub1.GetData();
  vector<double> sub2Data = sub2.GetData();

  double v1Norm = 0, v2Norm = 0, v1DotV2 = 0;
  for(size_t i = 0; i<sub1Data.size(); i++){
    v1Norm += sub1Data[i] * sub1Data[i];
    v2Norm += sub2Data[i] * sub2Data[i];
    v1DotV2 += sub1Data[i] * sub2Data[i];
  }

  v1Norm = sqrt(v1Norm);
  v2Norm = sqrt(v2Norm);

  double angle = v1DotV2/(v1Norm*v2Norm);
  angle = acos(angle);
  return angle;
}
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

//defines basic method container class to derive from
//  (for classes like DistanceMetric, LocalPlanner, NeighborhoodFinder,NodeGenerator etc)
//
//derived class must specify the Method type and MethodTypeList it contains
//  e.g., NeighborhoodFinder: Method = NeighborhoodFinderMethod
//                            MethodTypeList = boost::mpl::list<BruteForce,ANN,...>
//  e.g., LocalPlanner: Method = LocalPlannerMethod
//                      MethodTypeList = boost::mpl::list<Straightline,RotateAtS,...>
//  e.g., GenerateMapNodes: Method = NodeGenerationMethod
//                      MethodTypeList = boost::mpl::list<UniformSampler,ObstacleBasedSampler,...>
//
//MethodTypeList stores the available method types in 1 place 
//  used in dynamic_cast generation for dispatching 

//base pmpl container class that holds a list available method objects
template <typename Method, typename TypeList>
class ContainerBase {
  public:
    typedef boost::shared_ptr<Method> MethodPointer;
    
    ContainerBase() {} 
    virtual ~ContainerBase() {}

    void AddMethod(string _name, boost::shared_ptr<Method> _m) {
      if(m_methods.find(_name) != m_methods.end())
        cerr << "\nWarning, method list already has a method pointer associated with \"" << _name << "\", not added\n";
      else
        m_methods[_name] = _m;
    }

    boost::shared_ptr<Method> GetMethod(const string _name) {return m_methods[_name];}

  protected:
    //MethodTypes: sorted list of method types class will use, puts most derived first
    typedef typename boost::mpl::sort<TypeList, boost::is_base_of<boost::mpl::_2, boost::mpl::_1> >::type MethodTypes;

    //MethodTypes_begin, MethodTypes_end: used for iteration over type lists
    typedef typename boost::mpl::begin<MethodTypes>::type MethodTypesBegin;
    typedef typename boost::mpl::end<MethodTypes>::type MethodTypesEnd;

    //shared_ptr used instead of regular pointer to handle memory 
    //  allocation/release automatically
    map<string, boost::shared_ptr<Method> > m_methods;
};

//example: Element = SamplerMethod, ElementTypeList is mpl list of available sampler methods

template<typename Element>
struct ElementFactory {
  boost::shared_ptr<Element> operator()(XMLNodeReader& _node, MPProblem* _problem) const {
    return boost::shared_ptr<Element>(new Element(_node, _problem));
  }
};


template<typename Element>
class ElementSet {
  protected:
    typedef boost::function<boost::shared_ptr<Element> (XMLNodeReader&, MPProblem*)> FactoryType;

    map<string, FactoryType> m_universe;
    map<string, boost::shared_ptr<Element> > m_elements;
    string m_default;

  public:
    typedef boost::shared_ptr<Element> MethodPointer;

    template <typename ElementTypeList>
      ElementSet(ElementTypeList const& _etl) : m_default("") {
        AddToUniverse(typename boost::mpl::begin<ElementTypeList>::type(), typename boost::mpl::end<ElementTypeList>::type());
      }

    bool AddElement(string const& _str, XMLNodeReader& _node, MPProblem* _problem) {
      if(m_universe.find(_str) != m_universe.end()) {
        boost::shared_ptr<Element> e = m_universe[_str](_node, _problem);
        return AddElement(e->GetLabel(), e);
      }
      return false;
    }

    bool AddElement(string const& _str, boost::shared_ptr<Element> _e) {
      if(m_elements.empty())
        m_default = _str;
      if(m_elements.find(_e->GetLabel()) == m_elements.end())
        m_elements[_e->GetLabel()] = _e;
      else
        cerr << "\nWarning, method list already has a pointer associated with \"" << _e->GetLabel() << "\", not added\n";
      return true;
    }

    boost::shared_ptr<Element> GetElement(string const& _name) {
      boost::shared_ptr<Element> element;
      if(_name == "") 
        element = m_elements[m_default];
      else
        element = m_elements[_name];
      if(element.get() == NULL) {
        cerr << "\n\tError, requesting element with name \"" << _name << "\" which does not exist in the element list.\n";
        cerr << "\t\tPossible choices are:";
        for(typename map<string, boost::shared_ptr<Element> >::const_iterator E = m_elements.begin(); E != m_elements.end(); ++E)
          if(E->second.get() != NULL)
            cerr << " \"" << E->first << "\"";
        cerr << "\n\texiting.\n";
        exit(-1);
      }
      return element;
    }

    typename map<string, boost::shared_ptr<Element> >::const_iterator ElementsBegin() const { return m_elements.begin(); }
    typename map<string, boost::shared_ptr<Element> >::const_iterator ElementsEnd() const { return m_elements.end(); }

  protected:
    template <typename Last>
      void AddToUniverse(Last, Last){}

    template <typename First, typename Last>
      void AddToUniverse(First, Last) {
        typename boost::mpl::deref<First>::type first;
        m_universe[first.GetName()] = ElementFactory<typename boost::mpl::deref<First>::type>();
        AddToUniverse(typename boost::mpl::next<First>::type(), Last());
      }
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

class MPProblem;
class MPBaseObject {
  public: 
    MPBaseObject():m_problem(NULL), m_label(""), m_name(""), m_debug(false){};
    MPBaseObject(MPProblem* _problem, string _label="", string _name="", bool _debug=false) :
      m_problem(_problem), m_label(_label), m_name(_name), m_debug(_debug) {};
    MPBaseObject(XMLNodeReader& _node, MPProblem* _problem, string _name="") : 
      m_problem(_problem), m_label(""), m_name(_name), m_debug(false) { 
        ParseXML(_node); 
      };

    virtual ~MPBaseObject() {}

    virtual void ParseXML(XMLNodeReader& _node) {
      m_label = _node.stringXMLParameter("Label", false, "", "Label Identifier");
      m_debug = _node.boolXMLParameter("debug", false, false, "Run-time debug on(true)/off(false)");
      m_recordKeep = _node.boolXMLParameter("recordKeep", false, false, "Keeping track of algorithmic statistics, on(true)/off(false)");
    };

    MPProblem* GetMPProblem() const {return m_problem;}
    void SetMPProblem(MPProblem* _m){m_problem = _m;}
    virtual void PrintOptions(ostream& _os) {};
    string GetLabel() const {return m_label;}
    string GetName()  const {return m_name;}
    string GetNameAndLabel() const {return m_name + "::" + m_label;}
    void SetName (string _s) {m_name  = _s;}

  private:
    MPProblem* m_problem;
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

    CFG cfg = (*(_graph->find_vertex(_cc[i]))).property();
    center.add(center, cfg);
  }
  center.divide(center, _cc.size());
  return center;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// MPProblem Access Helpers
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
template<class CFG> class SamplerMethod;
template<class CFG, class WEIGHT> class LocalPlannerMethod;

boost::shared_ptr<LocalPlannerMethod<CfgType, WeightType> > GetLPMethod(MPProblem* _mp, string _s);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// RRTExpand
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

bool RRTExpand(MPProblem* _mp, int _regionID, string _vc, string _dm, CfgType _start, CfgType _dir, CfgType& _newCfg, double _delta);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Medial Axis Utility
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//***********************************//
// Main Push To Medial Axis Function //
//***********************************//
bool PushToMedialAxis(MPProblem* _mp, Environment* _env, CfgType& _cfg, StatClass& _stats, string _vc, 
    string _dm, bool _cExact, int _clearance, bool _pExact, int _penetration, bool _useBBX, double _eps, 
    int _hLen, bool _debug, bool _positional); 
bool PushToMedialAxis(MPProblem* _mp, Environment* _env, shared_ptr<BoundingBox> _bb, CfgType& _cfg, StatClass& _stats, 
    string _vc, string _dm, bool _cExact, int _clearance, bool _pExact, int _penetration, 
    bool _useBBX, double _eps, int _hLen, bool _debug, bool _positional); 

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//
bool PushFromInsideObstacle(MPProblem* _mp, CfgType& _cfg, Environment* _env, StatClass& _stats,
    string _vc, string _dm, bool _pExact, int _penetration, bool _debug, bool _positional);	
bool PushFromInsideObstacle(MPProblem* _mp, CfgType& _cfg, Environment* _env, shared_ptr<BoundingBox> _bb, 
    StatClass& _stats, string _vc, string _dm, bool _pExact, int _penetration, bool _debug, bool _positional);

//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//
bool PushCfgToMedialAxis(MPProblem* _mp, CfgType& cfg, Environment* _env, StatClass& _stats,
    string _vc, string _dm, bool _cExact, int _clearance, bool _useBBX, double _eps, int _hLen, 
    bool _debug, bool _positional);
bool PushCfgToMedialAxis(MPProblem* _mp, CfgType& cfg, Environment* _env, shared_ptr<BoundingBox> _bb, 
    StatClass& _stats, string _vc, string _dm, bool _cExact, int _clearance, bool _useBBX, double _eps, 
    int _hLen, bool _debug, bool _positional);

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
bool CalculateCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string _vc, string _dm, bool _exact, int _clearance, int _penetration, bool _useBBX, bool _positional);
bool CalculateCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, 
    shared_ptr<BoundingBox> _bb, StatClass& _stats, CDInfo& _cdInfo, string _vc, string _dm, 
    bool _exact, int _clearance, int _penetration, bool _useBBX, bool _positional);

//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
bool GetExactCollisionInfo(MPProblem* _mp, CfgType& _cfg, Environment* _env, 
    StatClass& _stats, CDInfo& _cdInfo, string _vc, bool _useBBX);
bool GetExactCollisionInfo(MPProblem* _mp, CfgType& _cfg, Environment* _env, shared_ptr<BoundingBox> _bb, 
    StatClass& _stats, CDInfo& _cdInfo, string _vc, bool _useBBX);

//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best calididate.            //
//*********************************************************************//
bool GetApproxCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, StatClass& _stats,
    CDInfo& _cdInfo, string _vc, string _dm, int _clearance, int _penetration, bool _useBBX, bool _positional);	
bool GetApproxCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, 
    shared_ptr<BoundingBox> _bb, StatClass& _stats, CDInfo& _cdInfo, string _vc, string _dm, 
    int _clearance, int _penetration, bool _useBBX, bool _positional);

#endif

