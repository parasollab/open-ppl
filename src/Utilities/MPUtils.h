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

#include "Vector.h"
#include "Point.h"
using namespace mathtool;
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
//ElementSet defines basic method container class to derive from
//  (for classes like DistanceMetric, LocalPlanner, NeighborhoodFinder, Sampler etc)
//
//derived class must specify the Method type and MethodTypeList
//  e.g., NeighborhoodFinder: Method = NeighborhoodFinderMethod
//                            MethodTypeList = boost::mpl::list<BruteForceNF,BandsNF,...>
//  e.g., LocalPlanner: Method = LocalPlannerMethod
//                      MethodTypeList = boost::mpl::list<Straightline,RotateAtS,...>
///////////////////////////////////////////////////////////////////////////////////////////

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

    void ParseXML(XMLNodeReader& _node, MPProblem* _problem){
      XMLNodeReader::childiterator citr;
      for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
        if (!AddElement(citr->getName(), *citr, _problem)){
          citr->warnUnknownNode();
          exit(-1);
        }
      }
    }

    bool AddElement(string const& _str, XMLNodeReader& _node, MPProblem* _problem) {
      if(m_universe.find(_str) != m_universe.end()) {
        boost::shared_ptr<Element> e = m_universe[_str](_node, _problem);
        return AddElement(e->GetLabel(), e);
      }
      return false;
    }

    bool AddElement(string const& _str, boost::shared_ptr<Element> _e) {
      _e->SetLabel(_str);
      if(m_elements.empty()) 
        m_default = _str;
      if(m_elements.find(_str) == m_elements.end()) 
        m_elements[_str] = _e;
      else 
        cerr << "\nWarning, method list already has a pointer associated with \"" << _str << "\", not added\n";
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

    void SetMPProblem(MPProblem* _mp){
      typedef typename map<string, boost::shared_ptr<Element> >::iterator MIT;
      for(MIT mit = m_elements.begin(); mit!=m_elements.end(); mit++){
        mit->second->SetMPProblem(_mp);
      }
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
    virtual void SetMPProblem(MPProblem* _m){m_problem = _m;}
    virtual void PrintOptions(ostream& _os) {};
    string GetLabel() const {return m_label;}
    void SetLabel(string _s) {m_label = _s;}
    string GetName()  const {return m_name;}
    void SetName (string _s) {m_name  = _s;}
    string GetNameAndLabel() const {return m_name + "::" + m_label;}
    bool GetDebug() const {return m_debug;}
    void SetDebug(bool _d) {m_debug = _d;}

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

bool RRTExpand(MPProblem* _mp, string _vc, string _dm, CfgType _start, CfgType _dir, CfgType& _newCfg, double _delta, int& _weight, CDInfo& cdInfo);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// ClearanceParams
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//used to encapsulate all the fields necessary for clearance and penetration calculations
class ClearanceParams : public MPBaseObject {
  public:
    string m_vcLabel;                //validity checker method label
    string m_dmLabel;                //distance metric method label
    bool m_exactClearance;           //use exact clearance calculations
    bool m_exactPenetration;         //use exact penetration calculations
    unsigned int  m_clearanceRays;   //number of rays used to approximate clearance
    unsigned int  m_penetrationRays; //number of rays used to approximate penetration
    bool m_useBBX;                   //use bounding box as obstacle
    bool m_positional;               //use only positional dofs

    ClearanceParams(MPProblem* mp = NULL,
      string _vcLabel = "", string _dmLabel = "",
      bool _exactClearance = false, bool _exactPenetration = false,
      int _clearanceRays = 10, int _penetrationRays = 10,
      bool _useBBX = true, bool _positional = true, bool _debug = false):
      MPBaseObject(), 
      m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
      m_exactClearance(_exactClearance), m_exactPenetration(_exactPenetration),
      m_clearanceRays(_clearanceRays), m_penetrationRays(_penetrationRays),
      m_useBBX(_useBBX), m_positional(_positional){
        SetMPProblem(mp);
        m_debug = _debug;
      }

    //see ParseXML for usage of the callee argument 
    ClearanceParams(XMLNodeReader& _node, MPProblem* _mp, string callee = "", bool _debug = false):
      MPBaseObject(_mp, "ClearanceParams,Clearance Parameters"){//don't use node
      ParseXML(_node, callee);
      m_debug = _debug;
    }

    //The callee argument so far is used for:
    //MARRT to provide its own "exact" XML parameter for future usage of this struct in MARRT.
    //ApproxSpheres as penetration is not necessary for it
    void ParseXML(XMLNodeReader& _node, string callee = ""){
      bool parseCType = callee.compare("MARRTStrategy") != 0;//is clearanceType applicable for the callee?
      bool parsePType = parseCType && (callee.compare("ApproxSpheres") != 0);//is penetrationType applicable?
      m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
      m_dmLabel = _node.stringXMLParameter("dmMethod", true, "", "Distance metric");
      m_clearanceRays = _node.numberXMLParameter("clearanceRays", false, 10, 1, 1000, "Number of Clearance Rays");
      m_penetrationRays = _node.numberXMLParameter("penetrationRays", false, 10, 1, 1000, "Number of Penetration Rays");
      if(parseCType){    
        string clearanceType = _node.stringXMLParameter("clearanceType", true, "", "Clearance Computation (exact or approx)");
        m_exactClearance = clearanceType.compare("exact")==0;
      }
      if(parsePType){
        string penetrationType = _node.stringXMLParameter("penetrationType",true, "", "Penetration Computation (exact or approx)");
        m_exactPenetration = penetrationType.compare("exact")==0;
      }
      m_useBBX = _node.boolXMLParameter("useBBX", false, true, "Use the Bounding Box as an Obstacle");
      m_positional = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");
    }

    friend ostream& operator<<(ostream& _out, const ClearanceParams& _c){
      _c.PrintOptions(_out);
      return _out;
    }

    virtual void PrintOptions(ostream& _os) const{
      _os << "\tvcLabel = " << m_vcLabel << endl;
      _os << "\tdmLabel = " << m_dmLabel << endl;
      _os << "\tuseBBX = " << m_useBBX << endl;
      _os << "\tclearance = ";
      _os << ((m_exactClearance) ? "exact, " : "approx, ");
      _os << m_clearanceRays << " rays\n";
      _os << "\tpenetration = ";
      _os << ((m_exactPenetration) ? "exact, " : "approx, ");
      _os << m_penetrationRays << " rays\n";
    }
};

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
bool PushToMedialAxis(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, double _epsilon, int _historyLen);
bool PushToMedialAxis(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, double _epsilon, int _historyLen, shared_ptr<Boundary> _bb);

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//
bool PushFromInsideObstacle(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams);
bool PushFromInsideObstacle(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, shared_ptr<Boundary> _bb);

//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//
bool PushCfgToMedialAxis(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, double _epsilon, int _historyLength);
bool PushCfgToMedialAxis(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, double _epsilon, int _historyLength, shared_ptr<Boundary> _bb);

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
bool CalculateCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
  StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams); 
bool CalculateCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
  StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams, shared_ptr<Boundary> _bb);

//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
bool GetExactCollisionInfo(CfgType& _cfg, StatClass& _stats, CDInfo& _cdInfo,
  const ClearanceParams& _cParams);
bool GetExactCollisionInfo(CfgType& _cfg, StatClass& _stats, CDInfo& _cdInfo,
  const ClearanceParams& _cParams, shared_ptr<Boundary> _bb);

//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best calididate.            //
//*********************************************************************//
bool GetApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
  StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams);
bool GetApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
  StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams, shared_ptr<Boundary> _bb);

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

#endif

