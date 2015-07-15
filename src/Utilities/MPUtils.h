#ifndef MPUTILS_H_
#define MPUTILS_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
using std::shared_ptr;

#ifndef _PARALLEL
#include <containers/sequential/graph/algorithms/connected_components.h>
#endif

#include <boost/mpl/list.hpp>
#include <boost/mpl/next_prior.hpp>

#include "Vector.h"
using namespace mathtool;

#include "IOUtils.h"

class Environment;

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////

#define MAX_INT  numeric_limits<int>::max()
#define MAX_DBL  numeric_limits<double>::max()

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// Variable resolution epsilon for doubles and float. This number is based upon
/// the resolution of the smaller value between _t1 and _t2.
template<typename T>
const T
Epsilon(const T& _t1, const T& _t2) {
  return abs(min(_t1, _t2))*10*numeric_limits<T>::epsilon();
}

///////////////////////////////////////////////////////////////////////////////
// Random Number Generation
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// Return non-negative double-prevision floating-point values uniformly
/// distributed over the interval [0.0, 1.0)
double DRand();

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// return non-negative long integers uniformly distributed over the interval [0, 2**31)
long LRand();

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// return signed long integers uniformly distributed over the interval [-2**31, 2**31)
long MRand();

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// normally(gaussian) distributed random number generator.
/// when reset is 1, it reset the internal static variable and return 0.0
double GRand(bool _reset = false);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// Same as GRand, but one can specify the mean and stdev of the distribution
double GaussianDistribution(double _mean, double _stdev);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// use seedval as the seed
long SRand(long _seed = 0x1234ABCD);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// "baseSeed" is a static variable in this function
/// we use baseSeed, methodName and nextNodeIndex to generate a deterministic seed,
/// then call seed48()
/// when reset is 1, baseSeed will be reset
long SRand(string _methodName, int _nextNodeIndex, long _base = 0x1234ABCD, bool _reset = false);

///////////////////////////////////////////////////////////////////////////////
// Simple Utilities (Angular Distance and Compare Second)
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// Normalize a value into the range [-1,1)
double Normalize(double _a);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// Calculate the minimum DIRECTED angular distan between two angles normalized
/// to 1.0.
double DirectedAngularDistance(double _a, double _b);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief Compare the second of a pair
/// @tparam T Type 1 of pair
/// @tparam U Type 2 of pair
////////////////////////////////////////////////////////////////////////////////
template <typename T, typename U>
class CompareSecond {
 public:
  bool operator()(const pair<T, U>& _a, const pair<T, U>& _b) const {
    return _a.second < _b.second;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief Compare the second of a pair reversed
/// @tparam T Type 1 of pair
/// @tparam U Type 2 of pair
////////////////////////////////////////////////////////////////////////////////
template <typename T, typename U>
class CompareSecondReverse {
 public:
  bool operator()(const pair<T, U>& _a, const pair<T, U>& _b) const {
    return _a.second > _b.second;
  }
};

///////////////////////////////////////////////////////////////////////////////
// Compose Functions
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename InputIterator, typename UnaryOperator>
struct ComposeNegate {
  bool operator()(InputIterator _it, UnaryOperator _op) {
    return !_op(*_it);
  }
};

///////////////////////////////////////////////////////////////////////////////
// Containers
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//MethodSet defines basic method container class to hold methods
//  (for classes like DistanceMetricMethod, LocalPlannerMethod, etc)
//
//MethodTypeList must be defined within templated class of MPTraits
//  e.g., NeighborhoodFinder: Method = NeighborhoodFinderMethod
//                            MethodTypeList = boost::mpl::list<BruteForceNF,BandsNF,...>
//  e.g., LocalPlanner: Method = LocalPlannerMethod
//                      MethodTypeList = boost::mpl::list<Straightline,RotateAtS,...>
///////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits, typename Method>
struct MethodFactory {
  shared_ptr<Method> operator()(typename MPTraits::MPProblemType* _problem, XMLNode& _node) const {
    return shared_ptr<Method>(new Method(_problem, _node));
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits, typename Method>
class MethodSet {
  public:
    typedef shared_ptr<Method> MethodPointer;

    template<typename MethodTypeList>
      MethodSet(const MethodTypeList& _etl, const string& _name) : m_default(""), m_name(_name) {
        AddToUniverse(typename boost::mpl::begin<MethodTypeList>::type(), typename boost::mpl::end<MethodTypeList>::type());
      }

    void ParseXML(typename MPTraits::MPProblemType* _problem, XMLNode& _node){
      for(auto& child : _node)
        AddMethod(_problem, child);
    }

    void AddMethod(typename MPTraits::MPProblemType* _problem, XMLNode& _node) {
      if(m_universe.find(_node.Name()) != m_universe.end()) {
        MethodPointer e = m_universe[_node.Name()](_problem, _node);
        AddMethod(e, e->m_label);
      }
    }

    void AddMethod(MethodPointer _e, const string& _label) {
      if(m_universe.find(_e->m_name) != m_universe.end()) {
        _e->SetLabel(_label);
        if(m_elements.empty())
          m_default = _label;
        if(m_elements.find(_label) == m_elements.end())
          m_elements[_label] = _e;
        else
          cerr << "\nWarning, method list already has a pointer associated with \"" << _label << "\", not added\n";
      }
      else
        throw ParseException(WHERE, "Method '" + _e->m_name +
            "' is not contained within the motion planning universe.");
    }

    MethodPointer GetMethod(const string& _label) {
      MethodPointer element;
      if(_label == "")
        element = m_elements[m_default];
      else
        element = m_elements[_label];
      if(element.get() == NULL) {
        ostringstream oss;
        oss << "Element '" << _label
          << "' does not exist in " << m_name << ". Choices are: ";
        for(auto& elem : m_elements)
          if(elem.second.get())
            oss << " '" << elem.first << "',";
        string err = oss.str();
        err.pop_back();
        throw RunTimeException(WHERE, err);
      }
      return element;
    }

    void SetMPProblem(typename MPTraits::MPProblemType* _problem) {
      for(auto&  elem : m_elements)
        elem.second->SetMPProblem(_problem);
    }

    void Print(ostream& _os) const {
      size_t count = 0;
      _os << endl << m_name << " has these methods available::" << endl << endl;
      for(auto& elem : m_elements) {
        _os << ++count << ") \"" << elem.first << "\" (" << elem.second->m_name << ")" << endl;
        elem.second->Print(_os);
        _os << endl;
      }
      _os << endl;
    }

    typedef typename map<string, MethodPointer>::iterator MIT;
    typedef typename map<string, MethodPointer>::const_iterator CMIT;
    MIT begin() {return m_elements.begin();}
    MIT end() {return m_elements.end();}
    CMIT begin() const {return m_elements.begin();}
    CMIT end() const {return m_elements.end();}

  protected:
    typedef function<MethodPointer(typename MPTraits::MPProblemType*, XMLNode&)> FactoryType;

    template <typename Last>
      void AddToUniverse(Last, Last){}

    template <typename First, typename Last>
      void AddToUniverse(First, Last) {
        typename boost::mpl::deref<First>::type first;
        m_universe[first.m_name] = MethodFactory<MPTraits, typename boost::mpl::deref<First>::type>();
        AddToUniverse(typename boost::mpl::next<First>::type(), Last());
      }

    string m_default, m_name;
    map<string, FactoryType> m_universe;
    map<string, MethodPointer> m_elements;
};

///////////////////////////////////////////////////////////////////////////////
// Cfg Utilities
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
template<class CfgType, class Environment>
bool
IsWithinResolution(const CfgType& _cfg1, const CfgType& _cfg2, Environment* _env) {
  CfgType diff = _cfg1 - _cfg2;
  return diff->PositionMagnitude() <= _env->GetPositionRes()
    && diff->OrientationMagnitude() <= _env->GetOrientationRes();
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// pt1 & pt2 are two endpts of a line segment
/// find the closest point to the current cfg on that line segment
/// it could be one of the two endpoints of course
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
// Centroid Utils
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
template<template<class CFG, class WEIGHT> class RDMP, class CFG, class WEIGHT>
CFG
GetCentroid(RDMP<CFG, WEIGHT>* _graph, vector<typename RDMP<CFG, WEIGHT>::VID>& _cc){
  CFG center;
  for(size_t i = 0; i < _cc.size(); i++) {

    CFG cfg = _graph->GetVertex(_cc[i]);
    center += cfg;
  }
  center /= _cc.size();
  return center;
};

#ifndef _PARALLEL
////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
template<template<class CFG, class WEIGHT> class RDMP, class CFG, class WEIGHT>
void
ComputeCCCentroidGraph(RDMP<CFG, WEIGHT>* _graph, RDMP<CFG, WEIGHT>* _centroidGraph) {
  typedef typename RDMP<CFG, WEIGHT>::VID VID;
  stapl::sequential::vector_property_map<RDMP<CFG, WEIGHT>, size_t> cmap;
  vector<pair<size_t, VID> > allCCs;
  vector<VID> cc;
  RDMP<CFG, WEIGHT> centroids;
  get_cc_stats(*_graph, cmap, allCCs);

  for(size_t i = 0; i < allCCs.size(); i++) {
    get_cc(*_graph, cmap, allCCs[i].second, cc);
    CFG centroid = GetCentroid(_graph, cc);
    centroid.SetStat("ccVID", allCCs[i].second);
    _centroidGraph->AddVertex(centroid);
  }

};
#endif

///////////////////////////////////////////////////////////////////////////////
// Geometry Utils
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// PtInTriangle: determine if point _P is in triange defined by (_A,_B,_C)
bool PtInTriangle(const Point2d& _A, const Point2d& _B, const Point2d& _C, const Point2d & _P);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// CHECKS IF 2D POINT P IS IN TRIANGLE ABC. RETURNS 1 IF IN, 0 IF OUT
///   uses barycentric coordinated to compute this and return the uv-coords
///   for potential usage later
bool PtInTriangle(const Point2d& _A, const Point2d& _B, const Point2d& _C, const Point2d & _P,
    double& _u, double& _v);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// GetPtFromBarycentricCoords: given triange defined by _A,_B,_C, return the
/// point inside triangle defined by barycentric coords. _u,_v
Point3d GetPtFromBarycentricCoords(const Point3d& _A, const Point3d& _B, const Point3d& _C, double _u, double _v);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// NormalizeTheta: given a value, lock it into the range -PI to PI
double NormalizeTheta(double _theta);

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits, class P>
struct DistanceCompareFirst : public binary_function<P, P, bool> {
  typedef typename MPTraits::MPProblemType::DistanceMetricPointer DistanceMetricPointer;

  Environment* m_env;
  DistanceMetricPointer m_dm;
  const typename P::first_type& m_cfg;

  DistanceCompareFirst(Environment* _e, DistanceMetricPointer _d, const typename P::first_type& _c) :
    m_env(_e), m_dm(_d), m_cfg(_c) {}
  ~DistanceCompareFirst() {}

  bool operator()(const P& _p1, const P& _p2) const {
    return m_dm->Distance(m_cfg, _p1.first) < m_dm->Distance(m_cfg, _p2.first);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <class P>
struct PlusSecond : public binary_function<typename P::second_type,
					    P,
					    typename P::second_type> {
  typename P::second_type operator()(const typename P::second_type& p1, const P& p2) const {
    return plus<typename P::second_type>()(p1, p2.second);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief Used for discarding collision data for regular sampling/connecting
///        classes
////////////////////////////////////////////////////////////////////////////////
struct NullOutputIterator : std::iterator<std::output_iterator_tag, NullOutputIterator> {
  //no-op assignment
  template<typename T>
    void operator=(const T&) { }

  NullOutputIterator& operator++() {return *this;}
  NullOutputIterator operator++(int) {return *this;}
  NullOutputIterator& operator*() {return *this;}
};

#endif

