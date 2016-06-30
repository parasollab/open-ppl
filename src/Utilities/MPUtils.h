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


///\name MPUtils
///@{

/*-------------------------------- Constants ---------------------------------*/

#define MAX_INT  numeric_limits<int>::max()
#define MAX_DBL  numeric_limits<double>::max()

////////////////////////////////////////////////////////////////////////////////
/// @brief Variable resolution epsilon for doubles and float. This number is
///        based upon the resolution of the smaller value between _t1 and _t2.
template<typename T>
const T
Epsilon(const T& _t1, const T& _t2) {
  return abs(min(_t1, _t2)) * 10 * numeric_limits<T>::epsilon();
}

/*------------------------- Random Number Generation -------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief Return non-negative double-prevision floating-point values uniformly
///        distributed over the interval [0.0, 1.0)
double DRand();

////////////////////////////////////////////////////////////////////////////////
/// @brief Return non-negative long integers uniformly distributed over the
///        interval [0, 2**31)
long LRand();

////////////////////////////////////////////////////////////////////////////////
/// @brief Return signed long integers uniformly distributed over the interval
///        [-2**31, 2**31)
long MRand();

////////////////////////////////////////////////////////////////////////////////
/// @brief Return normally(gaussian) distributed random numbers via the
///        Marsaglia polar method.
/// @param[in] _reset If true, clear internal cache and return 0.
double GRand(bool _reset = false);

////////////////////////////////////////////////////////////////////////////////
/// @brief Same as GRand, but one can specify the mean and stdev of the
///        distribution.
double GaussianDistribution(double _mean, double _stdev);

////////////////////////////////////////////////////////////////////////////////
/// @brief Use seedval as the seed.
long SRand(long _seed = 0x1234ABCD);

////////////////////////////////////////////////////////////////////////////////
/// @brief Set the random seed.
///
/// Use methodName, nextNodeIndex, and base to generate a deterministic seed,
/// then call seed48().
/// @param[in] _methodName A string to use in generating the seed. Just use base
///                        value if this is "NONE".
/// @param[in] _nextNodeIndex Another input for seed generation.
/// @param[in] _base  Base seed value, saved after first use until reset.
/// @param[in] _reset Use new value of _base.
long SRand(string _methodName, int _nextNodeIndex, long _base = 0x1234ABCD,
    bool _reset = false);

/*------------------------------ Geometry Utils ------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief Normalize a value into the range [-1,1).
double Normalize(double _a);

////////////////////////////////////////////////////////////////////////////////
/// @brief Calculate the minimum DIRECTED angular distan between two angles
///        normalized to 1.0.
double DirectedAngularDistance(double _a, double _b);

////////////////////////////////////////////////////////////////////////////////
/// @brief Determine height of triangle defined by three points.
/// @param _a The first triangle vertex.
/// @param _b The second triangle vertex.
/// @param _c The third triangle vertex.
/// @return The height of the triangle.
double TriangleHeight(const Point3d& _a, const Point3d& _b, const Point3d& _c);

////////////////////////////////////////////////////////////////////////////////
/// @brief Check if a 2d point lies inside the triangle defined by three other
///        points.
/// @param[in]  _a The first triangle vertex.
/// @param[in]  _b The second triangle vertex.
/// @param[in]  _c The third triangle vertex.
/// @param[in]  _p The query point.
/// @return True if _p is inside triangle _a_b_c, or false otherwise.
bool PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d & _p);

////////////////////////////////////////////////////////////////////////////////
/// @brief Check if a 2d point lies inside the triangle defined by three other
///        points.
/// @param[in]  _a The first triangle vertex.
/// @param[in]  _b The second triangle vertex.
/// @param[in]  _c The third triangle vertex.
/// @param[in]  _p The query point.
/// @param[out] _u The barycentric coordinate factor for _b.
/// @param[out] _v The barycentric coordinate factor for _c.
/// @return True if _p is inside triangle _a_b_c, or false otherwise.
bool PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d& _p, double& _u, double& _v);

////////////////////////////////////////////////////////////////////////////////
/// @brief Given triange defined by _a, _b, _c, return the point inside triangle
///        defined by barycentric coordinates _u, _v.
Point3d GetPtFromBarycentricCoords(const Point3d& _a, const Point3d& _b,
    const Point3d& _c, double _u, double _v);

////////////////////////////////////////////////////////////////////////////////
/// @brief Normalize an angle to standard range.
/// @param[in] _theta The angle to normalize.
/// @return Normalized representation of _theta in the range -PI to PI.
double NormalizeTheta(double _theta);

/*---------------------------- Comparators -----------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief Compare the second of a pair
/// @tparam T Type 1 of pair
/// @tparam U Type 2 of pair
////////////////////////////////////////////////////////////////////////////////
template <typename T, typename U>
struct CompareSecond {

  bool operator()(const pair<T, U>& _a, const pair<T, U>& _b) const {
    return _a.second < _b.second;
  }

};

////////////////////////////////////////////////////////////////////////////////
/// @brief Compare the second of a pair reversed
/// @tparam T Type 1 of pair
/// @tparam U Type 2 of pair
////////////////////////////////////////////////////////////////////////////////
template <typename T, typename U>
struct CompareSecondReverse {

  bool operator()(const pair<T, U>& _a, const pair<T, U>& _b) const {
    return _a.second > _b.second;
  }

};

////////////////////////////////////////////////////////////////////////////////
/// @brief Find the closer of two input configurations to an initial reference
///        cfg, assuming that the inputs are given as a pair<Cfg, Something>.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits, class P>
struct DistanceCompareFirst : public binary_function<P, P, bool> {

  typedef typename MPTraits::MPProblemType::DistanceMetricPointer
      DistanceMetricPointer;

  Environment* m_env;
  DistanceMetricPointer m_dm;
  const typename P::first_type& m_cfg;

  DistanceCompareFirst(Environment* _e, DistanceMetricPointer _d,
      const typename P::first_type& _c) : m_env(_e), m_dm(_d), m_cfg(_c) {}

  bool operator()(const P& _p1, const P& _p2) const {
    return m_dm->Distance(m_cfg, _p1.first) < m_dm->Distance(m_cfg, _p2.first);
  }

};

/*----------------------------- Compose Functions ----------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief TODO
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
/// @brief TODO
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
/// @brief TODO
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
/// @brief Applies a unary operator to an input iterator and returns its
///        negation.
////////////////////////////////////////////////////////////////////////////////
template <typename InputIterator, typename UnaryOperator>
struct ComposeNegate {

  bool operator()(InputIterator _it, UnaryOperator _op) {
    return !_op(*_it);
  }

};

/*------------------------------- Method Set ---------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief Creates new method instances from an XML node.
template<typename MPTraits, typename Method>
struct MethodFactory {

  shared_ptr<Method> operator()(typename MPTraits::MPProblemType* _problem,
      XMLNode& _node) const {
    return shared_ptr<Method>(new Method(_problem, _node));
  }

};

////////////////////////////////////////////////////////////////////////////////
/// @brief Defines basic method container class to hold methods (for classes like
///        DistanceMetricMethod, LocalPlannerMethod, etc).
///
/// MethodTypeList must be defined within templated class of MPTraits
///   e.g., Method = NeighborhoodFinderMethod
///         MethodTypeList = boost::mpl::list<BruteForceNF, BandsNF, ...>
///   e.g., Method = LocalPlannerMethod
///         MethodTypeList = boost::mpl::list<Straightline, RotateAtS, ...>
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits, typename Method>
class MethodSet {

  public:

    typedef shared_ptr<Method> MethodPointer;

    template<typename MethodTypeList>
    MethodSet(const MethodTypeList& _etl, const string& _name) : m_default(""),
        m_name(_name) {
      AddToUniverse(typename boost::mpl::begin<MethodTypeList>::type(),
          typename boost::mpl::end<MethodTypeList>::type());
    }

    void ParseXML(typename MPTraits::MPProblemType* _problem, XMLNode& _node) {
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
          cerr << "\nWarning, method list already has a pointer associated with "
               << "\"" << _label << "\", not added\n";
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
        _os << ++count << ") \"" << elem.first << "\" (" << elem.second->m_name
            << ")" << endl;
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

    typedef function<MethodPointer(typename MPTraits::MPProblemType*, XMLNode&)>
        FactoryType;

    template <typename Last>
    void AddToUniverse(Last, Last){}

    template <typename First, typename Last>
    void AddToUniverse(First, Last) {
      typename boost::mpl::deref<First>::type first;
      m_universe[first.m_name] = MethodFactory<MPTraits,
          typename boost::mpl::deref<First>::type>();
      AddToUniverse(typename boost::mpl::next<First>::type(), Last());
    }

    string m_default, m_name;
    map<string, FactoryType> m_universe;
    map<string, MethodPointer> m_elements;

};

/*------------------------------ Cfg Utilities -------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief Determine whether two configurations are within a resolution unit of
///        each other.
/// @param[in] _cfg1 The first configuration.
/// @param[in] _cfg2 The second configuration.
/// @return A bool indicating whether the two cfgs are separated by no more than
///         one resolution unit.
template<class CfgType, class Environment>
bool
IsWithinResolution(const CfgType& _cfg1, const CfgType& _cfg2,
    Environment* _env) {
  CfgType diff = _cfg1 - _cfg2;
  return diff->PositionMagnitude() <= _env->GetPositionRes()
      && diff->OrientationMagnitude() <= _env->GetOrientationRes();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Given a reference configuration and a line segment in C-space, find
///        the closest point on the line segment to the reference configuration.
/// @param[in] _ref The reference configuration.
/// @param[in] _p1  The first endpoint of the line segment.
/// @param[in] _p2  The second endpoint of the line segment.
/// @return The closest point to _ref on the segment from _p1 to _p2.
template<class CfgType>
CfgType
ClosestPtOnLineSegment(const CfgType& _ref, const CfgType& _p1,
    const CfgType& _p2) {
  CfgType b = _p2 - _p1;
  CfgType c = _ref - _p1;

  double bDotC = 0;
  double bSquared = 0;

  for(auto itb = b.GetData().begin(), itc = c.GetData().begin();
      itb != b.GetData().end(); ++itb, ++itc) {
    bDotC += (*itb) * (*itc);
    bSquared += (*itb) * (*itb);
  }

  if(bDotC <= 0)
    return _p1;
  else if(bDotC >= bSquared)
    return _p2;
  else
    return b * (bDotC / bSquared) + _p1;
}

/*------------------------------ Centroid Utils ------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief Compute the C-space centroid of the configurations in a specific
///        connected component of the roadmap.
/// @param[in] _graph A pointer to the roadmap graph.
/// @param[in] _cc The connected component.
/// @return The centroid configuration of _cc.
template<template<typename, typename> class Roadmap, class CfgType, class Weight>
CfgType
GetCentroid(Roadmap<CfgType, Weight>* _graph,
    vector<typename Roadmap<CfgType, Weight>::VID>& _cc){
  CfgType center;
  for(size_t i = 0; i < _cc.size(); i++) {
    CfgType cfg = _graph->GetVertex(_cc[i]);
    center += cfg;
  }
  center /= _cc.size();
  return center;
};

#ifndef _PARALLEL
////////////////////////////////////////////////////////////////////////////////
/// @brief Compute the graph of all connected component centroids. The output
///        graph will contain only the centroids and no edges.
/// @param[in] _graph The roadmap graph to analyze.
/// @param[out] _centroidGraph The output centroid graph. It should be
///                            initialized prior to this call.
template<template<typename, typename> class Roadmap, class CfgType, class Weight>
void
ComputeCCCentroidGraph(Roadmap<CfgType, Weight>* _graph,
    Roadmap<CfgType, Weight>* _centroidGraph) {
  typedef typename Roadmap<CfgType, Weight>::VID VID;
  stapl::sequential::vector_property_map<Roadmap<CfgType, Weight>, size_t> cmap;
  vector<pair<size_t, VID>> allCCs;
  vector<VID> cc;
  Roadmap<CfgType, Weight> centroids;
  get_cc_stats(*_graph, cmap, allCCs);

  for(size_t i = 0; i < allCCs.size(); i++) {
    get_cc(*_graph, cmap, allCCs[i].second, cc);
    CfgType centroid = GetCentroid(_graph, cc);
    centroid.SetStat("ccVID", allCCs[i].second);
    _centroidGraph->AddVertex(centroid);
  }

};
#endif

/*----------------------------- Other Random Stuff ---------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @brief Functor for adding the second types of two pairs.
////////////////////////////////////////////////////////////////////////////////
template <class P>
struct PlusSecond : public binary_function<typename P::second_type, P,
    typename P::second_type> {

  typename P::second_type operator()(const typename P::second_type& p1,
      const P& p2) const {
    return plus<typename P::second_type>()(p1, p2.second);
  }

};

////////////////////////////////////////////////////////////////////////////////
/// @brief Used for discarding collision data for regular sampling/connecting
///        classes. All operations are no-ops.
////////////////////////////////////////////////////////////////////////////////
struct NullOutputIterator : std::iterator<std::output_iterator_tag,
    NullOutputIterator> {

  template<typename T>
  void operator=(const T&) { }

  NullOutputIterator& operator++() {return *this;}
  NullOutputIterator operator++(int) {return *this;}
  NullOutputIterator& operator*() {return *this;}

};

/*----------------------------------------------------------------------------*/

///@}

#endif
