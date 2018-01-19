#ifndef MP_UTILS_H_
#define MP_UTILS_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
using std::shared_ptr;

#ifndef _PARALLEL
#include <containers/sequential/graph/algorithms/connected_components.h>
#endif

#include "Vector.h"
using namespace mathtool;

#include "IOUtils.h"

class Environment;
class Cfg;
class Robot;

///@name MPUtils
///@{

/*-------------------------------- Constants ---------------------------------*/

#define MAX_INT  numeric_limits<int>::max()
#define MAX_DBL  numeric_limits<double>::max()

/// Variable resolution epsilon for doubles and float. This number is based upon
/// the resolution of the lower magnitude value between _t1 and _t2.
template<typename T>
const T
Epsilon(const T& _t1, const T& _t2) {
  static constexpr T tenEpsilon = T(10) * std::numeric_limits<T>::epsilon();
  return std::min(std::abs(_t1), std::abs(_t2)) * tenEpsilon;
}

/*------------------------- Random Number Generation -------------------------*/

/// Return non-negative double-prevision floating-point values uniformly
/// distributed over the interval [0.0, 1.0).
double DRand();

/// Return non-negative long integers uniformly distributed over the interval
/// [0, 2**31).
long LRand();

/// Return signed long integers uniformly distributed over the interval
/// [-2**31, 2**31).
long MRand();

/// Return normally(gaussian) distributed random numbers via the Marsaglia polar
/// method.
/// @param[in] _reset If true, clear internal cache and return 0.
double GRand(bool _reset = false);

/// Same as GRand, but one can specify the mean and stdev of the distribution.
double GaussianDistribution(double _mean, double _stdev);

/// Use seedval as the seed.
void SRand(const unsigned long _seed);

/*------------------------------ Geometry Utils ------------------------------*/

/// Normalize a value into the range [-1,1).
double Normalize(double _a);

/// Calculate the minimum DIRECTED angular distan between two angles normalized
/// to 1.0.
double DirectedAngularDistance(double _a, double _b);

/// Determine height of triangle defined by three points.
/// @param _a The first triangle vertex.
/// @param _b The second triangle vertex.
/// @param _c The third triangle vertex.
/// @return The height of the triangle.
double TriangleHeight(const Point3d& _a, const Point3d& _b, const Point3d& _c);

/// Check if a 2d point lies inside the triangle defined by three other points.
/// @param[in]  _a The first triangle vertex.
/// @param[in]  _b The second triangle vertex.
/// @param[in]  _c The third triangle vertex.
/// @param[in]  _p The query point.
/// @return True if _p is inside triangle _a_b_c, or false otherwise.
bool PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d & _p);

/// Check if a 2d point lies inside the triangle defined by three other points.
/// @param[in]  _a The first triangle vertex.
/// @param[in]  _b The second triangle vertex.
/// @param[in]  _c The third triangle vertex.
/// @param[in]  _p The query point.
/// @param[out] _u The barycentric coordinate factor for _b.
/// @param[out] _v The barycentric coordinate factor for _c.
/// @return True if _p is inside triangle _a_b_c, or false otherwise.
bool PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d& _p, double& _u, double& _v);

/// Given triange defined by _a, _b, _c, return the point inside triangle defined
/// by barycentric coordinates _u, _v.
Point3d GetPtFromBarycentricCoords(const Point3d& _a, const Point3d& _b,
    const Point3d& _c, double _u, double _v);

/// Normalize an angle to standard range.
/// @param[in] _theta The angle to normalize.
/// @return Normalized representation of _theta in the range -PI to PI.
double NormalizeTheta(double _theta);

/*---------------------------- Comparators -----------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// Compare the second of a pair.
/// @tparam T Type 1 of pair
/// @tparam U Type 2 of pair
////////////////////////////////////////////////////////////////////////////////
template <typename T, typename U>
struct CompareSecond {

  const bool operator()(const pair<T, U>& _a, const pair<T, U>& _b) const {
    return _a.second < _b.second;
  }

};

////////////////////////////////////////////////////////////////////////////////
/// Compare the second of a pair reversed.
/// @tparam T Type 1 of pair
/// @tparam U Type 2 of pair
////////////////////////////////////////////////////////////////////////////////
template <typename T, typename U>
struct CompareSecondReverse {

  const bool operator()(const pair<T, U>& _a, const pair<T, U>& _b) const {
    return _a.second > _b.second;
  }

};

/*----------------------------- Compose Functions ----------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// @TODO
////////////////////////////////////////////////////////////////////////////////
template <typename InputIterator, typename BinaryOperator, typename UnaryOperator>
struct Compose {

  const bool operator()(InputIterator _first, InputIterator _last,
      BinaryOperator _binaryOp, UnaryOperator _op) const {
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
/// @TODO
////////////////////////////////////////////////////////////////////////////////
template <typename InputIterator, typename UnaryOperator>
struct Compose<InputIterator, logical_and<bool>, UnaryOperator> {

  const bool operator()(InputIterator _first, InputIterator _last,
      logical_and<bool> _binaryOp, UnaryOperator _op) const {
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
/// @TODO
////////////////////////////////////////////////////////////////////////////////
template <typename InputIterator, typename UnaryOperator>
struct Compose<InputIterator, logical_or<bool>, UnaryOperator> {

  const bool operator()(InputIterator _first, InputIterator _last,
      logical_or<bool> _binaryOp, UnaryOperator _op) const {
    if (_first == _last)
      return false;
    else {
      bool result = _op(*_first++);
      if(result == true)
        return result;
      while(_first != _last) {
        result = _binaryOp(result, _op(*_first++));
        if(result == true)
          return result;
      }
      return result;
    }
  }

};

////////////////////////////////////////////////////////////////////////////////
/// Applies a unary operator to an input iterator and returns its negation.
////////////////////////////////////////////////////////////////////////////////
template <typename InputIterator, typename UnaryOperator>
struct ComposeNegate {

  const bool operator()(InputIterator _it, UnaryOperator _op) const {
    return !_op(*_it);
  }

};

/*------------------------------ Cfg Utilities -------------------------------*/

/// Determine whether two configurations are within a resolution unit of each
/// other.
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


/// Given a reference configuration and a line segment in C-space, find the
/// closest point on the line segment to the reference configuration.
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

/// Compute the C-space centroid of the configurations in a specific connected
/// component of the roadmap.
/// @param[in] _graph A pointer to the roadmap graph.
/// @param[in] _cc The connected component.
/// @return The centroid configuration of _cc.
template<template<typename, typename> class RoadmapGraph, class CfgType,
    class Weight>
CfgType
GetCentroid(RoadmapGraph<CfgType, Weight>* _graph,
    vector<typename RoadmapGraph<CfgType, Weight>::VID>& _cc) {
  CfgType center(_graph->begin()->property().GetRobot());
  for(size_t i = 0; i < _cc.size(); i++) {
    CfgType cfg = _graph->GetVertex(_cc[i]);
    center += cfg;
  }
  center /= _cc.size();
  return center;
};

#ifndef _PARALLEL
/// Compute the graph of all connected component centroids. The output graph will
/// contain only the centroids and no edges.
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
/// Used for discarding collision data for regular sampling/connecting classes.
/// All operations are no-ops.
////////////////////////////////////////////////////////////////////////////////
struct NullOutputIterator : std::iterator<std::output_iterator_tag,
    NullOutputIterator> {

  template<typename T>
  void operator=(const T&) { }

  NullOutputIterator& operator++() {return *this;}
  NullOutputIterator operator++(int) {return *this;}
  NullOutputIterator& operator*() {return *this;}

};


////////////////////////////////////////////////////////////////////////////////
/// Search a random-access container using binary search. Needed for std::vector
/// because std::find cannot know whether the elements are sorted without doing
/// a linear scan.
////////////////////////////////////////////////////////////////////////////////
template <class RandomIterator, class T, class Compare = std::less<T>>
RandomIterator
BinarySearch(RandomIterator _begin, RandomIterator _end, const T& _value,
    Compare _comparator = Compare()) {
  auto start = _begin;
  auto end = _end;

  while(start != end) {
    const size_t mid = (size_t) (end - start) / 2;
    RandomIterator middle = start;
    std::advance(middle, mid);

    if(_value == *middle)
      return middle;
    if(_comparator(_value, *middle))
      end = middle;
    else
      start = middle;
  }
  return _end;
}

/// Loads a configuration path from a file for a dynamic obstacle.
std::vector<Cfg> LoadPath(const std::string &filename);

/*----------------------------------------------------------------------------*/

///@}

#endif
