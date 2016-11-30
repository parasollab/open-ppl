#ifndef REACHABLE_BOUNDARY_H_
#define REACHABLE_BOUNDARY_H_

#include <algorithm>
#include <utility>

#include "Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Boundary of the reachable set
/// 
/// @TODO: find a more accurate method of representing the boundary of the
///        reachable set that is also efficient to test for collisions.
///
/// @note currently not used
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class ReachableBoundary {

  public:

    typedef typename MPTraits::CfgType CfgType;

    ///\name Construction
    ///@{

    ReachableBoundary();

    ReachableBoundary(const vector<CfgType>& _cfgs);

    ~ReachableBoundary() {}

    ///@}
    ///\name Boundary Operations
    ///@{

    const pair<double, double>* const GetBox() const {return m_bbx;}

    double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const;
    pair<double, double>& GetRange(size_t _i);
    pair<double, double> GetRange(size_t _i) const;

    bool InBoundary(const Vector3d& _p) const;

    ///@}
  
    const vector<CfgType>& GetReachableSet() {return m_reachableSet;}

  private:

    pair<double, double> m_bbx[3];  ///< Each pair is the min and max x, y, and z value
    vector<CfgType> m_reachableSet; ///< Reachable set of the boundary
    Vector3d m_center;              ///< center of the boundary
};

/*------------------------------- Construction -------------------------------*/

template<typename MPTraits>
ReachableBoundary<MPTraits>::
ReachableBoundary() {
  for(size_t i = 0; i < 3; ++i)
    m_bbx[i] = make_pair(-numeric_limits<double>::max(),
        numeric_limits<double>::max());
}


template<typename MPTraits>
ReachableBoundary<MPTraits>::
ReachableBoundary(const vector<CfgType>& _cfgs) {
  Vector3d max;
  Vector3d min;

  // Get the point of each cfg and find the min x, y, and z values
  for(const auto c : _cfgs) {
    auto point = c.GetPoint();

    for(int i = 0; i < 3; ++i) {
      max[i] = std::max(max[i], point[i]);
      min[i] = std::min(min[i], point[i]);
    }
  }

  // Create the bounding box
  for(int i = 0; i < 3; ++i) {
    m_bbx[i] = make_pair(min[i], max[i]);
  }
  
  // find the center of the box
  m_center = (Vector3d(m_bbx[0].first, m_bbx[1].first, m_bbx[2].first) +
      Vector3d(m_bbx[0].second, m_bbx[1].second, m_bbx[2].second))/2.;

  m_reachableSet = _cfgs;
}

/*------------------------------ Boundary Operations -------------------------*/

template<typename MPTraits>
double
ReachableBoundary<MPTraits>::
GetMaxDist(double _r1, double _r2) const {
  double maxdist = 0;
  for(size_t i = 0; i < 3; ++i) {
    double diff = m_bbx[i].second - m_bbx[i].first;
    maxdist += pow(diff, _r1);
  }

  return pow(maxdist, _r2);
}


template<typename MPTraits>
pair<double, double>&
ReachableBoundary<MPTraits>::
GetRange(size_t _i) {
  if(_i > 2)
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + ::to_string(_i) + "'.");

  return m_bbx[_i];
}


template<typename MPTraits>
pair<double, double>
ReachableBoundary<MPTraits>::
GetRange(size_t _i) const {
  if(_i > 2)
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + ::to_string(_i) + "'.");

  return m_bbx[_i];
}


template<typename MPTraits>
bool
ReachableBoundary<MPTraits>::
InBoundary(const Vector3d& _p) const {
  for(size_t i = 0; i < 3; ++i)
    if( _p[i] < m_bbx[i].first || _p[i] > m_bbx[i].second)
      return false;

  return true;
}

#endif
