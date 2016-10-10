#include "GMSPolygon.h"

#include <algorithm>

#include "Utilities/MPUtils.h"

using namespace std;

/*-------------------------------- Construction ------------------------------*/

GMSPolygon::
GMSPolygon(int _v1, int _v2, int _v3, const vector<Point3d>& _pts) :
    m_vertexList{_v1, _v2, _v3}, m_pointList(&_pts) {
  // Rotate the vertex list so that the lowest-index point is always first. This
  // is required to ensure that polygons constructed from points {1, 2, 3} and
  // {2, 3, 1}, which represent the same structure, compare as equal.
  auto iter = min_element(m_vertexList.begin(), m_vertexList.end());
  rotate(m_vertexList.begin(), iter, m_vertexList.end());
  ComputeNormal();
}

/*--------------------------------- Accessors --------------------------------*/

const Point3d&
GMSPolygon::
GetPoint(const size_t _i) const {
  return (*m_pointList)[m_vertexList[_i]];
}

/*--------------------------------- Modifiers --------------------------------*/

void
GMSPolygon::
Reverse() {
  reverse(++m_vertexList.begin(), m_vertexList.end());
  m_normal *= -1;
}


void
GMSPolygon::
ComputeNormal() {
  const Vector3d v1 = GetPoint(1) - GetPoint(0);
  const Vector3d v2 = GetPoint(2) - GetPoint(0);
  m_normal = v1 % v2;
  const double norm = m_normal.norm();
  m_normal /= norm;
  m_area = .5 * norm;
}

/*-------------------------------- Equality ----------------------------------*/

const bool
GMSPolygon::
operator==(const GMSPolygon& _p) const {
  return m_area == _p.m_area
      && m_pointList == _p.m_pointList
      && m_normal == _p.m_normal
      && m_vertexList == _p.m_vertexList;
}


const bool
GMSPolygon::
operator!=(const GMSPolygon& _p) const {
  return !(*this == _p);
}

/*------------------------------- Queries ------------------------------------*/

const bool
GMSPolygon::
IsTriangle() const {
  return m_vertexList.size() == 3 &&
      m_vertexList[0] != m_vertexList[1] &&
      m_vertexList[1] != m_vertexList[2] &&
      m_vertexList[0] != m_vertexList[2];
}


const Point3d
GMSPolygon::
FindCenter() const {
  Point3d center;
  for(size_t i = 0; i < m_vertexList.size(); ++i)
    center += GetPoint(i);
  return center /= m_vertexList.size();
}


const bool
GMSPolygon::
PointIsAbove(const Point3d& _p) const {
  return (_p - GetPoint(0)) * m_normal > 0;
}


const int
GMSPolygon::
CommonVertex(const GMSPolygon& _p) const {
  for(size_t i = 0; i < m_vertexList.size(); ++i) {
    for(size_t j = 0; j < m_vertexList.size(); ++j) {
      if(m_vertexList[i] == _p.m_vertexList[j]) {
        return m_vertexList[i];
      }
    }
  }
  return -1;
}


const pair<int, int>
GMSPolygon::
CommonEdge(const GMSPolygon& _p) const {
  pair<int, int> edgeID(-1, -1);
  for(size_t i = 0; i < m_vertexList.size(); ++i) {
    for(size_t j = 0; j < m_vertexList.size(); ++j) {
      if(m_vertexList[i] == _p.m_vertexList[j]) {
        if(edgeID.first == -1)
          edgeID.first = m_vertexList[i];
        else
          edgeID.second = m_vertexList[i];
      }
    }
  }
  return edgeID;
}

/*----------------------------------------------------------------------------*/
