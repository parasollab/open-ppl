#include "WorkspaceRegion.h"
#include "WorkspaceDecomposition.h"

#include "Environment/Boundary.h"

#include <algorithm>

/*--------------------------------- Modifiers --------------------------------*/

void
WorkspaceRegion::
AddPoint(const size_t _i) {
  m_points.emplace_back(_i);
}


void
WorkspaceRegion::
AddFacet(Facet&& _f) {
  m_facets.emplace_back(move(_f));
}


void
WorkspaceRegion::
AddBoundary(Boundary* _b) {
  m_boundary = shared_ptr<Boundary>(_b);
}

/*--------------------------------- Accessors --------------------------------*/

const Point3d&
WorkspaceRegion::
GetPoint(const size_t _i) const {
  return m_decomposition->GetPoint(m_points[_i]);
}


const vector<Point3d>
WorkspaceRegion::
GetPoints() const {
  vector<Point3d> out;
  for(const auto& index : m_points)
    out.push_back(m_decomposition->GetPoint(index));
  return out;
}

/*---------------------------------- Queries ---------------------------------*/

const bool
WorkspaceRegion::
HasPoint(const Point3d& _p) const {
  for(const auto& index : m_points)
    if(m_decomposition->GetPoint(index) == _p)
      return true;
  return false;
}


const Point3d
WorkspaceRegion::
FindCenter() const {
  Point3d center;
  for(const auto& index : m_points)
    center += m_decomposition->GetPoint(index);
  center /= m_points.size();
  return center;
}


const vector<Point3d>
WorkspaceRegion::
FindSharedPoints(const WorkspaceRegion& _wr) const {
  // Find shared indexes.
  vector<size_t> mine = m_points;
  vector<size_t> theirs = _wr.m_points;
  sort(mine.begin(), mine.end());
  sort(theirs.begin(), theirs.end());

  vector<size_t> shared;
  set_intersection(mine.begin(), mine.end(), theirs.begin(), theirs.end(),
      back_inserter(shared));

  // Get points from indexes.
  vector<Point3d> out;
  for(const auto& index : shared)
    out.push_back(m_decomposition->GetPoint(index));

  return out;
}


const vector<const WorkspaceRegion::Facet*>
WorkspaceRegion::
FindSharedFacets(const WorkspaceRegion& _wr) const {
  vector<const Facet*> out;
  for(const auto& myFacet : m_facets) {
    // Flip facet orientation before comparing as the normals will be reversed.
    for(auto theirFacet : _wr.m_facets) {
      // Use a copy to maintain const-ness.
      /// @TODO Eliminate this extraneous copy. Perhaps check if = reverse within
      ///       facet class.
      theirFacet.Reverse();
      if(myFacet != theirFacet)
        continue;
      out.push_back(&myFacet);
      break;
    }
  }
  return out;
}

/*----------------------------------------------------------------------------*/
