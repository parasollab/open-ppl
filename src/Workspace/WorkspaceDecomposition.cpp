#include "WorkspaceDecomposition.h"
#include "Environment/TetrahedralBoundary.h"

/*----------------------------- Accessors ------------------------------------*/

const WorkspacePortal&
WorkspaceDecomposition::
GetPortal(const size_t _i1, const size_t _i2) const {
  const_vertex_iterator vert;
  const_adj_edge_iterator edge;
  RegionGraph::edge_descriptor ed(_i1, _i2);
  find_edge(ed, vert, edge);
  return edge->property();
}

/*----------------------------- Modifiers ------------------------------------*/

void
WorkspaceDecomposition::
AddPoint(const Point3d& _p) throw(PMPLException) {
  AssertMutable();
  m_points.push_back(_p);
}


void
WorkspaceDecomposition::
AddTetrahedralRegion(const int _pts[4]) throw(PMPLException) {
  AssertMutable();
  WorkspaceRegion wr(this);

  // Add points to the region.
  for(size_t i = 0; i < 4; ++i)
    wr.AddPoint(_pts[i]);

  // Add facets to the region. Create a facet for each combination of points,
  // and ensure that their normals face away from the center point.
  Point3d center = wr.FindCenter();
  for(size_t i = 0; i < 4; ++i) {
    WorkspaceRegion::Facet f(_pts[i], _pts[(i + 1) % 4], _pts[(i + 2) % 4],
        m_points);
    f.ComputeNormal();
    if(f.PointIsAbove(center))
      f.Reverse();
    wr.AddFacet(move(f));
  }

  // Create a tetrahedral boundary object for the region.
  /// @TODO The boundary currently double-stores the points. We would like to
  /// have a non-mutable boundary that holds only references to the points.
  wr.AddBoundary(new TetrahedralBoundary(wr.GetPoints()));

  // Add region to the graph.
  add_vertex(move(wr));
}


void
WorkspaceDecomposition::
AddPortal(const size_t _s, const size_t _t) throw(PMPLException) {
  AssertMutable();
  add_edge(_s, _t, WorkspacePortal(this, _s, _t));
}


void
WorkspaceDecomposition::
Finalize() {
  ComputeDualGraph();
  m_finalized = true;
}

/*------------------------------ Helpers -------------------------------------*/

void
WorkspaceDecomposition::
AssertMutable() const throw(PMPLException) {
  if(m_finalized)
    throw PMPLException("WorkspaceDecomposition error", WHERE, "Can't modify "
        "the decomposition after it has been finalized.");
}


void
WorkspaceDecomposition::
ComputeDualGraph() {
  // Make vertices.
  for(const_vertex_iterator vit = begin(); vit != end(); ++vit)
    m_dual.add_vertex(vit->property().FindCenter());

  // Make edges.
  for(const_edge_iterator iter = edges_begin(); iter != edges_end(); ++iter) {
    size_t s = iter->source();
    size_t t = iter->target();
    double length = (m_dual.find_vertex(t)->property() -
                     m_dual.find_vertex(s)->property()).norm();
    m_dual.add_edge(s, t, length);
  }
}

/*----------------------------------------------------------------------------*/
