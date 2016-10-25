#include "TetrahedralBoundary.h"

#include "Utilities/MPUtils.h"

#include <algorithm>

/*------------------------------- Construction -------------------------------*/

TetrahedralBoundary::
TetrahedralBoundary(const array<Point3d, 4>& _pts, const bool _check) :
    m_points(_pts) {
  if(_check)
    FixPoints();
  m_normals = ComputeNormals();
}


TetrahedralBoundary::
TetrahedralBoundary(const vector<Point3d>& _pts, const bool _check) {
  if(_pts.size() != 4)
    throw RunTimeException(WHERE, "Can't build tetrahedron with " +
        to_string(_pts.size()) + " points.");
  copy(_pts.begin(), _pts.end(), m_points.begin());
  if(_check)
    FixPoints();
  m_normals = ComputeNormals();
}

/*---------------------------- Property Accessors ----------------------------*/

double
TetrahedralBoundary::
GetMaxDist(double _r1, double _r2) const {
  auto edges = ComputeEdges();

  array<double, 6> edgeLengths;
  for(size_t i = 0; i < 6; ++i)
    edgeLengths[i] = edges[i].norm();

  return *max_element(edgeLengths.begin(), edgeLengths.end());
}


pair<double, double>
TetrahedralBoundary::
GetRange(size_t _i) const {
  double minV = numeric_limits<double>::max();
  double maxV = numeric_limits<double>::min();
  for(const auto& p : m_points) {
    minV = std::min(minV, p[_i]);
    maxV = std::max(maxV, p[_i]);
  }
  return make_pair(minV, maxV);
}

/*--------------------------------- Sampling ---------------------------------*/

Point3d
TetrahedralBoundary::
GetRandomPoint() const {
  // From:
  //   C. Rocchini and P. Cignoni, "Generating Random Points in a Tetrahedron,"
  //       Journal of Graphics Tools, 2001.

  // Pick a point in unit cube.
  double s = DRand();
  double t = DRand();
  double u = DRand();

  // Cut cube in half with plane s + t = 1.
  if(s + t > 1) {
    s = 1 - s;
    t = 1 - t;
  }

  // Cut cube with planes t + u = 1 and s + t + u = 1.
  if(s + t + u > 1) {
    if(t + u > 1) {
      double ttmp = 1 - u;
      double utmp = 1 - s - t;
      swap(t, ttmp);
      swap(u, utmp);
    }
    else {
      double stmp = 1 - t - u;
      double utmp = s + t + u - 1;
      swap(s, stmp);
      swap(u, utmp);
    }
  }

  // Determine random point in tetrahedron.
  return m_points[0] + s * (m_points[1] - m_points[0])
                     + t * (m_points[2] - m_points[0])
                     + u * (m_points[3] - m_points[0]);
}


bool
TetrahedralBoundary::
InBoundary(const Vector3d& _p) const {
  // Check dot-product with normals touching point 0.
  const Vector3d p = _p - m_points[0];
  for(size_t i = 0; i < 3; ++i)
    if(p * m_normals[i] > 0)
      return false;

  // Check dot-product with last normal.
  if((_p - m_points[1]) * m_normals[3] > 0)
    return false;

  return true;
}


double
TetrahedralBoundary::
GetClearance(const Vector3d& _p) const {
  throw RunTimeException(WHERE, "Not implemented");
  return 0;
}


int
TetrahedralBoundary::
GetSideID(const vector<double>& _p) const {
  throw RunTimeException(WHERE, "Not implemented");
  return -1;
}


Vector3d
TetrahedralBoundary::
GetClearancePoint(const Vector3d& _p) const {
  throw RunTimeException(WHERE, "Not implemented");
  return Vector3d();
}

/*--------------------------------- Modifiers --------------------------------*/

void
TetrahedralBoundary::
ApplyOffset(const Vector3d& _v) {
  m_center += _v;
  for(auto& p : m_points)
    p += _v;
}


void
TetrahedralBoundary::
ResetBoundary(vector<pair<double, double>>& _bbx, double _margin) {
  throw RunTimeException(WHERE, "Not implemented");
}

/*------------------------------------ I/O -----------------------------------*/

void
TetrahedralBoundary::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  throw RunTimeException(WHERE, "Not implemented");
}


void
TetrahedralBoundary::
Write(ostream& _os) const {
  throw RunTimeException(WHERE, "Not implemented");
}

/*------------------------------- CGAL Representation ------------------------*/

Boundary::CGALPolyhedron
TetrahedralBoundary::
CGAL() const {
  // Define builder object.
  struct builder : public CGAL::Modifier_base<CGALPolyhedron::HalfedgeDS> {

    const array<Point3d, 4>& m_points;

    /// The first three points must form an outward-facing facet.
    builder(const array<Point3d, 4>& _points) : m_points(_points) {}

    void operator()(CGALPolyhedron::HalfedgeDS& _h) {
      using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
      CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

      b.begin_surface(4, 4, 12);

      for(const auto& point : m_points)
        b.add_vertex(Point(point[0], point[1], point[2]));

      // Face 1
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(2);
      b.end_facet();

      // Face 2
      b.begin_facet();
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(3);
      b.add_vertex_to_facet(2);
      b.end_facet();

      // Face 3
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(3);
      b.add_vertex_to_facet(1);
      b.end_facet();

      // Face 4
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(2);
      b.add_vertex_to_facet(3);
      b.end_facet();

      b.end_surface();
    }
  };

  CGALPolyhedron cp;
  builder b(m_points);
  cp.delegate(b);

  if(!cp.is_valid())
    throw RunTimeException(WHERE, "TetrahedralBoundary:: Invalid CGAL polyhedron "
        "created!");
  return cp;
}

/*-------------------------------- Helpers -----------------------------------*/

void
TetrahedralBoundary::
FixPoints() {
  // Get the points.
  const Vector3d& base = m_points[0];
  const Vector3d& a    = m_points[1];
  const Vector3d& b    = m_points[2];
  const Vector3d& c    = m_points[3];

  // Find a,b,c relative to base.
  const Vector3d edgeA = a - base;
  const Vector3d edgeB = b - base;
  const Vector3d edgeC = c - base;

  // Points are OK if the norm of (base, a, b) faces away from edgeC.
  // Otherwise, swap points a and b to fix the ordering.
  const Vector3d norm  = edgeA % edgeB;
  const bool normFacesAway = norm * edgeC < 0;
  if(!normFacesAway)
    swap(m_points[1], m_points[2]);
}


array<Vector3d, 6>
TetrahedralBoundary::
ComputeEdges() const {
  array<Vector3d, 6> edges = {m_points[1] - m_points[0],
                              m_points[2] - m_points[0],
                              m_points[3] - m_points[0],
                              m_points[2] - m_points[1],
                              m_points[3] - m_points[1],
                              m_points[3] - m_points[2]};
  return edges;
}


array<Vector3d, 4>
TetrahedralBoundary::
ComputeNormals() const {
  auto edges = ComputeEdges();
  array<Vector3d, 4> normals = {(edges[0] % edges[1]).normalize(),
                                (edges[1] % edges[2]).normalize(),
                                (edges[2] % edges[0]).normalize(),
                                (edges[4] % edges[3]).normalize()};
  return normals;
}

/*----------------------------------------------------------------------------*/
