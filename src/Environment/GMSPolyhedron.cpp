#include "GMSPolyhedron.h"

#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <set>

#include "MovieBYULoader.h"
#include "ModelFactory.h"
#include "ObjLoader.h"

#include "Utilities/IOUtils.h"
#include "Utilities/MPUtils.h"

using namespace std;


/*-------------------------------- Equality ----------------------------------*/

bool
GMSPolyhedron::
operator==(const GMSPolyhedron& _p) const {
  return m_area == _p.m_area
      && m_maxRadius == _p.m_maxRadius
      && m_minRadius == _p.m_minRadius
      && m_polygonList == _p.m_polygonList
      && m_vertexList == _p.m_vertexList;
}


bool
GMSPolyhedron::
operator!=(const GMSPolyhedron& _p) const {
  return !(*this == _p);
}

/*------------------------------ I/O Functions -------------------------------*/

Vector3d
GMSPolyhedron::
Read(string _fileName, COMAdjust _comAdjust) {
  if(!FileExists(_fileName))
    throw ParseException(WHERE, "Geometry file '" + _fileName + "' not found.");

  // Get file extension.
  string ext;
  size_t pos = _fileName.rfind(".");
  if(pos != string::npos)
    ext = _fileName.substr(pos + 1);

  if(ext != "g" && ext != "obj")
    throw ParseException(WHERE, _fileName + " has an unrecognized format '" +
        ext + "'. Recognized formats are BYU(.g) and OBJ(.obj).");

  unique_ptr<IModel> imodel(CreateModelLoader(_fileName, false));
  if(!imodel)
    throw ParseException(WHERE, "Cannot read model '" + _fileName + "'.");

  Vector3d com = LoadFromIModel(imodel.get(), _comAdjust);

  return com;
}


Vector3d
GMSPolyhedron::
LoadFromIModel(IModel* _imodel, COMAdjust _comAdjust) {
  // Add vertices to the polyhedron and compute center of mass.
  Vector3d com;
  for(const auto& v : _imodel->GetVertices()) {
    m_vertexList.push_back(v);
    com += v;
  }
  com /= m_vertexList.size();

  // Repeat for CGAL points.
  CGALPoint ccom;
  for(const auto& c : _imodel->GetCGALVertices()) {
    m_cgalPoints.push_back(c);
    ccom[0] += c[0];
    ccom[1] += c[1];
    ccom[2] += c[2];
  }
  ccom[0] /= double(m_cgalPoints.size());
  ccom[1] /= double(m_cgalPoints.size());
  ccom[2] /= double(m_cgalPoints.size());

  // Apply COMAdjust to vertices and find radii.
  m_maxRadius = 0;
  m_minRadius = numeric_limits<double>::infinity();
  for(size_t i = 0; i < m_vertexList.size(); ++i) {
    auto& v = m_vertexList[i];
    auto& c = m_cgalPoints[i];
    switch(_comAdjust) {
      case COMAdjust::COM:     // Move COM to xyz origin.
        v -= com;
        c[0] -= ccom[0];
        c[1] -= ccom[1];
        c[2] -= ccom[2];
        break;
      case COMAdjust::Surface: // Move COM to xz origin.
        v[0] -= com[0];
        v[2] -= com[2];
        c[0] -= ccom[0];
        c[2] -= ccom[2];
        break;
      case COMAdjust::None:    // Do nothing.
      default:
        break;
    }

    double dist = v.norm();
    m_maxRadius = max(m_maxRadius, dist);
    m_minRadius = min(m_minRadius, dist);
  }

  // Add triangles to the polyhedron.
  for(auto& t : _imodel->GetTriP())
    m_polygonList.emplace_back(t[0], t[1], t[2], m_vertexList);

  // Compute the surface area.
  m_area = 0;
  for(const auto& p : m_polygonList)
    m_area += p.GetArea();

  return com;
}


void
GMSPolyhedron::
WriteBYU(ostream& _os) const {
  size_t numTri = m_polygonList.size();
  _os << "1 " << m_vertexList.size() << " " << numTri << " " << numTri * 3 << endl
      << "1 " << numTri << endl;
  for(const auto& v : m_vertexList)
    _os << v << endl;
  for(const auto& p : m_polygonList) {
    for(auto i = p.begin(); (i + 1) != p.end(); ++i)
      _os << *i + 1 << " ";
    _os << "-" << (*--p.end()) + 1 << endl;
  }
}

/*-------------------------------- Equality ----------------------------------*/

Point3d
GMSPolyhedron::
GetRandPtOnSurface() const {
  // Chose a polygon on the surface. Half of the time, choose by fraction of
  // total surface area. Otherwise, choose randomly.
  size_t index;
  if(DRand() < 0.5) {
    // Choose by fractional area.
    do {
      index = 0;
      double prob = DRand(), cummProb = 0;
      for(const auto& tri : m_polygonList) {
	cummProb += tri.GetArea() / m_area;
	if(prob <= cummProb)
	  break;
        ++index;
      }
    } while(!m_polygonList[index].IsTriangle());
  }
  else {
    // Choose randomly.
    do {
      index = LRand() % m_polygonList.size();
    } while(!m_polygonList[index].IsTriangle());
  }

  // Now that a polygon index has been selected, get a random point on it using
  // barycentric coordinates.
  double u = DRand(), v = DRand();
  if(u + v >= 1) {
    u = 1 - u;
    v = 1 - v;
  }
  const GMSPolygon& tri = m_polygonList[index];
  const Vector3d& p0 = tri.GetPoint(0);
  const Vector3d& p1 = tri.GetPoint(1);
  const Vector3d& p2 = tri.GetPoint(2);
  const Vector3d AB = p1 - p0;
  const Vector3d AC = p2 - p0;
  return p0 + (u * AB) + (v * AC);
}


bool
GMSPolyhedron::
IsOnSurface(const Point2d& _p) const {
  for(const auto& poly : m_polygonList) {
    if(!poly.IsTriangle())
      continue;
    const Vector3d& v0 = poly.GetPoint(0);
    const Vector3d& v1 = poly.GetPoint(1);
    const Vector3d& v2 = poly.GetPoint(2);
    const Point2d p0(v0[0], v0[2]);
    const Point2d p1(v1[0], v1[2]);
    const Point2d p2(v2[0], v2[2]);
    if(PtInTriangle(p0, p1, p2, _p))
      return true;
  }
  return false;
}


double
GMSPolyhedron::
HeightAtPt(const Point2d& _p, bool& _valid) const {
  for(const auto& poly : m_polygonList) {
    if(!poly.IsTriangle())
      continue;
    const Vector3d& v0 = poly.GetPoint(0);
    const Vector3d& v1 = poly.GetPoint(1);
    const Vector3d& v2 = poly.GetPoint(2);
    Point2d p0(v0[0], v0[2]);
    Point2d p1(v1[0], v1[2]);
    Point2d p2(v2[0], v2[2]);
    double u, v;
    if(PtInTriangle(p0, p1, p2, _p, u, v)) {
      _valid = true;
      Point3d pt3d = GetPtFromBarycentricCoords(v0, v1, v2, u, v);
      return pt3d[1];
    }
  }
  // After checking all triangles, inconsistency found in iscollision check
  _valid = false;
  return -19999.0;
}


////////////////////////////////////////////////////////////////////////////////
/// \brief Given a reference point and a line segment, find the point on the
///        segment nearest to the reference and return the squared distance
///        between them.
/// \param[in] _p The reference point.
/// \param[in] _s The start of the line segment.
/// \param[in] _e The end of the line segment.
/// \param[in/out] _closest The point on the segment that is closest to _p.
/// \return The squared distance between _p and _closest.
inline double
distanceSqrFromSegment(const Point3d& _p, const Point3d& _s, const Point3d& _e,
    Point3d& _closest) {
  const Vector3d n = _e - _s;
  const Vector3d q = _p - _s;
  double len = q.comp(n);
  if(len <= 0)
    _closest = _s;
  else if(len >= n.norm())
    _closest = _e;
  else
    _closest = q.proj(n);
  return (_closest - _p).normsqr();
}


double
GMSPolyhedron::
GetClearance(const Point3d& _p, Point3d& _closest) {
  double closestDist = 1e10;

  // Go through all boundary edges and find the closest point on any edge to _p.
  for(const auto& bl : GetBoundaryLines()) {
    const Vector3d& v1 =  m_vertexList[bl.first];
    const Vector3d& v2 =  m_vertexList[bl.second];

    // Find the point on this boundary line closest to p.
    Point3d c;
    double dist = distanceSqrFromSegment(_p, v1, v2, c);
    if(dist < closestDist) {
      closestDist = dist;
      _closest = c;
    }
  }

  return sqrt(closestDist);
}


double
GMSPolyhedron::
PushToMedialAxis(Point3d& _p) {
  Point3d orig = _p;

  // Compute clearance and closest point.
  Point3d closest;
  double clearance = GetClearance(_p, closest);

  // Compute the direction in the xz plane from closest to _p.
  Vector3d dir = (_p - closest).normalize();
  dir[1] = 0;
  dir = dir.normalize() * .5;

  Point3d newClosest = closest;
  size_t iteration = 0;
  do {
    // Push point along dir.
    _p += dir;

    // Project pushed point to xz plane.
    Point2d proj(_p[0], _p[2]);

    // Find height (y-coordinate) of polyhedron at this xz point.
    bool valid = true;
    double newH = HeightAtPt(proj, valid);

    // If polyhedron doesn't extend to this xz point, return original point.
    if(!valid) {
      _p = orig;
      break;
    }

    // Otherwise, set pushed point's y-coordinate to the polyhedron height.
    _p[1] = newH;
    clearance = GetClearance(_p, newClosest);
  } while((newClosest - closest).normsqr() < 0.1 && iteration < 1000000);

  return clearance;
}


GMSPolyhedron::CGALPolyhedron
GMSPolyhedron::
CGAL() const {
  // Define builder object.
  struct builder : public CGAL::Modifier_base<CGALPolyhedron::HalfedgeDS> {

    const GMSPolyhedron& m_poly;

    builder(const GMSPolyhedron& _p) : m_poly(_p) {}

    void operator()(CGALPolyhedron::HalfedgeDS& _h) {
      using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
      CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

      size_t num_vertices = m_poly.m_vertexList.size();
      size_t num_facets   = m_poly.m_polygonList.size();
      size_t num_edges    = num_facets * 3;

      b.begin_surface(num_vertices, num_facets, num_edges);

      // Add vertices.
      for(const auto& p : m_poly.m_cgalPoints)
        b.add_vertex(p);

      // Add facets.
      for(const auto& facet : m_poly.m_polygonList) {
        b.begin_facet();
        for(const auto& index : facet)
          b.add_vertex_to_facet(index);
        b.end_facet();
      }

      b.end_surface();
    }
  };

  // Delegate builder object.
  CGALPolyhedron cp;
  builder b(*this);
  cp.delegate(b);
  if(!cp.is_valid())
    throw RunTimeException(WHERE, "GMSPolyhedron:: Invalid CGAL polyhedron "
        "created!");
  return cp;
}

/*-------------------------- Initialization Helpers --------------------------*/

void
GMSPolyhedron::
BuildBoundary2D() {
  m_boundaryLines.clear();
  m_boundaryBuilt = false;
  m_force2DBoundary = true;
  BuildBoundary();
}


void
GMSPolyhedron::
BuildBoundary() {
  if(m_boundaryBuilt) return;            // only allow this to be attempted once
  if(m_boundaryLines.size() > 0) return; // this has been done

  m_boundaryBuilt = true;

  // Create function for determining if a polygon is near the XZ surface plane.
  static auto NearXZPlane = [&](const GMSPolygon& _p) -> bool {
    const double tolerance = 0.3; // Tolerance for considering points near-plane.
    return fabs(_p.GetPoint(0)[1]) <= tolerance
        && fabs(_p.GetPoint(1)[1]) <= tolerance
        && fabs(_p.GetPoint(2)[1]) <= tolerance;
  };

  // Get all of the edges from every triangle.
  multiset<pair<int, int>> lines;
  for(const auto& tri : m_polygonList) {
    if(m_force2DBoundary && !NearXZPlane(tri))
      continue;
    for(unsigned short i = 0; i < 3; ++i) {
      // Always put the lower vertex index first to make finding duplicates
      // easier.
      const int& a = tri[i];
      const int& b = tri[(i + 1) % 3];
      lines.emplace(min(a, b), max(a, b));
    }
  }

  // Store the edges that occurred exactly once as the boundary lines.
  for(auto iter = lines.begin(), next = ++lines.begin();
      iter != lines.end() && next != lines.end(); ++iter, ++next) {
    if(*iter != *next)
      continue;
    do {
      ++next;
    } while(next != lines.end() && *iter == *next);
    iter = lines.erase(iter, next);
    --iter;
  }

  m_boundaryLines.reserve(lines.size());
  copy(lines.begin(), lines.end(), back_inserter(m_boundaryLines));
}

/*----------------------------------------------------------------------------*/
