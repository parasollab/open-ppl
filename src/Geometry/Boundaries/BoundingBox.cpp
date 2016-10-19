#include "Geometry/Boundaries/BoundingBox.h"

#include "Utilities/MPUtils.h"

/*------------------------------- Construction -------------------------------*/

BoundingBox::
BoundingBox() {
  for(size_t i = 0; i < 3; ++i)
    m_bbx[i] = make_pair(-numeric_limits<double>::max(),
        numeric_limits<double>::max());
}

BoundingBox::
BoundingBox(pair<double, double> _x, pair<double, double> _y,
    pair<double, double> _z) {
  m_bbx[0] = _x;
  m_bbx[1] = _y;
  m_bbx[2] = _z;
  m_center = (Vector3d(_x.first, _y.first, _z.first) +
      Vector3d(_x.second, _y.second, _z.second))/2.;
}

/*---------------------------- Property Accessors ----------------------------*/

double
BoundingBox::
GetMaxDist(double _r1, double _r2) const {
  double maxdist = 0;
  for(size_t i = 0; i < 3; ++i) {
    double diff = m_bbx[i].second - m_bbx[i].first;
    maxdist += pow(diff, _r1);
  }
  return pow(maxdist, _r2);
}


pair<double, double>
BoundingBox::
GetRange(size_t _i) const {
  if(_i > 2)
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + ::to_string(_i) + "'.");
  return m_bbx[_i];
}

/*-------------------------------- Sampling ----------------------------------*/

Point3d
BoundingBox::
GetRandomPoint() const {
  Point3d p;
  for(size_t i = 0; i < 3; ++i)
    p[i] = m_bbx[i].first + (m_bbx[i].second - m_bbx[i].first)*DRand();
  return p;
}

/*----------------------------- Containment Testing --------------------------*/

const bool
BoundingBox::
InBoundary(const Vector3d& _p) const {
  for(size_t i = 0; i < 3; ++i)
    if( _p[i] < m_bbx[i].first || _p[i] > m_bbx[i].second)
      return false;
  return true;
}

/*------------------------------ Clearance Testing ---------------------------*/

double
BoundingBox::
GetClearance(const Vector3d& _p) const {
  double minClearance = numeric_limits<double>::max();
  for(size_t i = 0; i < 3; ++i) {
    double clearance = min((_p[i] - m_bbx[i].first ), (m_bbx[i].second - _p[i]));
    if (clearance < minClearance || i == 0)
      minClearance = clearance;
  }
  return minClearance;
}


int
BoundingBox::
GetSideID(const vector<double>& _p) const {
  double minClearance = numeric_limits<double>::max();
  int id, faceID = 0;
  for(size_t i = 0; i < _p.size(); ++i) {
    if((_p[i] - m_bbx[i].first) < (m_bbx[i].second - _p[i]))
      id = i;
    else
      id = i+3;
    double clearance = min((_p[i] - m_bbx[i].first ), (m_bbx[i].second - _p[i]));
    if (clearance < minClearance || i == 0) {
      faceID = id;
      minClearance = clearance;
    }
  }
  return faceID;
}


Vector3d
BoundingBox::
GetClearancePoint(const Vector3d& _p) const {
  Vector3d clrP;
  double minClearance = numeric_limits<double>::max();
  for(size_t i = 0; i < 3; ++i) {
    if(_p[i] - m_bbx[i].first < minClearance){
      minClearance = _p[i] - m_bbx[i].first;
      clrP = _p;
      clrP[i] = m_bbx[i].first;
    }
    if(m_bbx[i].second - _p[i] < minClearance){
      minClearance = m_bbx[i].second - _p[i];
      clrP = _p;
      clrP[i] = m_bbx[i].second;
    }
  }
  return clrP;
}

/*---------------------------------- Modifiers -------------------------------*/

void
BoundingBox::
ApplyOffset(const Vector3d& _v) {
  m_center += _v;
  for(size_t i = 0; i < 3; ++i) {
    m_bbx[i].first += _v[i];
    m_bbx[i].second += _v[i];
  }
}


void
BoundingBox::
ResetBoundary(const vector<pair<double, double>>& _bbx, double _margin) {
  for(size_t i = 0; i<3; ++i){
    m_bbx[i].first = _bbx[i].first - _margin;
    m_bbx[i].second = _bbx[i].second + _margin;
  }
}

/*------------------------------------ I/O -----------------------------------*/

void
BoundingBox::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  m_bbx[0].first = m_bbx[1].first = m_bbx[2].first =
      -numeric_limits<double>::max();
  m_bbx[0].second = m_bbx[1].second = m_bbx[2].second =
      numeric_limits<double>::max();

  //check for first [
  string tok;
  char sep;
  if(!(_is >> sep && sep == '['))
    throw ParseException(_cbs.Where(),
        "Failed reading bounding box. Missing '['.");

  //read min:max 0
  if(!(_is >> m_bbx[0].first >> sep >> m_bbx[0].second) && sep != ':')
    throw ParseException(_cbs.Where(), "Failed reading bounding box range 0.");

  //read ;
  if(!(_is >> sep && sep == ';'))
    throw ParseException(_cbs.Where(), "Failed reading separator ';'.");

  //read min:max 1
  if(!(_is >> m_bbx[1].first >> sep >> m_bbx[1].second) && sep != ':')
    throw ParseException(_cbs.Where(), "Failed reading bounding box range 1.");

  //read ;
  if(!(_is >> sep && sep == ';'))
    throw ParseException(_cbs.Where(), "Failed reading separator ';'.");

  //read min:max 2
  if(!(_is >> m_bbx[2].first >> sep >> m_bbx[2].second) && sep != ':')
    throw ParseException(_cbs.Where(), "Failed reading bounding box range 2.");

  if(!(_is >> sep) && sep != ']')
    throw ParseException(_cbs.Where(),
        "Failed reading bounding box. Missing ']'.");
}


void
BoundingBox::
Write(ostream& _os) const {
  _os << "[ ";
  _os << m_bbx[0].first << ':' << m_bbx[0].second << " ; ";
  _os << m_bbx[1].first << ':' << m_bbx[1].second;
  _os << " ; " << m_bbx[2].first << ':' << m_bbx[2].second;
  _os << " ]";
}

/*--------------------------- CGAL Representation ----------------------------*/

Boundary::CGALPolyhedron
BoundingBox::
CGAL() const {
  // Define builder object.
  struct builder : public CGAL::Modifier_base<CGALPolyhedron::HalfedgeDS> {

    const pair<double, double>* m_bbx;

    builder(const pair<double, double>* _bbx) : m_bbx(_bbx) {}

    void operator()(CGALPolyhedron::HalfedgeDS& _h) {
      using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
      CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

      b.begin_surface(8, 6, 24);

      b.add_vertex(Point(m_bbx[0].first  , m_bbx[1].first , m_bbx[2].first ));
      b.add_vertex(Point(m_bbx[0].second , m_bbx[1].first , m_bbx[2].first ));
      b.add_vertex(Point(m_bbx[0].second , m_bbx[1].second, m_bbx[2].first ));
      b.add_vertex(Point(m_bbx[0].first  , m_bbx[1].second, m_bbx[2].first ));
      b.add_vertex(Point(m_bbx[0].first  , m_bbx[1].first , m_bbx[2].second));
      b.add_vertex(Point(m_bbx[0].second , m_bbx[1].first , m_bbx[2].second));
      b.add_vertex(Point(m_bbx[0].second , m_bbx[1].second, m_bbx[2].second));
      b.add_vertex(Point(m_bbx[0].first  , m_bbx[1].second, m_bbx[2].second));

      // Front
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(2);
      b.add_vertex_to_facet(3);
      b.end_facet();

      // Right
      b.begin_facet();
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(5);
      b.add_vertex_to_facet(6);
      b.add_vertex_to_facet(2);
      b.end_facet();

      // Back
      b.begin_facet();
      b.add_vertex_to_facet(5);
      b.add_vertex_to_facet(4);
      b.add_vertex_to_facet(7);
      b.add_vertex_to_facet(6);
      b.end_facet();

      // Left
      b.begin_facet();
      b.add_vertex_to_facet(4);
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(3);
      b.add_vertex_to_facet(7);
      b.end_facet();

      // Top
      b.begin_facet();
      b.add_vertex_to_facet(3);
      b.add_vertex_to_facet(2);
      b.add_vertex_to_facet(6);
      b.add_vertex_to_facet(7);
      b.end_facet();

      // Bottom
      b.begin_facet();
      b.add_vertex_to_facet(4);
      b.add_vertex_to_facet(5);
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(0);
      b.end_facet();

      b.end_surface();
    }
  };

  CGALPolyhedron cp;
  builder b(m_bbx);
  cp.delegate(b);

  if(!cp.is_valid())
    throw RunTimeException(WHERE, "BoundingBox:: Invalid CGAL polyhedron "
        "created!");
  return cp;
}

/*----------------------------------------------------------------------------*/
