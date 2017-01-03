#include "Geometry/Boundaries/BoundingBox.h"

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/MPUtils.h"

/*------------------------------- Construction -------------------------------*/

BoundingBox::
BoundingBox(const size_t _dimension) {
  m_bbx.reserve(_dimension);
  for(size_t i = 0; i < _dimension; ++i)
    m_bbx.emplace_back(numeric_limits<double>::lowest(),
                       numeric_limits<double>::max());
}

BoundingBox::
BoundingBox(pair<double, double> _x, pair<double, double> _y,
    pair<double, double> _z) {
  m_bbx.reserve(3);
  m_bbx.emplace_back(_x);
  m_bbx.emplace_back(_y);
  m_bbx.emplace_back(_z);
  UpdateCenter();
}

/*---------------------------- Property Accessors ----------------------------*/

std::string
BoundingBox::
Type() const noexcept {
  return "Box";
}


double
BoundingBox::
GetMaxDist(double _r1, double _r2) const {
  double maxdist = 0;
  for(size_t i = 0; i < 3; ++i)
    maxdist += pow(m_bbx[i].Length(), _r1);
  return pow(maxdist, _r2);
}


pair<double, double>
BoundingBox::
GetRange(size_t _i) const {
  if(_i > m_bbx.size())
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + ::to_string(_i) + "'.");
  /// @TODO Return ranges instead of pairs.
  return make_pair(m_bbx[_i].min, m_bbx[_i].max);
}

/*-------------------------------- Sampling ----------------------------------*/

Point3d
BoundingBox::
GetRandomPoint() const {
  Point3d p;
  for(size_t i = 0; i < 3; ++i)
    p[i] = m_bbx[i].min + m_bbx[i].Length() * DRand();
  return p;
}

/*----------------------------- Containment Testing --------------------------*/

bool
BoundingBox::
InBoundary(const Vector3d& _p) const {
  for(size_t i = 0; i < 3; ++i)
    if(!m_bbx[i].Contains(_p[i]))
      return false;
  return true;
}


bool
BoundingBox::
InCSpace(const Cfg& _cfg) const {
  const auto& data = _cfg.GetData();
  for(size_t i = 0; i < m_bbx.size(); ++i)
    if(!m_bbx[i].Contains(data[i]))
      return false;
  return true;
}

/*------------------------------ Clearance Testing ---------------------------*/

double
BoundingBox::
GetClearance(const Vector3d& _p) const {
  double minClearance = numeric_limits<double>::max();
  for(size_t i = 0; i < 3; ++i)
    minClearance = min(minClearance, m_bbx[i].Clearance(_p[i]));
  return minClearance;
}


int
BoundingBox::
GetSideID(const vector<double>& _p) const {
  double minClearance = numeric_limits<double>::max();
  int id = 0;
  for(size_t i = 0; i < 3; ++i) {
    const double clearance = m_bbx[i].Clearance(_p[i]);
    if(clearance < minClearance) {
      minClearance = clearance;
      if((_p[i] - m_bbx[i].min) < (m_bbx[i].max - _p[i]))
        id = i;
      else
        id = i + 3;
    }
  }
  return id;
}


Vector3d
BoundingBox::
GetClearancePoint(const Vector3d& _p) const {
  Vector3d clrP = _p;
  double minClearance = numeric_limits<double>::max();
  pair<size_t, double> adjust;
  for(size_t i = 0; i < 3; ++i) {
    if(_p[i] - m_bbx[i].min < minClearance){
      minClearance = _p[i] - m_bbx[i].min;
      clrP = _p;
      clrP[i] = m_bbx[i].min;
    }
    if(m_bbx[i].max - _p[i] < minClearance){
      minClearance = m_bbx[i].max - _p[i];
      clrP = _p;
      clrP[i] = m_bbx[i].max;
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
    m_bbx[i].min += _v[i];
    m_bbx[i].max += _v[i];
  }
}


void
BoundingBox::
ResetBoundary(const vector<pair<double, double>>& _bbx, double _margin) {
  for(size_t i = 0; i < min(m_bbx.size(), _bbx.size()); ++i)
    SetRange(i, _bbx[i].first - _margin, _bbx[i].second + _margin);
  UpdateCenter();
}


void
BoundingBox::
SetRange(const size_t _i, const double _min, const double _max) {
  m_bbx[_i].min = _min;
  m_bbx[_i].max = _max;
  UpdateCenter();
}


void
BoundingBox::
SetRange(const size_t _i, Range<double>&& _r) {
  m_bbx[_i] = move(_r);
  UpdateCenter();
}

/*------------------------------------ I/O -----------------------------------*/

void
BoundingBox::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  for(size_t i = 0; i < m_bbx.size(); ++i)
    SetRange(i, numeric_limits<double>::lowest(), numeric_limits<double>::max());

  //check for first [
  string tok;
  char sep;
  if(!(_is >> sep && sep == '['))
    throw ParseException(_cbs.Where(),
        "Failed reading bounding box. Missing '['.");

  //read min:max 0
  if(!(_is >> m_bbx[0]))
    throw ParseException(_cbs.Where(), "Failed reading bounding box range 0.");

  //read ;
  if(!(_is >> sep && sep == ';'))
    throw ParseException(_cbs.Where(), "Failed reading separator ';'.");

  //read min:max 1
  if(!(_is >> m_bbx[1]))
    throw ParseException(_cbs.Where(), "Failed reading bounding box range 1.");

  //read ;
  if(!(_is >> sep && sep == ';'))
    throw ParseException(_cbs.Where(), "Failed reading separator ';'.");

  //read min:max 2
  if(!(_is >> m_bbx[2]))
    throw ParseException(_cbs.Where(), "Failed reading bounding box range 2.");

  if(!(_is >> sep && sep == ']'))
    throw ParseException(_cbs.Where(),
        "Failed reading bounding box. Missing ']'.");
}


void
BoundingBox::
Write(ostream& _os) const {
  _os << "[ ";
  size_t i = 0;
  for(; i < m_bbx.size() - 1; ++i)
    _os << m_bbx[i] << " ; ";
  _os << m_bbx[i] << " ]";
}

/*--------------------------- CGAL Representation ----------------------------*/

Boundary::CGALPolyhedron
BoundingBox::
CGAL() const {
  // Define builder object.
  struct builder : public CGAL::Modifier_base<CGALPolyhedron::HalfedgeDS> {

    const std::vector<Range<double>>& m_bbx;

    builder(const std::vector<Range<double>>& _bbx) : m_bbx(_bbx) {}

    void operator()(CGALPolyhedron::HalfedgeDS& _h) {
      using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
      CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

      b.begin_surface(8, 6, 24);

      b.add_vertex(Point(m_bbx[0].min, m_bbx[1].min, m_bbx[2].min));
      b.add_vertex(Point(m_bbx[0].max, m_bbx[1].min, m_bbx[2].min));
      b.add_vertex(Point(m_bbx[0].max, m_bbx[1].max, m_bbx[2].min));
      b.add_vertex(Point(m_bbx[0].min, m_bbx[1].max, m_bbx[2].min));
      b.add_vertex(Point(m_bbx[0].min, m_bbx[1].min, m_bbx[2].max));
      b.add_vertex(Point(m_bbx[0].max, m_bbx[1].min, m_bbx[2].max));
      b.add_vertex(Point(m_bbx[0].max, m_bbx[1].max, m_bbx[2].max));
      b.add_vertex(Point(m_bbx[0].min, m_bbx[1].max, m_bbx[2].max));

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

/*--------------------------------- Helpers ----------------------------------*/

void
BoundingBox::
UpdateCenter() {
  /// @TODO Generalize the notion of boundary centers. Need one for CSpace and
  ///       one for Workspace.
  for(size_t i = 0; i < 3; ++i)
    m_center[i] = m_bbx[i].Center();
}

/*----------------------------------------------------------------------------*/
