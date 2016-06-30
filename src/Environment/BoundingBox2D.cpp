#include "BoundingBox2D.h"

#include "Utilities/MPUtils.h"

BoundingBox2D::
BoundingBox2D() {
  for(size_t i = 0; i < 2; ++i)
    m_bbx[i] = make_pair(-numeric_limits<double>::max(),
        numeric_limits<double>::max());
}

BoundingBox2D::
BoundingBox2D(pair<double, double> _x, pair<double, double> _y) {
  m_bbx[0] = _x;
  m_bbx[1] = _y;
  m_center = (Vector3d(_x.first, _y.first, 0) +
      Vector3d(_x.second, _y.second, 0))/2.;
}

double
BoundingBox2D::
GetMaxDist(double _r1, double _r2) const {
  double maxdist = 0;
  for(size_t i = 0; i < 2; ++i) {
    double diff = m_bbx[i].second - m_bbx[i].first;
    maxdist += pow(diff, _r1);
  }
  return pow(maxdist, _r2);
}

pair<double, double>
BoundingBox2D::
GetRange(size_t _i) const {
  if(_i > 2)
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + ::to_string(_i) + "'.");
  if(_i > 1)
    return make_pair(-numeric_limits<double>::max(), numeric_limits<double>::max());
  return m_bbx[_i];
}

Point3d
BoundingBox2D::
GetRandomPoint() const {
  Point3d p;
  for(size_t i = 0; i < 2; ++i)
    p[i] = m_bbx[i].first + (m_bbx[i].second - m_bbx[i].first)*DRand();
  return p;
}

bool
BoundingBox2D::
InBoundary(const Vector3d& _p) const {
  for(size_t i = 0; i < 2; ++i)
    if( _p[i] < m_bbx[i].first || _p[i] > m_bbx[i].second)
      return false;
  return true;
}

double
BoundingBox2D::
GetClearance(const Vector3d& _p) const {
  double minClearance = numeric_limits<double>::max();
  for(size_t i = 0; i < 2; ++i) {
    double clearance = min((_p[i] - m_bbx[i].first ), (m_bbx[i].second - _p[i]));
    if (clearance < minClearance || i == 0)
      minClearance = clearance;
  }
  return minClearance;
}

int
BoundingBox2D::
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
BoundingBox2D::
GetClearancePoint(const Vector3d& _p) const {
  Vector3d clrP;
  double minClearance = numeric_limits<double>::max();
  for(size_t i = 0; i < 2; ++i) {
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

double
BoundingBox2D::
GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const {
  double minDist=1e10;
  double cbbx[6]={m_bbx[0].first,m_bbx[0].second,
    m_bbx[1].first,m_bbx[1].second,
    0, 0};
  double dist[4]={_pos[0]-cbbx[0],cbbx[1]-_pos[0],
    _pos[1]-cbbx[4],cbbx[5]-_pos[1]};
  if(dist[0]<minDist){
    minDist=dist[0];
    _cdPt(cbbx[0], _pos[1]);
  }
  if(dist[1]<minDist){
    minDist=dist[1];
    _cdPt(cbbx[1], _pos[1]);
  }
  if(dist[2]<minDist){
    minDist=dist[2];
    _cdPt(_pos[0], cbbx[4]);
  }
  if(dist[3]<minDist){
    minDist=dist[3];
    _cdPt(_pos[0], cbbx[5]);
  }

  if( minDist<0 ) minDist=0;

  return minDist;
}

void
BoundingBox2D::
ApplyOffset(const Vector3d& _v) {
  m_center += _v;
  for(size_t i = 0; i < 2; ++i) {
    m_bbx[i].first += _v[i];
    m_bbx[i].second += _v[i];
  }
}

void
BoundingBox2D::
ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d){
  for(size_t i = 0; i < 2; ++i){
    m_bbx[i].first = _obstBBX[i].first - _d;
    m_bbx[i].second = _obstBBX[i].second + _d;
  }
}

void
BoundingBox2D::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  m_bbx[0].first = m_bbx[1].first = -numeric_limits<double>::max();
  m_bbx[0].second = m_bbx[1].second = numeric_limits<double>::max();

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

  if(!(_is >> sep) && sep != ']')
    throw ParseException(_cbs.Where(),
        "Failed reading bounding box. Missing ']'.");
}

void
BoundingBox2D::
Write(ostream& _os) const {
  _os << "[ ";
  _os << m_bbx[0].first << ':' << m_bbx[0].second << " ; ";
  _os << m_bbx[1].first << ':' << m_bbx[1].second;
  _os << " ]";
}

Boundary::CGALPolyhedron
BoundingBox2D::
CGAL() const {
  // Define builder object.
  struct builder : public CGAL::Modifier_base<CGALPolyhedron::HalfedgeDS> {

    const pair<double, double>* m_bbx;

    builder(const pair<double, double>* _bbx) : m_bbx(_bbx) {}

    void operator()(CGALPolyhedron::HalfedgeDS& _h) {
      using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
      CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

      b.begin_surface(4, 2, 8);

      b.add_vertex(Point(m_bbx[0].first  , m_bbx[1].first , 0));
      b.add_vertex(Point(m_bbx[0].second , m_bbx[1].first , 0));
      b.add_vertex(Point(m_bbx[0].second , m_bbx[1].second, 0));
      b.add_vertex(Point(m_bbx[0].first  , m_bbx[1].second, 0));

      // The bounding box has two facets: one on top, and one below. This is
      // needed to create a closed surface as this representation is three-
      // dimensional.
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(2);
      b.add_vertex_to_facet(3);
      b.end_facet();

      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(3);
      b.add_vertex_to_facet(2);
      b.add_vertex_to_facet(1);
      b.end_facet();

      b.end_surface();
    }
  };

  CGALPolyhedron cp;
  builder b(m_bbx);
  cp.delegate(b);

  if(!cp.is_valid())
    throw RunTimeException(WHERE, "BoundingBox2D:: Invalid CGAL polyhedron "
        "created!");
  return cp;
}
