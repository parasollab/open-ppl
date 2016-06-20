#include "BoundingSphere2D.h"

#include "Utilities/MPUtils.h"

BoundingSphere2D::
BoundingSphere2D() : m_radius(numeric_limits<double>::max()) {
}

BoundingSphere2D::
BoundingSphere2D(const Vector2d& _center, double _radius) :
  m_center(_center[0], _center[1], 0), m_radius(_radius) {
}

double
BoundingSphere2D::
GetMaxDist(double _r1, double _r2) const {
  return pow(2*m_radius, _r1 * _r2);
}

pair<double, double>
BoundingSphere2D::
GetRange(size_t _i) const {
  if(_i > 2)
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + ::to_string(_i) + "'.");
  if(_i > 1)
    return make_pair(-numeric_limits<double>::max(), numeric_limits<double>::max());
  return make_pair(m_center[_i]-m_radius, m_center[_i]+m_radius);
}

Point3d
BoundingSphere2D::
GetRandomPoint() const {
  double t = DRand() * TWOPI, r = DRand();
  double sr = sqrt(r);
  Vector3d p(sr*cos(t), sr*sin(t), 0);
  return p*m_radius + m_center;
}

bool
BoundingSphere2D::
InBoundary(const Vector3d& _p) const {
  Vector3d p(_p[0], _p[1], 0);
  return (p-m_center).norm() < m_radius;
}

double
BoundingSphere2D::
GetClearance(const Vector3d& _p) const {
  Vector3d p(_p[0], _p[1], 0);
  return m_radius - (p - m_center).norm();
}

int
BoundingSphere2D::
GetSideID(const vector<double>& _p) const {
  return -1;
}


Vector3d
BoundingSphere2D::
GetClearancePoint(const Vector3d& _p) const {
  Vector3d p(_p[0], _p[1], 0);
  Vector3d v = (p - m_center).normalize();
  Vector3d c = v*m_radius + m_center;
  c[2] = _p[2];
  return c;
}

double
BoundingSphere2D::
GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const {
  throw RunTimeException(WHERE, "Not implemented.");
}

void
BoundingSphere2D::
ApplyOffset(const Vector3d& _v) {
  m_center += _v;
}

void
BoundingSphere2D::
ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d) {
  double maxRange = -numeric_limits<double>::max();
  for(int i = 0; i<3; ++i) {
    double diff = _obstBBX[i].second - _obstBBX[i].first;
    if(diff > maxRange)
      maxRange = diff;
  }
  m_center[0] = (_obstBBX[0].second + _obstBBX[0].first)/2;
  m_center[1] = (_obstBBX[1].second + _obstBBX[1].first)/2;
  m_radius = maxRange + _d;
}

void
BoundingSphere2D::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  //check for first [
  string tok;
  if(!(_is >> tok && tok == "["))
    throw ParseException(_cbs.Where(),
        "Failed reading bounding sphere. Missing '['.");

  Vector2d center = ReadField<Vector2d>(_is, _cbs,
      "Failed reading center point of bounding sphere.");
  m_center(center[0], center[1], 0);

  //read comma
  if(!(_is >> tok && tok == ";"))
    throw ParseException(_cbs.Where(), "Failed reading bounding box. "
        "Missing ';' separator between dimensions.");

  m_radius = ReadField<double>(_is, _cbs,
      "Failed reading radius of bounding sphere.");

  //check for ending ]
  if(!(_is >> tok && tok == "]"))
    throw ParseException(_cbs.Where(), "Failed reading bounding sphere. Missing ']'.");
}

void
BoundingSphere2D::
Write(ostream& _os) const {
  _os << "[ " << m_center[0] << " " << m_center[1] << " ; " << m_radius << " ]";
}

