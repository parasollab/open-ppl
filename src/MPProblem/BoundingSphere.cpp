#include "BoundingSphere.h"
#include "Cfg/Cfg.h"

BoundingSphere::BoundingSphere() :
  m_center(0.0, 0.0, 0.0),
  m_radius(numeric_limits<double>::max()) {
}

BoundingSphere::BoundingSphere(const Vector3d& _center, double _radius) :
  m_center(_center), m_radius(_radius) {
}

BoundingSphere::BoundingSphere(const BoundingSphere& _bs) :
  m_center(_bs.m_center),
  m_radius(_bs.m_radius) {
  }

bool
BoundingSphere::operator==(const Boundary& _b) const {
  const BoundingSphere* bs = dynamic_cast<const BoundingSphere*>(&_b);
  return bs && m_center == bs->m_center && m_radius == bs->m_radius;
}

double
BoundingSphere::GetMaxDist(double _r1, double _r2) const {
  return pow(pow(2*m_radius, _r1), _r2);
}

pair<double, double>
BoundingSphere::GetRange(size_t _i) const {
  if(_i > 2){
    cerr << "Error::BoundingBox::GetRange::Invalid access to dimension " << _i << "." << endl;
    exit(1);
  }
  return make_pair(m_center[_i]-m_radius, m_center[_i]+m_radius);
}

Point3d
BoundingSphere::GetRandomPoint() const {
  //Box-Muller method for generating random point inside of a sphere
  double x = GRand(), y = GRand(), z = GRand(), u = DRand();
  double a = m_radius * u /sqrt(x*x + y*y + z*z);
  Point3d p(a*x, a*y, a*z);
  return p + m_center;
}

bool
BoundingSphere::InBoundary(const Vector3d& _p) const {
  return (_p-m_center).norm() < m_radius;
}

double
BoundingSphere::GetClearance(const Vector3d& _p) const {
  return m_radius - (_p - m_center).norm();
}

int
//BoundingSphere::GetSideID(const Vector3d& _p) const {
BoundingSphere::GetSideID(const vector<double>& _p) const {
  return -1;
}


Vector3d
BoundingSphere::GetClearancePoint(const Vector3d& _p) const {
  Vector3d v = (_p - m_center).normalize();
  return v + m_center;
}

double
BoundingSphere::GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const {
  cerr << "Error::GetClearance2DSurf not implemented for bounding sphere. Exiting." << endl;
  exit(1);
}

void
BoundingSphere::ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d) {
  double maxrange = -numeric_limits<double>::max();
  for(int i = 0; i<3; ++i){
    double diff = _obstBBX[i].second - _obstBBX[i].first;
    if(diff > maxrange)
      maxrange = diff;
  }
  m_center[0] = (_obstBBX[0].second + _obstBBX[0].first)/2;
  m_center[1] = (_obstBBX[1].second + _obstBBX[1].first)/2;
  m_center[2] = (_obstBBX[2].second + _obstBBX[2].first)/2;
  m_radius = maxrange + _d;
}

void
BoundingSphere::Read(istream& _is) {
  m_center = ReadField<Vector3d>(_is, WHERE, "Failed reading center point of bounding sphere.");
  m_radius = ReadField<double>(_is, WHERE, "Failed reading radius of bounding sphere.");
}

void
BoundingSphere::Write(ostream& _os) const {
  _os << "[ " << m_center << " " << m_radius << " ]";
}

