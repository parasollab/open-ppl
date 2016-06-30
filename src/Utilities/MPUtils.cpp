#include "MPUtils.h"
#include "MetricUtils.h"

/*------------------------- Random Number Generation -------------------------*/

double DRand() {return drand48();}
long LRand() {return lrand48();}
long MRand() {return mrand48();}

double
GRand(bool _reset) {
  static bool cached = false;
  static double cachedValue;

  // Clear the cache if reset was called.
  if(_reset) {
    cached = false;
    return 0.;
  }

  // If an output is cached, return it.
  if(cached) {
    cached = false;
    return cachedValue;
  }

  // Otherwise, compute the next two values. Store one, cache the other.
  double v1, v2, rsq;
  do {
    v1 = 2 * DRand() - 1.;
    v2 = 2 * DRand() - 1.;
    rsq = v1*v1 + v2*v2;
  } while(rsq >= 1. || rsq == 0.);
  double fac = sqrt(-2. * log(rsq) / rsq);
  cachedValue = v1 * fac;
  cached = true;
  return v2 * fac;
}


double
GaussianDistribution(double _mean, double _stdev) {
  return GRand(false) * _stdev + _mean;
}


long
SRand(long _seedVal) {
  static long oldSeed = _seedVal;
  if(oldSeed == _seedVal)
    return SRand("NONE", 0, _seedVal);
  oldSeed = _seedVal;
  return SRand("NONE", 0, _seedVal, true);
}


long
SRand(string _methodName, int _nextNodeIndex, long _base, bool _reset) {
  static long baseSeed = _base;
  if(_reset)
    baseSeed = _base;
  if(_methodName != "NONE") {
    long methodID = 0;
    for(size_t i = 0; i < _methodName.length(); ++i) {
      int tmp = _methodName[i];
      methodID += tmp * (i + 1) * (i + 2);
    }
    srand48(static_cast<long>(baseSeed * (_nextNodeIndex + 1) + methodID));
  }
  else {
    srand48(baseSeed);
  }
  return baseSeed;
}

/*------------------------------ Geometry Utils ------------------------------*/

double
Normalize(double _a) {
  _a = fmod(_a + 1., 2.);
  if(_a < 0.)
    _a += 2.;
  return --_a;
}


double
DirectedAngularDistance(double _a, double _b) {
  // normalize both a and b to [-1, 1)
  _a = Normalize(_a);
  _b = Normalize(_b);

  if( _b - _a  > 1.0 )
    _a+=2.0;
  else if ( _a - _b > 1.0 )
    _b+=2.0;

  return _b-_a;
}


double
TriangleHeight(const Point3d& _a, const Point3d& _b, const Point3d& _c) {
  //Using Heron's formula
  double ab = (_a - _b).norm();
  double bc = (_b - _c).norm();
  double ac = (_a - _c).norm();
  double p = (ab + bc + ac) / 2; //half of the perimeter
  double area = sqrt(p * (p - ab) * (p - bc) * (p - ac));
  double height = 2 * area / (max(max(ab, bc), ac)); //h = 2A/b
  return height;
}


bool
PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d & _p) {
  // Check whether _p is inside the triangle by ensuring that it is in the
  // correct half-space of each edge segment.
  Vector2d ab = _b - _a,
           bc = _c - _b;
  // CW
  if(ab[0] * bc[1] - ab[1] * bc[0] < 0) {
    // ABxAP
    if(ab[0] * (_p[1] - _a[1]) >= ab[1] * (_p[0] - _a[0]))
      return false;
    // BCxBP
    if(bc[0] * (_p[1] - _b[1]) >= bc[1] * (_p[0] - _b[0]))
      return false;
    // CAxCP
    if((_a[0] - _c[0]) * (_p[1] - _c[1]) >= (_a[1] - _c[1]) * (_p[0] - _c[0]))
      return false;
  }
  // CCW
  else {
    // ABxAP
    if(ab[0] * (_p[1] - _a[1]) < ab[1] * (_p[0] - _a[0]))
      return false;
    // BCxBP
    if(bc[0] * (_p[1] - _b[1]) < bc[1] * (_p[0] - _b[0]))
      return false;
    // CAxCP
    if((_a[0] - _c[0]) * (_p[1] - _c[1]) < (_a[1] - _c[1]) * (_p[0] - _c[0]))
      return false;
  }
  return true;
}


bool
PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d& _p, double& _u, double& _v) {

  double epsilon = 0.0000001;

  // Compute vectors
  Vector2d v0 = _c - _a;
  Vector2d v1 = _b - _a;
  Vector2d v2 = _p - _a;

  // Compute dot products
  double dot00 = v0 * v0;
  double dot01 = v0 * v1;
  double dot02 = v0 * v2;
  double dot11 = v1 * v1;
  double dot12 = v1 * v2;

  // Compute barycentric coordinates
  double invDenom = 1. / (dot00 * dot11 - dot01 * dot01);
  _u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  _v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (_u >= -epsilon) && (_v >= -epsilon) && (_u + _v < 1. + epsilon);
}


Point3d
GetPtFromBarycentricCoords(const Point3d& _a, const Point3d& _b,
    const Point3d& _c, double _u, double _v) {
  return _a + (_u * (_c - _a)) + (_v * (_b - _a));
}


double
NormalizeTheta(double _theta) {
  double val = _theta + PI;
  return val == 0 ? _theta : val - TWOPI * floor(val / TWOPI) - PI;
}

/*----------------------------------------------------------------------------*/
