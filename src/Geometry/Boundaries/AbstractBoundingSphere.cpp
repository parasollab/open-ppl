#include "Geometry/Boundaries/AbstractBoundingSphere.h"

#include "Geometry/Boundaries/Range.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"


/*------------------------------- Construction -------------------------------*/

AbstractBoundingSphere::
AbstractBoundingSphere(const size_t _n, const double _radius) :
    NSphere(_n, _radius) { }


AbstractBoundingSphere::
AbstractBoundingSphere(const std::vector<double>& _center, const double _radius) :
    NSphere(_center, _radius) { }

/*---------------------------- Property Accessors ----------------------------*/

double
AbstractBoundingSphere::
GetMaxDist(const double _r1, const double _r2) const {
  return std::pow(2. * NSphere::GetRadius(), _r1 * _r2);
}


Range<double>
AbstractBoundingSphere::
GetRange(const size_t _i) const {
  if(_i > NSphere::GetDimension())
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + std::to_string(_i) + "'.");
  return Range<double>(NSphere::GetCenter()[_i] - NSphere::GetRadius(),
                       NSphere::GetCenter()[_i] + NSphere::GetRadius());
}


const std::vector<double>&
AbstractBoundingSphere::
GetCenter() const noexcept {
  return NSphere::GetCenter();
}

/*-------------------------------- Sampling ----------------------------------*/

std::vector<double>
AbstractBoundingSphere::
GetRandomPoint() const {
  return NSphere::Sample();
}

/*----------------------------- Containment Testing --------------------------*/

bool
AbstractBoundingSphere::
InBoundary(const std::vector<double>& _p) const {
  return NSphere::Contains(_p);
}

/*------------------------------ Clearance Testing ---------------------------*/

double
AbstractBoundingSphere::
GetClearance(const Vector3d& _p) const {
  return NSphere::Clearance(std::vector<double>{_p[0], _p[1], _p[2]});
}


Vector3d
AbstractBoundingSphere::
GetClearancePoint(const Vector3d& _p) const {
  auto v = NSphere::ClearancePoint(std::vector<double>{_p[0], _p[1], _p[2]});
  return Vector3d(v[0], v[1], NSphere::GetDimension() > 2 ? v[2] : 0);
}

/*---------------------------------- Modifiers -------------------------------*/

void
AbstractBoundingSphere::
ApplyOffset(const Vector3d& _v) {
  NSphere::Translate(std::vector<double>{_v[0], _v[1], _v[2]});
}


void
AbstractBoundingSphere::
ResetBoundary(const vector<pair<double, double>>&, const double) {
  throw RunTimeException(WHERE, "This operation does not make sense for a "
      "sphere. Can't resize each dimension independently!");
}

/*------------------------------------ I/O -----------------------------------*/

void
AbstractBoundingSphere::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  // Try to read in using NSphere. Re-propogate any exceptions with the better
  // debug info from the CountingStreamBuffer.
  try {
    _is >> static_cast<NSphere&>(*this);
  }
  catch(PMPLException& _e) {
    throw ParseException(_cbs.Where(), _e.what());
  }
}


void
AbstractBoundingSphere::
Write(ostream& _os) const {
  _os << static_cast<const NSphere&>(*this);
}

/*----------------------------------------------------------------------------*/
