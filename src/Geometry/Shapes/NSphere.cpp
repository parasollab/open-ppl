#include "NSphere.h"

#include "nonstd/container_ops.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/MPUtils.h"


/*------------------------------ Construction --------------------------------*/

NSphere::
NSphere(const size_t _n, const double _r) : m_center(_n, 0), m_radius(_r) { }


NSphere::
NSphere(const std::vector<double>& _c, const double _r) : m_center(_c),
    m_radius(_r) { }

/*------------------------------- Accessors ----------------------------------*/

size_t
NSphere::
GetDimension() const noexcept {
  return m_center.size();
}


void
NSphere::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] = _c[i];
}


const std::vector<double>&
NSphere::
GetCenter() const noexcept {
  return m_center;
}


double
NSphere::
GetRadius() const noexcept {
  return m_radius;
}


void
NSphere::
SetRadius(const double _r) noexcept {
  m_radius = _r;
}


void
NSphere::
Translate(const std::vector<double>& _v) noexcept {
  const size_t maxIndex = std::min(_v.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] += _v[i];
}

/*--------------------------------- Testing ----------------------------------*/

bool
NSphere::
Contains(const std::vector<double>& _p) const {
  return Clearance(_p) >= 0;
}


double
NSphere::
Clearance(std::vector<double> _p) const {
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    _p[i] -= m_center[i];
  return m_radius - nonstd::magnitude<double>(_p);
}


std::vector<double>
NSphere::
ClearancePoint(std::vector<double> _p) const {
  // Ensure _p has full dimension.
  if(_p.size() != GetDimension())
    throw RunTimeException(WHERE, "The point and boundary must have the same "
        "dimension for this function!");

  // Find the vector from m_center to _p.
  const size_t maxIndex = GetDimension();
  for(size_t i = 0; i < maxIndex; ++i)
    _p[i] -= m_center[i];

  // Scale the vector to length m_radius.
  const double scale = m_radius / nonstd::magnitude<double>(_p);
  for(auto& v : _p)
    v *= scale;

  return _p;
}

/*--------------------------------- Sampling ---------------------------------*/

std::vector<double>
NSphere::
Sample() const {
  // Generate a random value for each dimension in the range [-1,1] with
  // gaussian probability.
  std::vector<double> point(GetDimension());
  for(auto& value : point)
    value = GRand();

  // Scale and translate the point appropriately.
  const double scale = m_radius * DRand() / nonstd::magnitude<double>(point);
  for(size_t i = 0; i < point.size(); ++i) {
    point[i] *= scale;
    point[i] += m_center[i];
  }

  return point;
}

/*----------------------------------- I/O ------------------------------------*/

std::istream&
operator>>(std::istream& _is, NSphere& _sphere) {
  _sphere.m_center.clear();

  // Read opening bracket.
  char c;
  if(!(_is >> c && c == '['))
    throw ParseException(WHERE, "Failed reading NSphere bounds. Missing '['.");

  // Read the center point values.
  double temp;
  while(1) {
    // Eat white space.
    _is >> std::ws;

    // If the next character is not digit, we are done reading center point
    // values.
    if(!isdigit(_is.peek()))
      break;

    // Otherwise, read the next center point value.
    if(!(_is >> temp))
      throw ParseException(WHERE, "Failed reading center point value " +
          std::to_string(_sphere.m_center.size()) + ".");
    _sphere.m_center.push_back(temp);
  }

  // Read separator.
  if(!(_is >> c and c == ';'))
    throw ParseException(WHERE, "Failed reading NSphere bounds. Missing ';'.");

  // Read the radius.
  if(!(_is >> temp))
    throw ParseException(WHERE, "Failed reading NSphere radius.");
  _sphere.m_radius = temp;

  // Read the last separator.
  _is >> std::ws;
  if(!(_is >> c and c == ']'))
    throw ParseException(WHERE, "Failed reading NSphere bounds. Missing ']'.");

  return _is;
}


std::ostream&
operator<<(std::ostream& _os, const NSphere& _sphere) {
  _os << "[ ";
  for(size_t i = 0; i < _sphere.GetDimension() - 1; ++i)
    _os << _sphere.GetCenter()[i] << " ";
  return _os << "; " << _sphere.GetRadius() << " ]";
}

/*----------------------------------------------------------------------------*/
