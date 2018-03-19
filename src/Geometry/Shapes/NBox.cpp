#include "NBox.h"

#include <cmath>

#include "Utilities/PMPLExceptions.h"

#include "nonstd/container_ops.h"


/*------------------------------ Construction --------------------------------*/

NBox::
NBox(const size_t _n) : m_center(_n, 0),
    m_range(_n, Range<double>(std::numeric_limits<double>::lowest(),
                              std::numeric_limits<double>::max())) {
}


NBox::
NBox(const std::vector<double>& _center) : m_center(_center),
    m_range(m_center.size(), Range<double>(std::numeric_limits<double>::lowest(),
                                           std::numeric_limits<double>::max())) {
}


NBox::
~NBox() noexcept = default;

/*------------------------------- Accessors ----------------------------------*/

size_t
NBox::
GetDimension() const noexcept {
  return m_center.size();
}


void
NBox::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i) {
    const double offset = _c[i] - m_center[i];
    m_range[i].min += offset;
    m_range[i].max += offset;
    m_center[i] = _c[i];
  }
}


const std::vector<double>&
NBox::
GetCenter() const noexcept {
  return m_center;
}


const Range<double>&
NBox::
GetRange(const size_t _i) const noexcept {
  return m_range[_i];
}


const std::vector<Range<double>>&
NBox::
GetRanges() const noexcept {
  return m_range;
}


void
NBox::
SetRange(const size_t _i, const Range<double>& _r) noexcept {
  m_range[_i] = _r;
  m_center[_i] = m_range[_i].Center();
}


void
NBox::
SetRange(const size_t _i, Range<double>&& _r) noexcept {
  m_range[_i] = std::move(_r);
  m_center[_i] = m_range[_i].Center();
}


void
NBox::
SetRange(const size_t _i, const double _min, const double _max) noexcept {
  m_range[_i].Resize(_min, _max);
  m_center[_i] = m_range[_i].Center();
}


void
NBox::
Translate(const std::vector<double>& _v) noexcept {
  const size_t maxIndex = std::min(_v.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i) {
    m_range[i].min += _v[i];
    m_range[i].max += _v[i];
    m_center[i] += _v[i];
  }
}

/*------------------------------ Point Testing -------------------------------*/

bool
NBox::
Contains(const std::vector<double>& _p) const noexcept {
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    if(!m_range[i].Contains(_p[i]))
      return false;
  return true;
}


double
NBox::
Clearance(const std::vector<double>& _p) const noexcept {
  double minClearance = std::numeric_limits<double>::max();

  const size_t maxIndex = std::min(_p.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i) {
    // If _p is outside the range, we must use a more expensive computation.
    if(!m_range[i].Contains(_p[i]))
    {
      auto cp = ClearancePoint(_p);
      for(size_t i = 0; i < cp.size(); ++i)
        cp[i] -= _p[i];
      return -nonstd::magnitude<double>(cp);
    }
    minClearance = std::min(minClearance, m_range[i].Clearance(_p[i]));
  }
  return minClearance;
}


std::vector<double>
NBox::
ClearancePoint(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  _p.resize(maxIndex);

  // Find the clearance in each dimension.
  double minClearance = numeric_limits<double>::max();
  size_t index = -1;

  for(size_t i = 0; i < maxIndex; ++i) {
    const auto& r = m_range[i];

    // Compute clearance in this dimension.
    const double clearance = r.Clearance(_p[i]);

    // If _p lies outside the range in this dimension, use the closest endpoint.
    if(clearance < 0)
      _p[i] = r.ClearancePoint(_p[i]);
    // If _p lies inside the range in this dimension and is closer than previous
    // best, update the clearance index.
    else if(clearance < minClearance) {
      index = i;
      minClearance = clearance;
    }
  }

  // If at least one dimension of _p was inside the range, push _p to the
  // boundary in the closest dimension.
  if(index != size_t(-1))
    _p[index] = m_range[index].ClearancePoint(_p[index]);

  return _p;
}

/*-------------------------------- Sampling ----------------------------------*/

std::vector<double>
NBox::
Sample() const {
  std::vector<double> point(GetDimension());
  for(size_t i = 0; i < GetDimension(); ++i)
    point[i] = m_range[i].Sample();
  return point;
}

/*----------------------------------- I/O ------------------------------------*/

std::istream&
operator>>(std::istream& _is, NBox& _box) {
  _box.m_center.clear();
  _box.m_range.clear();

  // Read opening bracket.
  char c;
  if(!(_is >> c && c == '['))
    throw ParseException(WHERE, "Failed reading NBox bounds. Missing '['.");

  Range<double> r;
  while(true) {
    // Read the next range.
    if(!(_is >> r))
      throw ParseException(WHERE, "Failed reading NBox range " +
          std::to_string(_box.m_range.size()) + ".");
    _box.m_range.push_back(r);
    _box.m_center.push_back(r.Center());

    // Read the next separator
    if(!(_is >> c) || (c != ';' && c != ']'))
      throw ParseException(WHERE, "Failed reading NBox bounds. Missing ';' "
          "or ']'.");

    if(c != ';')
      break;
  }

  return _is;
}


std::ostream&
operator<<(std::ostream& _os, const NBox& _box) {
  _os << "[";
  size_t i = 0;
  for(; i < _box.GetDimension() - 1; ++i)
    _os << _box.GetRange(i) << " ; ";
  return _os << _box.GetRange(i) << " ]";
}

/*----------------------------------------------------------------------------*/
