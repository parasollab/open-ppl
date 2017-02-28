#ifndef RANGE_TYPE_H_
#define RANGE_TYPE_H_

#include <cctype>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

#include "Utilities/MPUtils.h"


/*------------------------- Range-checking Functions -------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// Check a value for containment within a given range.
/// @param _val The value to test.
/// @param _min The lower bound of the range.
/// @param _max The upper bound of the range.
/// @param _inclusive Count values on the boundary as inside?
/// @return True if the test value lies inside the range (or on the boundary
///         for inclusive test).
template <typename T, typename U>
inline
bool
InRange(const U& _val, const T& _min, const T& _max, const bool _inclusive = true)
    noexcept {
  return _inclusive ? _min <= _val && _val <= _max :
                      _min <  _val && _val <  _max ;
}


////////////////////////////////////////////////////////////////////////////////
/// Check a value for containment within a given range.
/// @param _val The value to test.
/// @param _bounds A pair describing the lower, upper bounds.
/// @param _inclusive Count values on the boundary as inside?
/// @return True if the test value lies inside the range (or on the boundary
///         for inclusive test).
template <typename T, typename U>
inline
bool
InRange(const U& _val, const std::pair<T, T>& _bounds,
    const bool _inclusive = true) noexcept {
  return InRange(_val, _bounds.first, _bounds.second, _inclusive);
}


////////////////////////////////////////////////////////////////////////////////
/// Sample a range for an enclosed value with uniform probability.
/// @param _min The lower bound.
/// @param _max The upper bound.
/// @return A random value within [_min, _max] sampled with uniform probability.
template <typename T>
inline
T
SampleRange(const T& _min, const T& _max) noexcept {
  return _min + (_max - _min) * DRand();
}

/*-------------------------------- Range Type --------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// A range of numeric values.
////////////////////////////////////////////////////////////////////////////////
template <typename T>
struct Range final {

  ///@}
  ///@name Internal State
  ///@{

  T min;  ///< The lower bound on this range.
  T max;  ///< The upper bound on this range.

  ///@}
  ///@name Construction
  ///@{

  /// Construct a range over all values of T.
  Range() noexcept;

  /// Construct a bounded range.
  /// @param _min The lower bound.
  /// @param _max The upper bound.
  Range(const T _min, const T _max) noexcept;

  /// Construct a bounded range.
  /// @param _bounds A pair of min, max values.
  Range(const std::pair<T, T>& _bounds) noexcept;

  ///@}
  ///@name Queries
  ///@{

  /// Compute the length of this range.
  T Length() const noexcept;

  /// Compute the center of this range.
  /// @warning This will not be exact for integral types.
  T Center() const noexcept;

  /// Test if a value is inside this range.
  /// @param _val The value to test.
  /// @param _inclusive Count values on the boundary as inside?
  /// @return True if the test value lies inside the range (or on the boundary
  ///         for inclusive test).
  template <typename U>
  bool Contains(const U& _val, const bool _inclusive = true) const noexcept;

  /// Test the clearance of a value. Clearance is defined as the minimum
  /// distance to a boundary value, and will be negative if the test value falls
  /// outside the range.
  /// @param _val The value to test.
  template <typename U>
  T Clearance(const U& _val) const noexcept;

  /// Sample the range for a random contained value with uniform probability.
  T Sample() const noexcept;

  ///@}
  ///@name Modifiers
  ///@{

  /// Resize the range.
  /// @param _min The new lower bound.
  /// @param _max The new upper bound.
  void Resize(const T _min, const T _max) noexcept;

  ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename T>
inline
Range<T>::
Range() noexcept :
    min(T(0)), max(T(0)) { }


template <typename T>
inline
Range<T>::
Range(const T _min, const T _max) noexcept :
    min(_min), max(_max) { }


template <typename T>
inline
Range<T>::
Range(const std::pair<T, T>& _bounds) noexcept :
    min(_bounds.first), max(_bounds.second) { }

/*-------------------------------- Queries -----------------------------------*/

template <typename T>
inline
T
Range<T>::
Length() const noexcept {
  return max - min;
}


template <typename T>
inline
T
Range<T>::
Center() const noexcept {
  return Length() / T(2);
}


template <typename T>
template <typename U>
inline
bool
Range<T>::
Contains(const U& _val, const bool _inclusive) const noexcept {
  return InRange(_val, min, max, _inclusive);
}


template <typename T>
template <typename U>
inline
T
Range<T>::
Clearance(const U& _val) const noexcept {
  static_assert(!std::is_unsigned<T>::value, "Can't compute clearance for "
      "unsigned ranges as the return would be negative if the test value "
      "lies outside the range.");
  return std::min(_val - min, max - _val);
}


template <typename T>
inline
T
Range<T>::
Sample() const noexcept {
  return SampleRange(min, max);
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename T>
inline
void
Range<T>::
Resize(const T _min, const T _max) noexcept {
  min = _min;
  max = _max;
}

/*---------------------------------- I/O -------------------------------------*/

template <typename T>
std::ostream&
operator<<(std::ostream& _os, const Range<T>& _r) {
  return _os << _r.min << ":" << _r.max;
}


template <typename T>
std::istream&
operator>>(std::istream& _is, Range<T>& _r) {
  char delim;
  return _is >> _r.min >> std::skipws >> delim >> _r.max;
}

/*----------------------------------------------------------------------------*/

#endif
