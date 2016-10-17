#ifndef RANGE_TYPE_H_
#define RANGE_TYPE_H_

#include <limits>

////////////////////////////////////////////////////////////////////////////////
/// A range of numeric values.
////////////////////////////////////////////////////////////////////////////////
template <typename NumericType>
struct Range final {

  ///@name Local Types
  ///@{

  ///@}
  ///@name Internal State
  ///@{

  T min;  ///< The lower bound on this range.
  T max;  ///< The upper bound on this range.

  ///@}
  ///@name Construction
  ///@{

  //////////////////////////////////////////////////////////////////////////////
  /// Construct a range over all values of T.
  Range();

  //////////////////////////////////////////////////////////////////////////////
  /// Construct a bounded range.
  /// @param _min The lower bound.
  /// @param _max The upper bound.
  Range(const T _min, const T _max);

  ///@}
  ///@name Interface
  ///@{

  //////////////////////////////////////////////////////////////////////////////
  /// Test if a value is inside this range.
  /// @param _t The value to test.
  /// @param _inclusive Count values on the boundary as inside?
  /// @return True if the test value lies inside the range (or on the boundary
  ///         for inclusive test).
  const bool Inside(const T& _t, const bool _inclusive = true) const noexcept;

  //////////////////////////////////////////////////////////////////////////////
  /// Compute the length of this range.
  const T Length() const noexcept;

  ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename T>
Range<T>::
Range() :
    min(std::numeric_limits<T>::min()),
    max(std::numeric_limits<T>::max()) { }


template <typename T>
Range<T>::
Range(const T _min, const T _max) :
    min(_min), max(_max) { }

/*------------------------------- Interface ----------------------------------*/

template <typename T>
inline
const bool
Range<T>::
Inside(const T& _t, const bool _inclusive) const noexcept {
  return _inclusive ? min <= _t && _t <= max :
                      min <  _t && _t <= max ;
}


template <typename T>
inline
const T
Range<T>::
Length() const noexcept {
  return max - min;
}

/*----------------------------------------------------------------------------*/

#endif
