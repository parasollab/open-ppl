#ifndef NONSTD_NUMERICS_H_
#define NONSTD_NUMERICS_H_

#include <ctgmath>
#include <limits>

namespace nonstd {

  ///@name Numerics
  ///@{

  /// Test the signum of a number.
  /// @param[in] _f The value to check.
  /// @return       +1 if _f is positive, -1 if it is negative, or 0 otherwise.
  template<typename numeric_type>
  inline short
  sign(const numeric_type _f)
  {
    static constexpr numeric_type zero{0};
    return (zero < _f) - (_f < zero);
  }


  /// Compare numeric values for equality within a tolerance.
  /// @param[in] _n1  The first value.
  /// @param[in] _n2  The second value.
  /// @param[in] _tol The maximum allowed difference between _n1 and _n2.
  /// @return         A bool indicating whether the difference between _n1 and
  ///                 _n2 is within the tolerance.
  template <typename numeric_type>
  inline bool
  approx(const numeric_type& _n1, const numeric_type& _n2,
      const numeric_type& _tol =
          std::numeric_limits<numeric_type>::epsilon() * numeric_type(10))
  {
    return std::abs(_n1 - _n2) <= _tol;
  }


  /// Test if a value lies within a given range, inclusive.
  /// @param[in] _n   The test value.
  /// @param[in] _min The lower bound.
  /// @param[in] _max The upper bound.
  /// @param[in] _inclusive  Toggle inclusive/exclusive.
  /// @return         A bool indicating whether the test value lies between _min
  ///                 and _max.
  template <typename numeric_type>
  inline bool
  in_bounds(const numeric_type& _n, const numeric_type& _min,
      const numeric_type& _max, const bool _inclusive = true)
  {
    return _inclusive ? (_min <= _n && _n <= _max) : (_min < _n && _n < _max);
  }


  /// Test if a value lies within a given range, with tolerance.
  /// @param[in] _n   The test value.
  /// @param[in] _min The lower bound.
  /// @param[in] _max The upper bound.
  /// @param[in] _tol The allowed tolerance.
  /// @return         A bool indicating whether the test value lies between _min
  ///                 and _max.
  template <typename numeric_type>
  inline bool
  approx_in_bounds(const numeric_type& _n, const numeric_type& _min,
      const numeric_type& _max,
      const numeric_type& _tol =
          std::numeric_limits<numeric_type>::epsilon() * numeric_type(10))
  {
    return (_min - _tol) <= _n && _n <= (_max + _tol);
  }


  /// Rescale a number _x from the range [_min0, _max0] to [_min1, _max1].
  /// @param[in] _x The number to scale.
  /// @param[in] _min0 The minimum of the original range.
  /// @param[in] _max0 The maximum of the original range.
  /// @param[in] _min1 The minimum of the new range.
  /// @param[in] _max1 The maximum of the new range.
  /// @return The rescaled version of _x.
  template <typename numeric_type>
  inline double
  rescale(const numeric_type& _x,
      const numeric_type& _min0, const numeric_type& _max0,
      const numeric_type& _min1 = numeric_type(-1),
      const numeric_type& _max1 = numeric_type(1))
  {
    return static_cast<double>(_max1 - _min1) * (_x - _min0) / (_max0 - _min0) +
        _min1;
  }


  /// Return the factorial of an unsigned integer.
  /// @param[in] _n The integer, which must be less than or equal to 20.
  /// @return       The factorial of _n.
  size_t factorial(const size_t _n) noexcept;


  /// Compute a logarithm of arbitrary base.
  /// @param[in] _x The operand.
  /// @param[in] _b The logarithm base.
  /// @return       The log base _b of _x.
  inline double
  log_base(const double _x, const double _b) noexcept
  {
    return std::log2(_x) / std::log2(_b);
  }


  /// Compute the sigmoid function of a floating-point number.
  /// @param[in] _x The operand.
  /// @return       The sigmoid value of _x.
  inline double
  sigmoid(const double _x) noexcept
  {
    return 1. / (1. + std::exp(-_x));
  }

  ///@}

}

#endif
