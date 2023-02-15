#ifndef BASIC_H_
#define BASIC_H_

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <limits>

namespace mathtool {

  // PI to 100 significant figures
#ifndef PI
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117068
#endif

#ifndef TWOPI
#define TWOPI (2 * PI)
#endif

  template<typename T>
  inline T sqr(const T& x) {return x * x;} ///< Return the square of x.

  inline int sign(double x) {return x >= 0 ? 1 : -1;} ///< Return the sign of x.

  ///@name Angle conversions
  ///@{

  inline double degToRad(double x) {return x * PI / 180;}

  inline double radToDeg(double x) {return x * 180 / PI;}

  ///@}
  ///@name Degree Trig functions
  ///@{

  inline double sind(double x) {return std::sin(degToRad(x));}

  inline double cosd(double x) {return std::cos(degToRad(x));}

  inline double tand(double x) {return std::tan(degToRad(x));}

  inline double asind(double x) {return radToDeg(std::asin(x));}

  inline double acosd(double x) {return radToDeg(std::acos(x));}

  inline double atand(double x) {return radToDeg(std::atan(x));}

  inline double atan2d(double x, double y) {return radToDeg(std::atan2(x,y));}

  ///@}
  ///@name Approximate Equality
  ///@{

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Check if two numeric values are within a tolerance of each other.
  /// @param[in] _a The first value.
  /// @param[in] _b The second value.
  /// @param[in] _tolerance The tolerance to use (10 * eps by default).
  /// @return Is _a within _tolerance of _b?
  template <typename NumericType>
  inline const bool approx(const NumericType _a, const NumericType _b,
      const NumericType _tolerance =
      10 * std::numeric_limits<NumericType>::epsilon()) {
    return std::fabs(_a - _b) <= _tolerance;
  }

  ///@}
}

#endif
