#ifndef NONSTD_CONTAINER_OPS_H_
#define NONSTD_CONTAINER_OPS_H_

#include <cmath>
#include <cstddef>
#include <iterator>
#include <type_traits>

#include "nonstd/io.h"
#include "nonstd/numerics.h"
#include "nonstd/runtime.h"


namespace nonstd {

  ///@name Container Operations
  ///@{
  /// Vector operations for STL-like containers.

  /// Compute the dot product of two containers.
  /// @param[in] _c1 The first container.
  /// @param[in] _c2 The second container.
  /// @return The dot product _c1 * _c2.
  template <typename T, typename C1, typename C2>
  T
  dot(const C1& _c1, const C2& _c2) noexcept;


  /// Return a unit-vector version of a container.
  /// @param[in] _c The container of interest.
  /// @return A modified copy of _c with vector norm equal to 1.
  template <typename Container>
  Container
  unit(const Container& _c) noexcept;


  /// Compute the squared vector magnitude of the elements in a container.
  /// @param[in] _c The container of interest.
  /// @return The squared vector magnitude of _c.
  template <typename T, typename Container>
  T
  magnitude_sqr(const Container& _c) noexcept;


  /// Compute the vector magnitude of the elements in a container.
  /// @param[in] _c The container of interest.
  /// @return The vector magnitude of _c.
  template <typename T, typename Container>
  T
  magnitude(const Container& _c) noexcept;

  ///@}
}


template <typename T, typename C1, typename C2>
inline
T
nonstd::
dot(const C1& _c1, const C2& _c2) noexcept
{
  auto i1 = _c1.begin();
  auto i2 = _c2.begin();

  T out(0);
  for(; i1 != _c1.end() && i2 != _c2.end(); ++i1, ++i2)
    out += (*i1) * (*i2);

  nonstd::assert_msg(i1 == _c1.end() && i2 == _c2.end(), "nonstd::dot error: "
      "container dot-product called on containers of differing size. "
      "|c1| = " + std::to_string(std::distance(_c1.begin(), _c1.end())) + ", " +
      "|c2| = " + std::to_string(std::distance(_c2.begin(), _c2.end())) + ".");

  return out;
}


template <typename Container>
inline
Container
nonstd::
unit(const Container& _c) noexcept
{
  using base_type = typename std::remove_const<
                      typename std::remove_reference<
                        decltype(*_c.begin())
                      >::type
                    >::type;

  // If the magnitude is nearly zero, just return the same vector instead of a
  // container full of nan.
  auto mag = nonstd::magnitude<base_type, Container>(_c);
  if(approx(mag, base_type(0)))
    return _c;

  Container out = _c;
  for(auto it = out.begin(); it != out.end(); ++it)
    (*it) /= mag;

  return out;
}


template <typename T, typename Container>
inline
T
nonstd::
magnitude_sqr(const Container& _c) noexcept
{
  T mag_sqr(0);
  for(auto it = _c.begin(); it != _c.end(); ++it)
    mag_sqr += (*it) * (*it);
  return mag_sqr;
}


template <typename T, typename Container>
inline
T
nonstd::
magnitude(const Container& _c) noexcept
{
  return std::sqrt(nonstd::magnitude_sqr<T>(_c));
}

#endif
