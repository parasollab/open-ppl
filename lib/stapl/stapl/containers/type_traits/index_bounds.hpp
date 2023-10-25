/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_INDEX_BOUNDS_HPP
#define STAPL_CONTAINERS_INDEX_BOUNDS_HPP

#include <boost/numeric/conversion/bounds.hpp>
#include <boost/limits.hpp>
#include <string>
#include <utility>
#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class to determine the bounds (min, max, error) of an index
/// or GID.
///
/// @tparam T The index type. Typically a container GID or view index_type.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct index_bounds
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the lowest value for an index. If the index is integral,
  /// this is equivalent to @ref std::numeric_limits<T>::min(). If T is a float,
  /// this will be @p -std::numeric_limits<T>::max().
  //////////////////////////////////////////////////////////////////////
  static T lowest(void)
  { return boost::numeric::bounds<T>::lowest(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the highest value for an index.
  /// This is equivalent to @ref std::numeric_limits<T>::max().
  //////////////////////////////////////////////////////////////////////
  static T highest(void)
  { return boost::numeric::bounds<T>::highest(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the smallest positive normalized value for an index.
  /// If the index is integral, this will be 0.
  //////////////////////////////////////////////////////////////////////
  static T smallest(void)
  { return boost::numeric::bounds<T>::smallest(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a designated "invalid" value for a GID.
  //////////////////////////////////////////////////////////////////////
  static T invalid(void)
  { return stapl::index_bounds<T>::highest(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when the index or GID is a @ref std::string.
///
/// @see index_bounds
//////////////////////////////////////////////////////////////////////
template<>
struct index_bounds<std::string>
{
  static std::string invalid(void)
  { return std::string("STAPL_INVALID_STRING_GID_MUST_BE_UNIQUE_KEY"); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when the index or GID is a @ref std::pair of the
/// same type.
///
/// @see index_bounds
//////////////////////////////////////////////////////////////////////
template<typename T>
struct index_bounds<std::pair<T,T> >
{
  static std::pair<T,T> invalid(void)
  {
    return std::make_pair(index_bounds<T>::highest(),
                          index_bounds<T>::highest());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when the index or GID is a @ref std::pair of
/// different types.
///
/// @see index_bounds
//////////////////////////////////////////////////////////////////////
template<typename... T>
struct index_bounds<stapl::tuple<T...> >
{
  static stapl::tuple<T...> lowest(void)
  {
    return stapl::make_tuple(index_bounds<T>::lowest()...);
  }

  static stapl::tuple<T...> highest(void)
  {
    return stapl::make_tuple(index_bounds<T>::highest()...);
  }

  static stapl::tuple<T...> invalid(void)
  {
    return stapl::make_tuple(index_bounds<T>::highest()...);
  }
};

} // namespace stapl

#endif // STAPL_CONTAINERS_INDEX_BOUNDS_HPP
