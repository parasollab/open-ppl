/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_LOCATION_RANGE_HPP
#define STAPL_RUNTIME_LOCATION_RANGE_HPP

#include "config.hpp"
#include "exception.hpp"
#include "serialization_fwd.hpp"
#include "utility/algorithm.hpp"
#include <iterator>
#include <type_traits>
#include <vector>
#include <boost/range/size.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Range of location ids.
///
/// @tparam T Range container type.
///
/// @related location_range()
/// @see location_range
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class location_range_wrapper
{
public:
  using size_type      = typename T::size_type;
  using const_iterator = typename T::const_iterator;
  using value_type     = runtime::location_id;

private:
  T m_t;

public:
  explicit location_range_wrapper(T t)
  : m_t(std::move(t))
  {
    STAPL_RUNTIME_CHECK(!empty(), "Range is empty.");
    STAPL_RUNTIME_ASSERT_MSG(runtime::all_unique(m_t),
                             "Range contains duplicates.");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if any of the location ids in the range exceeds
  ///        @p max.
  //////////////////////////////////////////////////////////////////////
  bool has_invalid(const value_type max) const noexcept
  {
    return std::find_if(begin(), end(),
                        [=](const value_type id) { return (id>=max); })!=end();
  }

  bool empty(void) const noexcept
  { return m_t.empty(); }

  size_type size(void) const noexcept
  { return boost::size(m_t); }

  T const& get(void) const noexcept
  { return m_t; }

  const_iterator begin(void) const noexcept
  { return m_t.begin(); }

  const_iterator end(void) const noexcept
  { return m_t.end(); }

  void define_type(typer& t)
  { t.member(m_t); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref location_range_wrapper for references to
///        containers.
///
/// @related location_range()
/// @see location_range
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class location_range_wrapper<T const&>
{
public:
  using size_type      = typename T::size_type;
  using const_iterator = typename T::const_iterator;
  using value_type     = runtime::location_id;

private:
  T const* m_t;

public:
  explicit location_range_wrapper(T const& t)
  : m_t(&t)
  {
    STAPL_RUNTIME_CHECK(!empty(), "Range is empty.");
    STAPL_RUNTIME_ASSERT_MSG(runtime::all_unique(*m_t),
                             "Range contains duplicates.");
  }

  bool has_invalid(const value_type max) const noexcept
  {
    return std::find_if(begin(), end(),
                        [=](const value_type id) { return (id>=max); })!=end();
  }

  bool empty(void) const noexcept
  { return m_t->empty(); }

  size_type size(void) const noexcept
  { return boost::size(*m_t); }

  T const& get(void) const noexcept
  { return *m_t; }

  const_iterator begin(void) const noexcept
  { return m_t->begin(); }

  const_iterator end(void) const noexcept
  { return m_t->end(); }

  void define_type(typer& t)
  { t.member(m_t); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a @ref location_range_wrapper from @p t.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename = typename std::enable_if<!std::is_reference<T>::value>::type>
location_range_wrapper<typename std::decay<T>::type> location_range(T&& t)
{
  return location_range_wrapper<typename std::decay<T>::type>{
           std::forward<T>(t)};
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a @ref location_range_wrapper from @p t.
///
/// @warning It assumes that @p t will be alive when the
///          @ref location_range_wrapper is used.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
location_range_wrapper<T const&> location_range(T& t)
{
  return location_range_wrapper<T const&>{t};
}

} // namespace stapl

#endif

