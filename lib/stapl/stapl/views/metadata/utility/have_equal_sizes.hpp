/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_UTILITY_HAVE_EQUAL_SIZES_HPP
#define STAPL_VIEWS_METADATA_UTILITY_HAVE_EQUAL_SIZES_HPP

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/utility/vs_map.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Unary functor comparing member to input parameter's size method.
///   True if values (representing sizes) are the same or the parameter's size
///   is tagged as infinite.
//////////////////////////////////////////////////////////////////////
template<typename T>
class compare_size
{
private:
  using size_type = typename T::size_type;

  const size_type m_sz;

public:
  typedef bool result_type;

  compare_size(T const& md)
    : m_sz(md.size())
  { }

  template<typename Partition>
  bool operator()(Partition const& p) const
  {
    const size_type partition_size = p.size();

    return partition_size == m_sz
           || partition_size == index_bounds<size_type>::highest();
  }
}; // class compare_size


//////////////////////////////////////////////////////////////////////
/// @brief Unary functor comparing member to input parameter's size method.
///   True if values (representing sizes) are the same or the parameter's size
///   is tagged as infinite.
///
/// Specialization to handle case where metadata containers are passed with
/// a boolean indicating if they are fixed size.
//////////////////////////////////////////////////////////////////////
template<typename T>
class compare_size<std::pair<bool, T>>
{
private:
  using size_type = typename T::size_type;

  const size_type m_sz;

public:
  typedef bool result_type;

  compare_size(std::pair<bool, T> const& md)
    : m_sz(md.second.size())
  { }

  template<typename Partition>
  bool operator()(Partition const& p) const
  {
    const size_type partition_size = p.second.size();

    return partition_size == m_sz
           || partition_size == index_bounds<size_type>::highest();
  }
}; // class compare_size

//////////////////////////////////////////////////////////////////////
/// @brief Unary function that returns the boolean from the pair representing
///   the metadata of a view indicating if the metadata container is a
///   fixed-size balanced distributed container
//////////////////////////////////////////////////////////////////////
class check_static_metadata
{
public:
  typedef bool result_type;

  template<typename Partition>
  bool operator()(Partition const& p) const
  { return p.first; }
}; // class check_static_metadata

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Compute whether or not a set of views have the same size.
///
/// @param md Tuple of views
/// @tparam Guide Index in the tuple with which to compare sizes
//////////////////////////////////////////////////////////////////////
template<int Guide = 0, typename Containers>
bool have_equal_sizes(Containers const& md)
{
  return vs_map_reduce(
    detail::compare_size<
      typename stapl::tuple_element<Guide, Containers>::type
    >(get<Guide>(md)),
    std::logical_and<bool>(), true, md
  );
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a boolean indicating if all metadata containers are static
//////////////////////////////////////////////////////////////////////
template<int Guide = 0, typename Containers>
bool have_static_metadata(Containers const& md)
{
  return vs_map_reduce(
    detail::check_static_metadata(),
    std::logical_and<bool>(), true, md
  );
}

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_UTILITY_HAVE_EQUAL_SIZES_HPP
