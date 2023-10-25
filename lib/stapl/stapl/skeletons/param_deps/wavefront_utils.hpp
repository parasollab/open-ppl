/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_WAVEFRONT_UTILS_HPP
#define STAPL_SKELETONS_PARAM_DEPS_WAVEFRONT_UTILS_HPP

#include <type_traits>
#include <map>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/utility/position.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>


namespace stapl {
namespace skeletons {
namespace wavefront_utils {


template <std::size_t i, std::size_t... Indices>
auto
wavefront_direction_impl(
  std::array<skeletons::position, i> const& start_corner,
  index_sequence<Indices...>&&) noexcept
STAPL_AUTO_RETURN((
  stapl::make_tuple((
    start_corner[Indices] == skeletons::position::first ? -1 : 1)...)
))

//////////////////////////////////////////////////////////////////////
/// @brief Determines the direction along each axis of a wavefront
//////////////////////////////////////////////////////////////////////
template <std::size_t i>
auto
wavefront_direction(
  std::array<skeletons::position, i> const& start_corner)
STAPL_AUTO_RETURN((
  wavefront_direction_impl(start_corner, make_index_sequence<i>())
))

//////////////////////////////////////////////////////////////////////
/// @brief Determines the corners of this wavefront parametric dependency.
//////////////////////////////////////////////////////////////////////
template <std::size_t i, typename Coord, std::size_t... Indices>
auto
first_index_impl(
  std::array<skeletons::position, i> const& start_corner,
  Coord const& skeleton_size,
  index_sequence<Indices...>&&) noexcept
STAPL_AUTO_RETURN((
  stapl::make_tuple((start_corner[Indices] == skeletons::position::first ?
                     0 : std::get<Indices>(skeleton_size)-1)...)
))

//////////////////////////////////////////////////////////////////////
/// @brief Determines the start corners of a wavefront
//////////////////////////////////////////////////////////////////////
template <std::size_t i, typename Coord>
auto
first_index(
  std::array<skeletons::position, i> const& start_corner,
  Coord const& skeleton_size) noexcept
STAPL_AUTO_RETURN((
  first_index_impl(start_corner, skeleton_size, make_index_sequence<i>())
))

//////////////////////////////////////////////////////////////////////
/// @brief Determines the end corners of a wavefront
//////////////////////////////////////////////////////////////////////
template <std::size_t i, typename Coord, std::size_t... Indices>
auto
last_index_impl(
  std::array<skeletons::position, i> const& start_corner,
  Coord const& skeleton_size,
  index_sequence<Indices...>&&) noexcept
STAPL_AUTO_RETURN((
  stapl::make_tuple((start_corner[Indices] == skeletons::position::last ?
                   0 : std::get<Indices>(skeleton_size)-1)...)
))

//////////////////////////////////////////////////////////////////////
/// @brief Determines the corners of this wavefront parametric dependency.
//////////////////////////////////////////////////////////////////////
template <std::size_t i, typename Coord>
auto
last_index(
  std::array<skeletons::position, i> const& start_corner,
  Coord const& skeleton_size) noexcept
STAPL_AUTO_RETURN((
  last_index_impl(start_corner, skeleton_size, make_index_sequence<i>())
))

} // namespace wavefront_utils

namespace wavefront_utils {

//////////////////////////////////////////////////////////////////////
/// @brief A filter operation used in the coarsened wavefront operation.
///        when the output of coarsened tasks is not only boundary planes.
///
/// @tparam dims   the dimensionality of the given wavefront.
/// @tparam Filter the filter should be applied to each element of view.
//////////////////////////////////////////////////////////////////////
template <int dims, typename Filter>
struct wavefront_filter
{
private:
  using corner_t    = std::array<skeletons::position, dims>;
  using dir_t       = skeletons::direction;
  corner_t m_corner;
  dir_t    m_direction;
  Filter   m_filter;

public:
  template <typename F>
  struct result;

  template <typename V>
  struct result<wavefront_filter(V)>
  {
    using value_t    = typename std::decay<V>::type::value_type;
    using filtered_t = typename filters::result_of<Filter,value_t>::type;
    using type       = lightweight_multiarray<filtered_t, dims>;
  };

  wavefront_filter(corner_t const& corner, Filter const& filter)
    : m_corner(corner),
      m_direction(dir_t::direction0),
      m_filter(filter)
  { }

  void set_direction(dir_t direction)
  {
    m_direction = direction;
    filters::set_direction(m_filter, m_direction);
  }

  template <typename Dir, typename... Args>
  int configure_filter(Dir&& dir, Args&&...)
  {
    this->set_direction(dir);
    return 0;
  }


private:
  //////////////////////////////////////////////////////////////////////
  /// @brief The specialization of the filter for the 2D wavefront.
  ///
  /// @param v the input to be filtered
  ///
  /// @return the filtered portion of the input
  //////////////////////////////////////////////////////////////////////
  template <typename V>
  typename result<wavefront_filter(V)>::type
  apply(V&& v, std::integral_constant<int, 2>) const
  {
    auto&& dimensions = v.domain().dimensions();
    std::size_t dims0, dims1;
    std::tie(dims0, dims1) = dimensions;

    using namespace wavefront_utils;
    std::size_t end_i, end_j;
    std::tie(end_i, end_j) =  last_index(m_corner, dimensions);

    // Calculating the output result dimensions. For example, in a wavefront
    // starting from the top left corner, on direction0 the dimensions are
    // (1, dims1) and on direction1 they are (dims0, 1)
    //   ___________
    //  |         | |
    //  |         | | -> direction1
    //  |_________|_|
    //  |_________|_|
    //       |
    //       v
    //   direction0
    //
    std::size_t result_dim0 = (m_direction == dir_t::direction0 ? 1 : dims0);
    std::size_t result_dim1 = (m_direction == dir_t::direction1 ? 1 : dims1);

    std::size_t start_offset_i = (m_direction == dir_t::direction0 ? end_i : 0);
    std::size_t start_offset_j = (m_direction == dir_t::direction1 ? end_j : 0);

    using result_type = typename result<wavefront_filter(V)>::type;
    result_type res(make_tuple(result_dim0, result_dim1));

    using filters::apply_filter;

    // NOTE - end indices are inclusive
    for (std::size_t i = 0; i < result_dim0; ++i) {
      for (std::size_t j = 0; j < result_dim1; ++j) {

        res(i, j) = apply_filter(m_filter,
                                 v(start_offset_i + i, start_offset_j + j));
      }
    }
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The specialization of the filter for the 3D wavefront.
  ///
  /// @param v the input to be filtered
  ///
  /// @return the filtered portion of the input
  //////////////////////////////////////////////////////////////////////
  template <typename V>
  typename result<wavefront_filter(V)>::type
  apply(V&& v, std::integral_constant<int, 3>) const
  {
    auto&& dimensions = v.domain().dimensions();
    std::size_t dims0, dims1, dims2;
    std::tie(dims0, dims1, dims2) = dimensions;

    using namespace wavefront_utils;
    std::size_t end_i, end_j, end_k;
    std::tie(end_i, end_j, end_k) =  last_index(m_corner, dimensions);

    // Calculating the output result dimensions
    std::size_t result_dim0 = (m_direction == dir_t::direction0 ? 1 : dims0);
    std::size_t result_dim1 = (m_direction == dir_t::direction1 ? 1 : dims1);
    std::size_t result_dim2 = (m_direction == dir_t::direction2 ? 1 : dims2);

    std::size_t start_offset_i = (m_direction == dir_t::direction0 ? end_i : 0);
    std::size_t start_offset_j = (m_direction == dir_t::direction1 ? end_j : 0);
    std::size_t start_offset_k = (m_direction == dir_t::direction2 ? end_k : 0);

    using result_type = typename result<wavefront_filter(V)>::type;
    result_type res(make_tuple(result_dim0, result_dim1, result_dim2));

    using filters::apply_filter;

    // NOTE - end indices are inclusive
    for (std::size_t i = 0; i < result_dim0; ++i) {
      for (std::size_t j = 0; j < result_dim1; ++j) {
        for (std::size_t k = 0; k < result_dim2; ++k) {
          res(i, j, k) = apply_filter(m_filter,
                                      v(start_offset_i + i,
                                        start_offset_j + j,
                                        start_offset_k + k));
        }
      }
    }
    return res;
  }

public:
  template <typename V>
  typename result<wavefront_filter(V)>::type
  operator()(V&& v) const
  {
    return apply(std::forward<V>(v), std::integral_constant<int, dims>());
  }

  bool operator==(wavefront_filter const& other) const
  {
    return m_direction == other.m_direction and
           m_corner == other.m_corner;
  }

  Filter should_flow_filter() const
  {
    return m_filter;
  }

  void define_type(typer& t)
  {
    t.member(m_direction);
    t.member(m_corner);
    t.member(m_filter);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief A filter operation used in the coarsened wavefront operation
///        when the output of the wavefront coarsened tasks is represented
///        as an array of planes.
///
/// For example for 2D wavefornt, the output of the coarsened task is
/// std::array<plane_type, 2>.
///
/// @tparam dims   the dimensionality of the given wavefront.
/// @tparam Filter the filter should be applied to each element of view.
//////////////////////////////////////////////////////////////////////
template <int dims, typename Filter>
struct wavefront_filter_v2
{
private:
  using corner_t    = std::array<skeletons::position, dims>;
  using dir_t       = skeletons::direction;
  corner_t m_corner;
  dir_t    m_direction;
  Filter   m_filter;

public:
  template <typename F>
  struct result;

  template <typename V>
  struct result<wavefront_filter_v2(V)>
  {
    using type    = typename std::decay<V>::type::value_type;
  };

  wavefront_filter_v2(corner_t const& corner, Filter const& filter)
    : m_corner(corner),
      m_direction(dir_t::direction0),
      m_filter(filter)
  { }

  void set_direction(dir_t direction)
  {
    m_direction = direction;
    filters::set_direction(m_filter, m_direction);
  }

  template <typename Dir, typename... Args>
  int configure_filter(Dir&& dir, Args&&...)
  {
    this->set_direction(dir);
    return 0;
  }


private:
  //////////////////////////////////////////////////////////////////////
  /// @brief The specialization of the filter for the 2D wavefront.
  ///
  /// @param v the input to be filtered
  ///
  /// @return the filtered portion of the input
  //////////////////////////////////////////////////////////////////////
  template <typename V>
  typename result<wavefront_filter_v2(V)>::type
  apply(V&& v, std::integral_constant<int, 2>) const
  {
    return v[(size_t)m_direction];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The specialization of the filter for the 3D wavefront.
  ///
  /// @param v the input to be filtered
  ///
  /// @return the filtered portion of the input
  //////////////////////////////////////////////////////////////////////
  template <typename V>
  typename result<wavefront_filter_v2(V)>::type
  apply(V&& v, std::integral_constant<int, 3>) const
  {
    return v[(size_t)m_direction];
  }

public:
  template <typename V>
  typename result<wavefront_filter_v2(V)>::type
  operator()(V&& v) const
  {
    return apply(std::forward<V>(v), std::integral_constant<int, dims>());
  }

  bool operator==(wavefront_filter_v2 const& other) const
  {
    return m_direction == other.m_direction and
           m_corner == other.m_corner;
  }

  Filter should_flow_filter() const
  {
    return m_filter;
  }

  void define_type(typer& t)
  {
    t.member(m_direction);
    t.member(m_corner);
    t.member(m_filter);
  }
};


} // namespace wavefront_utils


//////////////////////////////////////////////////////////////////////
/// @brief Increases the size of @wavefront corner with the position::first
///        to @c nested_dims. It is used in nested execution of skeletons
///        when each level has a different dimensionality.
///
/// @tparam nested_dims Maximum of dimensionality between all
///         the nested levels
/// @tparam dims actual size of the corner
//////////////////////////////////////////////////////////////////////
template <int nested_dims, int dims>
struct pad_corner
{
  template <typename Corner>
  static std::array<Corner, nested_dims>
  apply(std::array<Corner, dims> const& arr)
  {
    std::array<Corner, nested_dims> res;

    constexpr int diff = nested_dims - dims;

    auto f = position::first;

    for (size_t i = 0; i < diff; ++i)
      res[i] = f;

    for (size_t i = 0; i < dims; ++i)
      res[i + diff] = arr[i];

    return res;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief When nested_dims and dims are same just returns the corner.
///
/// @tparam dims actual size of the corner
//////////////////////////////////////////////////////////////////////
template <int dims>
struct pad_corner<dims, dims>
{
  template <typename Corner>
  static std::array<Corner, dims>
  apply(std::array<Corner, dims> const& arr)
  {
    return arr;
  }
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_WAVEFRONT_UTILS_hpp
