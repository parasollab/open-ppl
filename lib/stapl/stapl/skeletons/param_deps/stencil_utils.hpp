/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_STENCIL_UTILS_HPP
#define STAPL_SKELETONS_PARAM_DEPS_STENCIL_UTILS_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/nd_traits.hpp>

namespace stapl {
namespace skeletons {
namespace stencil_impl {

////////////////////////////////////////////////////////////////////////////////
/// @brief Represents a point read by a stencil characterized by a signed offset
/// in a given dimension.
////////////////////////////////////////////////////////////////////////////////
template<int Sign, size_t Dim, size_t Off>
struct point
{
  static constexpr int sign = Sign;
  static constexpr size_t dimension = Dim;
  static constexpr size_t offset = Off;
  static constexpr long int signed_offset = sign * offset;
};


template<int S, size_t D, class Offs>
struct expand_off;

// expand offsets, starting from 1
template<int S, size_t D, size_t... Offs>
struct expand_off<S,D,index_sequence<Offs...>>
{
  using type = tuple<point<S,D,Offs+1>...>;
};


template<int S, class Ds, class Offs>
struct expand_dim;

template<int S, size_t... Ds, class Offs>
struct expand_dim<S, index_sequence<Ds...>, Offs>
{
  using type = typename stapl::result_of::tuple_cat<
      typename expand_off<S, Ds, Offs>::type...
  >::type;
};


////////////////////////////////////////////////////////////////////////////////
/// @brief Computes the traversal order for the given dimensions and offsets
/// in each dimension as a tuple of @c point types excluding the center.
////////////////////////////////////////////////////////////////////////////////
template<typename Dims, typename Offs>
using traversal_order = typename stapl::result_of::tuple_cat<
  typename stencil_impl::expand_dim<-1, Dims, Offs>::type,
  typename stencil_impl::expand_dim<+1, Dims, Offs>::type
>::type;

template<typename Pt, size_t... Is>
using get_filter_tag = tags::direction<
  (Is == Pt::dimension ? Pt::sign : 0)...
>;

} // namespace stencil_impl

namespace stencil_utils {

using stencil_impl::point;

//////////////////////////////////////////////////////////////////////
/// @brief Returns a copy of the filter with the direction set based
/// on the tag.
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Filter>
Filter
get_stencil_filter(Filter f)
{
  f.set_direction(Tag());
  return f;
}

//////////////////////////////////////////////////////////////////////
/// @brief The special case that no filtering function is defined.
//////////////////////////////////////////////////////////////////////
template <typename Tag>
skeletons::no_filter
get_stencil_filter(skeletons::no_filter f)
{
  return f;
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a copy of the filter with the direction set based
/// on the point.
//////////////////////////////////////////////////////////////////////
template<typename Pt, typename Filter, size_t... Is>
Filter
get_stencil_filter(Pt, Filter f, index_sequence<Is...>)
{
  f.set_direction(stencil_impl::get_filter_tag<Pt, Is...>());
  return f;
}

template<typename Pt, size_t... Is>
skeletons::no_filter
get_stencil_filter(Pt, skeletons::no_filter f, index_sequence<Is...>)
{
  return f;
}

//check if the point is out-of-bounds on the left/negative side
template<typename Coord, typename Size, size_t Dim, size_t Off>
bool past_boundary(Coord const& c, Size const&, point<-1, Dim, Off>)
{
  return nd_get<Dim>(c) < Off;
}

//check if the point is out-of-bounds on the right/positive side
template<typename Coord, typename Size, size_t Dim, size_t Off>
bool past_boundary(Coord const& c, Size const& s, point<+1, Dim, Off>)
{
  return nd_get<Dim>(c) + Off >= s;
}

//compute a dependence for reading up to P from the left boundary in Dim
template<size_t P, size_t Dim, size_t Off, typename Index, size_t... Is>
Index left_boundary_index(Index const& idx,
  index_sequence<Is...>)
{
  return Index{
    Is != Dim ? nd_get<Is>(idx)
              : P + nd_get<Is>(idx) - Off...};
}

//compute a dependence for reading up to P from the right boundary in Dim
template<size_t Dim, size_t Off, typename Index, size_t... Is>
Index right_boundary_index(Index const& idx, size_t m,
  index_sequence<Is...>)
{
  return Index{
    Is != Dim ? nd_get<Is>(idx)
              : nd_get<Is>(idx) + Off - m...};
}

//compute a dependence specified by Pt and idx without boundary checks
template<typename Pt, typename Index, size_t... Is>
Index nonperiodic_dep(Pt, Index const& idx, index_sequence<Is...>)
{
  return Index{(Is != Pt::dimension
        ? nd_get<Is>(idx)
        : nd_get<Is>(idx) + Pt::signed_offset)...
      };
}




} // namespace stencil_utils
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_STENCIL_UTILS_hpp
