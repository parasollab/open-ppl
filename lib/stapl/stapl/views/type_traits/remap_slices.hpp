/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_REMAP_SLICES_HPP
#define STAPL_VIEWS_TYPE_TRAITS_REMAP_SLICES_HPP

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/tuple/from_index.hpp>
#include <stapl/utility/tuple/to_index.hpp>
#include <stapl/utility/tuple/difference.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/utility/tuple/discard.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to recursively call
/// @ref tuple_ops::heterogeneous_discard
///
/// Recursive case
//////////////////////////////////////////////////////////////////////
template<int CurrentIndex, int SlicesSize, typename Slices, typename Tuple>
struct discard_from_sequence_impl
{
  using type = typename discard_from_sequence_impl<
    CurrentIndex+1, SlicesSize, Slices,
    typename tuple_ops::result_of::heterogeneous_discard<
      typename tuple_ops::from_index_sequence<
        index_sequence<
          tuple_element<CurrentIndex, Slices>::type::value - CurrentIndex
        >
      >::type,
      Tuple
    >::type
  >::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to recursively call
/// @ref tuple_ops::heterogeneous_discard
///
/// Base case.
//////////////////////////////////////////////////////////////////////
template<int SlicesSize, typename Slices, typename Tuple>
struct discard_from_sequence_impl<SlicesSize, SlicesSize, Slices, Tuple>
{
  using type = Tuple;
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to call @ref tuple_ops::heterogeneous_discard
/// multiple times based on Slices from an index_sequence.
///
/// @tparam Slices An index_sequence of slices
/// @tparam Tuple Tuple from which to discard
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Tuple>
struct discard_from_sequence;

template<std::size_t... Indices, typename Tuple>
struct discard_from_sequence<index_sequence<Indices...>, Tuple>
{
  using slices_tuple = tuple<integral_constant<std::size_t, Indices>...>;
  using type = typename discard_from_sequence_impl<
    0, sizeof...(Indices), slices_tuple, Tuple
  >::type;
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction used when creating nested deep slices views
/// to remap the slices received in a .slice<> operation.
///
/// For example, consider a 5D view (v) with a slices_view<0,2,4> on top of it
/// (sv). If this slices view receives a slice<0,2> call, then the new slices
/// view that is created would need to remap these new slices (NewSlices)
/// based on the old slices (OuterSlices) and the original view's dimension.
///
/// slices_view<0,2,4>.slice<0,2>(my_5d_view);
///
/// In this case, the new slices <0,2> will be remapped into <1>.
///
/// This is because <0,2> refers to <0,4> in sv, leaving the middle element
/// (<2>) to be free. Since the original view's 5D space is really <0,1,2,3,4>,
/// we drop <0, 4>, leaving <1,2,3>. Now we find where the remaining element (2)
/// is in this list. Its position is 1, therefore, the remapped slices will be
/// <1>.
///
/// @tparam OuterSlices An @ref index_sequence of slices from the original
///   slices view.
/// @tparam NewSlices An @ref index_sequence of slices of slices that is being
///   requested
/// @tparam D The dimensionality of the view beneath the original slices view.
//////////////////////////////////////////////////////////////////////
template<typename OuterSlices, typename NewSlices, int D>
struct remap_slices
{
  using new_as_tuple = typename tuple_ops::from_index_sequence<NewSlices>::type;
  using outer_as_tuple =
    typename tuple_ops::from_index_sequence<OuterSlices>::type;

  // Choose the correct dimensions by using the new slices as indices into the
  // original slices tuple
  using projected = typename tuple_ops::result_of::heterogeneous_filter<
    new_as_tuple, outer_as_tuple
  >::type;

  // Find the indices that are left free
  using diff =
    typename tuple_ops::result_of::difference<outer_as_tuple, projected>::type;

  // Find the indicies in the bottom-most view that are left when we discard
  // the non-free indices
  using remaining = typename detail::discard_from_sequence<
    typename tuple_ops::to_index_sequence<projected>::type,
    typename tuple_ops::from_index_sequence<make_index_sequence<D>>::type
  >::type;

  // Diff contains values, we need to find their positions in the remaining
  // tuple
  using type = typename find_first_indices<diff, remaining>::type;
};

} // namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_REMAP_SLICES_HPP
