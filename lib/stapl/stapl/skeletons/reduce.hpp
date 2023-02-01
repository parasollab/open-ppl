/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_REDUCE_HPP
#define STAPL_SKELETONS_REDUCE_HPP

#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/reduce.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>
#include <stapl/skeletons/functional/sink_value.hpp>
#include <stapl/skeletons/transformations/coarse/reduce.hpp>
#include <stapl/views/metadata/coarseners/multiview.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace reduce_helpers {

//////////////////////////////////////////////////////////////////////
/// @brief Reduces the value of elements in a given view by applying
/// @c BinaryOp given an algorithm executor and the skeleton to be used.
///
/// @tparam T       result type of the reduction
/// @tparam C       data coarsening method to be used
/// @param skeleton the reduction skeleton to be used
/// @param view     the input view
/// @param binop    the operation used to reduce the values
///
/// @return returns the reduction value on each location
///
/// @todo the extraneous creation of an array with one element per
///       location should be replaced by the direct results from the
///       PARAGRAPH
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename C, typename S, typename View>
typename std::decay<View>::type::value_type
reduce(S const& skeleton, View&& view)
{
  using namespace skeletons;

  return skeletons::execute(
           execution_params<typename df_stored_type<T>::type>(C()),
           compose(skeleton, broadcast_to_locs<true>()),
           std::forward<View>(view));
}

//////////////////////////////////////////////////////////////////////
/// @brief Reduces the value of elements in a given view by applying
/// @c BinaryOp.
///
/// @param view  the input view
/// @param binop the operation used to reduce the values
///
/// @return returns the reduction value on each location
///
/// @todo the extraneous creation of an array with one element per
///       location should be replaced by the direct results from the
///       PARAGRAPH
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename View, typename BinaryOp>
typename std::decay<View>::type::value_type
reduce(stapl::use_default, View&& view, BinaryOp const& binop)
{
  typedef typename
    boost::result_of<
      BinaryOp(typename std::decay<View>::type::reference,
               typename std::decay<View>::type::reference)
    >::type val_t;

  using namespace skeletons;
  return reduce_helpers::reduce<val_t, default_coarsener>(
           skeletons::coarse(skeletons::reduce(binop)),
           std::forward<View>(view));
}

//////////////////////////////////////////////////////////////////////
/// @brief This specialization is used when no coarsening is intended.
///
/// @param view  the input view
/// @param binop the operation used to reduce the values
///
/// @return returns the reduction value on each location
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename View, typename BinaryOp>
typename std::decay<View>::type::value_type
reduce(skeletons::tags::no_coarsening, View&& view, BinaryOp const& binop)
{
  typedef typename
    boost::result_of<
      BinaryOp(typename std::decay<View>::type::reference,
               typename std::decay<View>::type::reference)
    >::type val_t;

  return reduce_helpers::reduce<val_t, null_coarsener>(
           skeletons::reduce(binop),
           std::forward<View>(view));
}

}

//////////////////////////////////////////////////////////////////////
/// @brief Users can specify which version of the reduce skeleton to
/// be used for reduction algorithm. The possible choices are
/// @li stapl::use_default which uses the default coarsened reduction
///     skeleton.
/// @li no_coarsening In some cases it is desired to use a reduction
///     in its fine-grained format. One can use this tag to avoid
///     the coarsening phase altogether.
///
/// @param view  the input view
/// @param binop the operation used to reduce the values
/// @tparam Tag which reduction to be used
///
/// @return returns the reduction value on each location
///
/// @see algorithm_fwd.hpp for default values.
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template <typename Tag = stapl::use_default,
          typename View, typename BinaryOp>
typename std::decay<View>::type::value_type
reduce(View&& view, BinaryOp const& binop)
{
  return reduce_helpers::reduce(Tag(), view, binop);
}

} // namespace stapl

#endif // STAPL_SKELETONS_REDUCE_HPP
