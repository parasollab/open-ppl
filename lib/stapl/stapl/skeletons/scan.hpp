/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SCAN_HPP
#define STAPL_SKELETONS_SCAN_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/scan.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/transformations/coarse/scan.hpp>
#include <stapl/algorithms/identity_value.hpp>

#include <stapl/views/metadata/coarseners/default.hpp>

namespace stapl {

namespace scan_helpers {

//////////////////////////////////////////////////////////////////////
/// @brief Computes the inclusive scan of the elements of the input
/// view and stores the result in the output view.
///
/// In an inclusive scan, each element in the result view is the
/// result of successive application of @c binop on all the elements
/// before it, including itself.
///
/// The type of the inclusive scan to be used can be specified by Tag
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan operation.
/// @param tag   determines the type of inclusive scan to be used
/// @tparam CoarseTag specializes the coarsened scan
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Binop, typename Tag,
         typename CoarseTag>
void scan(View0&& view0, View1&& view1, Binop binop,
          skeletons::tags::scan<Tag, skeletons::tags::inclusive>,
          CoarseTag)
{
  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::coarse<CoarseTag>(skeletons::scan<Tag>(binop)),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

//////////////////////////////////////////////////////////////////////
/// @brief A non-coarsened inclusive scan skips the coarsening process
/// and is faster for the cases that the number of elements per location
/// are too small to benefit from coarsening.
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan operation.
/// @param tag   determines the type of inclusive scan to be used
/// @see exclusive_scan
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Binop, typename Tag>
void scan(View0&& view0, View1&& view1, Binop binop,
          skeletons::tags::scan<Tag, skeletons::tags::inclusive>,
          skeletons::tags::no_coarsening)
{
  typedef typename std::decay<View0>::type::value_type val_t;
  skeletons::execute(
    skeletons::default_execution_params(),
    skeletons::sink<val_t>(skeletons::scan<Tag>(binop)),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the exclusive scan of the elements of the input
/// view and stores the result in the output view.
///
/// In an exclusive scan, each element in the result view is the
/// result of successive application of @c binop on all the elements
/// before it, not including itself.
///
/// The type of the exclusive scan to be used can be specified by Tag
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan operation.
/// @param tag   determines the type of exclusive scan to be used
/// @tparam CoarseTag specializes the coarsened scan
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Binop, typename Tag,
         typename CoarseTag>
void scan(View0&& view0, View1&& view1, Binop binop,
          skeletons::tags::scan<Tag, skeletons::tags::exclusive>,
          CoarseTag)
{
  typedef typename std::decay<View0>::type::value_type val_t;
  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::coarse<CoarseTag>(
      skeletons::scan<Tag>(
        binop, identity_value<Binop, val_t>::value())),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the exclusive scan of the elements of the input
/// view and stores the result in the output view.
///
/// In an exclusive scan, each element in the result view is the
/// result of successive application of @c binop on all the elements
/// before it, not including itself.
///
/// @note The naive coarsened exclusive scan is not an in-place algorithm.
/// The result of computation should be copied to the output explicitly
/// using the @c sink skeleton.
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan operation.
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Binop,
         typename Tag, typename ExecutionTag>
void scan(View0&& view0, View1&& view1, Binop binop,
          skeletons::tags::scan<Tag, skeletons::tags::exclusive>,
          skeletons::tags::coarse<skeletons::tags::naive, ExecutionTag>)
{
  using val_t = typename std::decay<View0>::type::value_type;
  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::sink<val_t>(
      skeletons::coarse<skeletons::tags::naive>(
        skeletons::scan<Tag>(
          binop, identity_value<Binop, val_t>::value()))),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

//////////////////////////////////////////////////////////////////////
/// @brief A non-coarsened exclusive scan skips the coarsening process
/// and is faster for the cases that the number of elements per location
/// are too small to benefit from coarsening.
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan operation.
/// @param tag   determines the type of scan to be used
/// @see exclusive_scan
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Binop, typename Tag>
void scan(View0&& view0, View1&& view1, Binop binop,
          skeletons::tags::scan<Tag, skeletons::tags::exclusive>,
          skeletons::tags::no_coarsening)
{
  using val_t = typename std::decay<View0>::type::value_type;
  skeletons::execute(
    skeletons::default_execution_params(),
    skeletons::sink<val_t>(
      skeletons::scan<Tag>(
        binop, identity_value<Binop, val_t>::value())
    ),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

}

//////////////////////////////////////////////////////////////////////
/// @brief A scan is an operation in which each element in the results
/// is the result of the cumulative operation on all elements before it.
/// It can be one of two types
/// @li exclusive in which the cumulative operation for the value at
///     index i of the output contains all elements before index i
///     in the input
/// @li inclusive in which the cumulative operation for element at index
///     i of the output contains all elements before index i and at
///     index i of the input.
///
/// The type of the scan to be used is determined by the scan tag
/// passed to this function. In addition, one can choose the coarsening
/// method used for the skeleton by specifying a @c CoarseTag
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan operation.
/// @tparam ScanTag   which scan algorithm to be used
/// @tparam CoarseTag what type of coarsening to be used (if any)
/// @see inclusive_scan
/// @see exclusive_scan
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename ScanTag, typename CoarseTag,
         typename View0, typename View1, typename Binop>
void scan(View0&& view0, View1&& view1, Binop binop)
{
  scan_helpers::scan(std::forward<View0>(view0), std::forward<View1>(view1),
                     binop, ScanTag(), CoarseTag());
}

//////////////////////////////////////////////////////////////////////
/// @brief A scan is an operation in which each element in the results
/// is the result of the cumulative operation on all elements before it.
/// It can be one of two types
/// @li exclusive in which the cumulative operation for the value at
///     index i of the output contains all elements before index i
///     in the input
/// @li inclusive in which the cumulative operation for element at index
///     i of the output contains all elements before index i and at
///     index i of the input.
///
/// This function specifies the default skeleton to be used for both
/// inclusive and exclusive skeletons. The default case for an exclusive
/// scan is Blelloch and the default for inclusive scan is Binomial.
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan operation.
/// @param shift Whether to use an exclusive or inclusive scan.
/// @tparam CoarseTag which type of coarsening to be used for the
///         coarsening of the scan skeleton. One can pass no_coarsening
///         in order to avoid the coarsening as a whole.
///
/// @see inclusive_scan
/// @see exclusive_scan
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename CoarseTag = stapl::use_default,
         typename View0, typename View1, typename Binop>
void scan(View0&& view0, View1&& view1, Binop binop,
          bool shift = false)
{
  using namespace skeletons;
  if (shift) {
    scan<tags::scan<tags::blelloch, tags::exclusive>, CoarseTag>
      (std::forward<View0>(view0), std::forward<View1>(view1), binop);
  }
  else {
    scan<tags::scan<tags::binomial, tags::inclusive>, CoarseTag>
      (std::forward<View0>(view0), std::forward<View1>(view1), binop);
  }
}

} // namespace stapl
#endif // STAPL_SKELETONS_SCAN_HPP
