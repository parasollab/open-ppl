/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SCAN_REDUCE_HPP
#define STAPL_SKELETONS_SCAN_REDUCE_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/scan_reduce.hpp>
#include <stapl/skeletons/transformations/coarse/scan_reduce.hpp>
#include <stapl/algorithms/identity_value.hpp>

#include <stapl/views/metadata/coarseners/default.hpp>

namespace stapl {
namespace scan_reduce_helpers {

//////////////////////////////////////////////////////////////////////
/// @brief Computes the inclusive scan of the elements of the input
/// view and stores the result in the output view. Also computes
/// the reduce value of input elements and returns it.
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
///              the result of scan and reduce operation.
/// @param tag   determines the type of inclusive scan to be used
/// @tparam CoarseTag specializes the coarsened scan
///
/// @returns the result of reduce skeleton
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1, typename Binop, typename Tag,
          typename CoarseTag>
typename std::decay<View0>::type::value_type
scan_reduce(
  View0&& view0, View1&& view1, Binop binop,
  skeletons::tags::scan_reduce<Tag, skeletons::tags::inclusive>, CoarseTag)
{
  using value_type = typename std::decay<View0>::type::value_type;
  return skeletons::execute(
    skeletons::execution_params<value_type>(default_coarsener()),
    skeletons::coarse<CoarseTag>(
      skeletons::scan_reduce<value_type, Tag>(binop)),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the inclusive scan of the elements of the input
/// view and stores the result in the output view. Also computes
/// the reduce value of input elements and returns it.
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan and reduce operation.
/// @param tag   determines the type of inclusive scan to be used
/// @see inclusive_scan
///
/// @returns the result of reduce skeleton
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1, typename Binop, typename Tag>
typename std::decay<View0>::type::value_type
scan_reduce(View0&& view0, View1&& view1, Binop binop,
            skeletons::tags::scan_reduce<Tag, skeletons::tags::inclusive>,
            skeletons::tags::no_coarsening)
{
  using value_type = typename std::decay<View0>::type::value_type;
  return skeletons::execute(
    skeletons::execution_params<value_type>(),
    skeletons::scan_reduce<value_type, Tag>(binop),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the exclusive scan of the elements of the input
/// view and stores the result in the output view. Also computes
/// the reduce value of input elements and returns it.
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
///              the result of scan and reduce operation.
/// @param tag   determines the type of exclusive scan to be used
/// @tparam CoarseTag specializes the coarsened scan
///
/// @returns the result of reduce skeleton
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1, typename Binop, typename Tag,
          typename CoarseTag>
typename std::decay<View0>::type::value_type
scan_reduce(
  View0&& view0, View1&& view1, Binop binop,
  skeletons::tags::scan_reduce<Tag, skeletons::tags::exclusive>, CoarseTag)
{
  using value_type = typename std::decay<View0>::type::value_type;
  return skeletons::execute(
    skeletons::execution_params<value_type>(default_coarsener()),
    skeletons::coarse<CoarseTag>(skeletons::scan_reduce<value_type, Tag>(
      binop,
      identity_value<typename std::decay<Binop>::type, value_type>::value())),
    std::forward<View0>(view0), std::forward<View1>(view1));
}

//////////////////////////////////////////////////////////////////////
/// @brief A non-coarsened exclusive scan_reduce skips the coarsening process
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
///
/// @returns the result of reduce skeleton
/// @see reduce
/// @see exclusive_scan
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1, typename Binop, typename Tag>
typename std::decay<View0>::type::value_type
scan_reduce(View0&& view0, View1&& view1, Binop binop,
            skeletons::tags::scan_reduce<Tag, skeletons::tags::exclusive>,
            skeletons::tags::no_coarsening)
{
  using value_t = typename std::decay<View0>::type::value_type;
  return skeletons::execute(
    skeletons::execution_params<value_t>(),
    skeletons::scan_reduce<value_t, Tag>(
      binop,
      identity_value<typename std::decay<Binop>::type, value_t>::value()),
    std::forward<View0>(view0),
    std::forward<View1>(view1));
}

} // namespace scan_reduce_helpers

//////////////////////////////////////////////////////////////////////
/// @brief combines the functionality of scan and reduce skeleton.
///
/// for the scan skeleton it can be one of two types
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
///              the result of scan and reduce operation.
/// @tparam ScanTag   which scan algorithm to be used
/// @tparam CoarseTag what type of coarsening to be used (if any)
///
/// @returns the result of reduce skeleton
/// @see reduce
/// @see scan
/// @see inclusive_scan
/// @see exclusive_scan
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename ScanTag, typename CoarseTag,
         typename View0, typename View1, typename Binop>
typename std::decay<View0>::type::value_type
scan_reduce(View0&& view0, View1&& view1, Binop binop)
{
  return scan_reduce_helpers::scan_reduce(std::forward<View0>(view0),
                                          std::forward<View1>(view1),
                                          binop,
                                          ScanTag(), CoarseTag());
}

//////////////////////////////////////////////////////////////////////
/// @brief Combines the functionality of scan and reduce skeleton.
///
/// This function specifies the default skeleton to be used for scan
/// skeleton. The default case for an exclusive scan is Blelloch and
/// the default for inclusive scan is Binomial.
///
/// @param view0 A one-dimensional view over the input elements that
///              are of a numeric type.
/// @param view1 A one-dimensional view over the elements where the
///              result of the scan will be written.
/// @param binop The binary functor that will be used to compute
///              the result of scan and reduce operation.
/// @param shift Whether to use an exclusive or inclusive scan.
/// @tparam CoarseTag which type of coarsening to be used for the
///         coarsening of the scan skeleton. One can pass no_coarsening
///         in order to avoid the coarsening as a whole.
/// @returns the result of reduce skeleton
/// @see reduce
/// @see scan
/// @see inclusive_scan
/// @see exclusive_scan
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename CoarseTag = stapl::use_default,
         typename View0, typename View1, typename Binop>
typename std::decay<View0>::type::value_type
scan_reduce(View0&& view0, View1&& view1, Binop binop, bool shift = false)
{
  using namespace skeletons;
  if (shift) {
    return scan_reduce<tags::scan_reduce<tags::blelloch, tags::exclusive>,
                       CoarseTag>(std::forward<View0>(view0),
                                  std::forward<View1>(view1),
                                  binop);
  } else {
    return scan_reduce<tags::scan_reduce<tags::binomial, tags::inclusive>,
                       CoarseTag>(std::forward<View0>(view0),
                                  std::forward<View1>(view1),
                                  binop);
  }
}

} // namespace stapl
#endif // STAPL_SKELETONS_SCAN_REDUCE_HPP
