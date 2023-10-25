/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ZIP_REDUCE_HPP
#define STAPL_SKELETONS_ZIP_REDUCE_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include "zip.hpp"
#include "reduce.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a zip_reduce skeleton
/// by exposing only the necessary information in its representation.
///
/// A zip_reduce skeleton is a skeleton that first combines the given
/// inputs pairwise and then applies a reduction on the produced results
/// in order to produce one result.
///
/// The most used specialization of a zip_reduce is a map_reduce in
/// which only one input is passed in to the skeleton.
///
/// This abstraction not only makes the reconstruction of a
/// a zip_reduce skeleton easier, but also reduces the symbol size for a
/// zip_reduce skeleton, hence, reducing the total compilation time.
///
/// @tparam arity  the arity of the zip skeleton.
/// @tparam ZipOp  the underlying operation to combine the input element.
/// @tparam RedOp  the underlying operation to reduce the produced
///                result by the zip skeleton.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<int arity, typename ZipOp, typename RedOp>
struct zip_reduce
  : public
      decltype(
        skeletons::compose(
          skeletons::zip<arity>(std::declval<ZipOp>()),
          skeletons::reduce(std::declval<RedOp>())
        )
      )
{
  using skeleton_tag_type = tags::zip_reduce<arity>;
  using base_type = decltype(
                      skeletons::compose(
                        skeletons::zip<arity>(std::declval<ZipOp>()),
                        skeletons::reduce(std::declval<RedOp>())));

  using zip_op_type = typename std::decay<decltype(
                        std::declval<base_type>().
                          template get_skeleton<0>().get_op())>::type;
  using reduce_op_type = typename std::decay<decltype(
                           std::declval<base_type>().
                             template get_skeleton<1>().get_op())>::type;

  zip_reduce(ZipOp const& zip_op, RedOp const& reduce_op)
    : base_type(
        skeletons::compose(
          skeletons::zip<arity>(zip_op),
          skeletons::reduce(reduce_op))
      )
  { }

  reduce_op_type get_reduce_op(void) const
  {
    return base_type::template get_skeleton<1>().get_op();
  }

  zip_op_type get_zip_op(void) const
  {
    return base_type::template get_skeleton<0>().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}


namespace result_of {

template<int Arity, typename ZipOp, typename RedOp>
using zip_reduce = skeletons_impl::zip_reduce<
                     Arity,
                     typename std::decay<ZipOp>::type,
                     typename std::decay<RedOp>::type>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief As its name implies it creates a zip-reduce skeleton by
/// piping the result of a @c zip skeleton to a @c reduce skeleton.
///
/// The difference between @c map_reduce and @c zip_reduce is in the
/// arity of the first phase. In other words a @c map_reduce is a
/// @c zip_reduce with the arity of 1.
///
/// @param zip_op    the operation to be applied in the @c zip step
/// @param reduce_op the operation to be applied in the @c reduce step
///
/// @return a zip-reduce skeleton
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <int Arity = 2,
          typename ZipOp, typename RedOp>
result_of::zip_reduce<Arity, ZipOp, RedOp>
zip_reduce(ZipOp&& zip_op,
           RedOp&& reduce_op)
{
  return result_of::zip_reduce<Arity, ZipOp, RedOp>(
           std::forward<ZipOp>(zip_op),
           std::forward<RedOp>(reduce_op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_ZIP_REDUCE_HPP
