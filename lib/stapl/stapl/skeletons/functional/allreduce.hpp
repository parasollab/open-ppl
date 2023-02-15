/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_ALLREDUCE_HPP
#define STAPL_SKELETONS_FUNCTIONAL_ALLREDUCE_HPP

#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include "reduce.hpp"
#include "broadcast.hpp"
#include "reduce_to_pow_two.hpp"
#include "expand_from_pow_two.hpp"
#include "butterfly.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an allreduce skeleton
/// by exposing only the necessary information in its representation.
///
/// The general case for an allreduce skeleton which can handle inputs
/// of arbitrary sizes is a reduction skeleton followed by a broadcast
/// skeleton.
///
/// This abstraction not only makes the reconstruction of an
/// allreduce skeleton easier, but also provides access to the
/// underlying operations of the enclosed reduction operation. Furthermore,
/// it reduces the symbol size for an allreduce skeleton, hence, reducing
/// the total compilation time.
///
/// @tparam Op   the operation to be used while reducing the input.
/// @tparam Span    the iteration space for elements on each level of
///                 both the reduction and the broadcast tree.
/// @tparam Tag     determines the type of the allreduce skeleton
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename Op, typename Span, typename Tag>
struct allreduce
  : public decltype(
             skeletons::compose(
               skeletons::reduce<Tag, stapl::use_default, Span>(
                 std::declval<Op>()),
               skeletons::broadcast<Tag, stapl::use_default, Span>(
                 stapl::identity_op()))
           )
{
  using skeleton_tag_type = tags::allreduce<Tag>;
  using base_type = decltype(
                      skeletons::compose(
                        skeletons::reduce<Tag, stapl::use_default, Span>(
                          std::declval<Op>()),
                        skeletons::broadcast<Tag, stapl::use_default, Span>(
                          stapl::identity_op())));

  allreduce(Op const& op)
    : base_type(
        skeletons::compose(
          skeletons::reduce<Tag, stapl::use_default, Span>(op),
          skeletons::broadcast<Tag, stapl::use_default, Span>(
            stapl::identity_op()))
      )
  { }

  Op get_op() const
  {
    return base_type::template get_skeleton<0>().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a butterfly-based
/// allreduce skeleton by exposing only the necessary information in
/// its representation.
///
/// A butterfly-based allreduce skeleton can handle only inputs of
/// power-of-two sizes.
///
/// This abstraction not only makes the reconstruction of an
/// butterfly-based allreduce skeleton easier, but also provides access
/// to the underlying operations of the enclosed reduction operation.
/// Furthermore, it reduces the symbol size for an allreduce skeleton,
/// hence, reducing the total compilation time.
///
/// @tparam Op   the operation to be used while reducing the input.
/// @tparam Span    the iteration space for elements on each level of
///                 both the reduction and the broadcast tree.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename Op, typename Span>
struct allreduce<Op, Span, tags::butterfly<false>>
  : public decltype(
             skeletons::compose(
               skeletons::reduce_to_pow_two<Span>(std::declval<Op>()),
               skeletons::butterfly<
                 false, stapl::use_default, spans::nearest_pow_two<Span>
               >(std::declval<Op>()),
               skeletons::expand_from_pow_two<Span>(stapl::identity_op())
             )
           )
{
  using skeleton_tag_type = tags::allreduce<tags::butterfly<false>>;
  using base_type = decltype(
                      skeletons::compose(
                        skeletons::reduce_to_pow_two<Span>(std::declval<Op>()),
                        skeletons::butterfly<
                          false, stapl::use_default,
                          spans::nearest_pow_two<Span>
                        >(std::declval<Op>()),
                        skeletons::expand_from_pow_two<Span>(
                          stapl::identity_op())));

  allreduce(Op const& op)
    : base_type(
        skeletons::compose(
          skeletons::reduce_to_pow_two<Span>(op),
          skeletons::butterfly<
            false, stapl::use_default, spans::nearest_pow_two<Span>
          >(op),
          skeletons::expand_from_pow_two<Span>(stapl::identity_op())
        )
      )
  { }

  Op get_op() const
  {
    return base_type::template get_skeleton<0>().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a reverse-butterfly-based
/// allreduce skeleton by exposing only the necessary information in
/// its representation.
///
/// A reverse-butterfly-based allreduce skeleton can handle only
/// inputs of power-of-two sizes.
///
/// This abstraction not only makes the reconstruction of an
/// reverse-butterfly-based allreduce skeleton easier, but also provides
/// access to the underlying reduction operations.
/// Furthermore, it reduces the symbol size for an allreduce skeleton,
/// hence, reducing the total compilation time.
///
/// @tparam Op   the operation to be used while reducing the input.
/// @tparam Span    the iteration space for elements on each level of
///                 both the reduction and the broadcast tree.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename Op, typename Span, bool B>
struct allreduce<Op, Span, tags::reverse_butterfly<B>>
  : public decltype(
             skeletons::compose(
               skeletons::reduce_to_pow_two<Span>(std::declval<Op>()),
               skeletons::reverse_butterfly<
                 false, stapl::use_default, spans::nearest_pow_two<Span>
               >(std::declval<Op>()),
               skeletons::expand_from_pow_two<Span>(stapl::identity_op())))
{
  using skeleton_tag_type = tags::allreduce<tags::reverse_butterfly<B>>;
  using base_type = decltype(
                      skeletons::compose(
                        skeletons::reduce_to_pow_two<Span>(std::declval<Op>()),
                        skeletons::reverse_butterfly<
                          false, stapl::use_default,
                          spans::nearest_pow_two<Span>
                        >(std::declval<Op>()),
                        skeletons::expand_from_pow_two<Span>(
                          stapl::identity_op())));

  allreduce(Op const& op)
    : base_type(
        skeletons::compose(
          skeletons::reduce_to_pow_two<Span>(op),
          skeletons::reverse_butterfly<
            false, stapl::use_default, spans::nearest_pow_two<Span>
          >(op),
          skeletons::expand_from_pow_two<Span>(stapl::identity_op())))
  { }

  Op get_op(void) const
  {
    return base_type::get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespae skeletons_impl

namespace result_of {

template <typename Tag, typename Span, typename Op>
using allreduce = skeletons_impl::allreduce<
                    typename std::decay<Op>::type,
                    stapl::default_type<Span, spans::balanced<>>,
                    stapl::default_type<Tag, tags::left_aligned>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief This all reduce skeleton consists of a reduction phase and
/// a broadcast phase. You can use various types of reduce and
/// broadcast for this skeleton by specifying a tag. Your tag can be
/// one of, but not limited to, the following tags:
/// @li tags::right_aligned - which will use right-aligned reduce and
///     broadcast
/// @li tags::left_aligned  - which will use left-aligned reduce and
///     broadcast
/// @li stapl::use_default  - which will use the default reduce and
///     broadcast
///
/// @param  reduce_op    the operation to be used for reduction
/// @param  tag          determines which type of reduce and broadcast
///                      should be used
/// @tparam Span         the span to be used for @c reduce and
///                      @c broadcast skeletons
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <typename Tag = stapl::use_default,
          typename Span = stapl::use_default,
          typename Op>
skeletons::result_of::allreduce<Tag, Span, Op>
allreduce(Op&& reduce_op)
{
  return skeletons::result_of::allreduce<Tag, Span, Op>(
           std::forward<Op>(reduce_op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_ALLREDUCE_HPP
