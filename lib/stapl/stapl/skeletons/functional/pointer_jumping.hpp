/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_POINTER_JUMPING_HPP
#define STAPL_SKELETONS_FUNCTIONAL_POINTER_JUMPING_HPP

#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/lazy_sizes.hpp>
#include <stapl/skeletons/param_deps/pointer_jumping_pd.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/operators/repeat.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of pointer jumping skeleton
/// by exposing only the necessary information in its representation.
///
/// Pointer jumping skeleton (aka recursive doubling) is used
/// in various computations such as Hillis-Steele scan.
///
/// A pointer jumping skeleton consists of log2 levels (\f$k\f$) of reading
/// from \f$i - 2^{k}\f$ and combining the value with the value
/// already existing at index \f$i\f$.
///
/// This abstraction not only makes the reconstruction of an
/// pointer_jumping skeleton easier, but also provides access to the
/// underlying operation. Furthermore, it reduces the symbol size for
/// an allreduce skeleton, hence, reducing the total compilation time.
///
/// @tparam Op    the operation to be used in each level
/// @tparam Flows the flows to be used for the repetition
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename Op, typename Flows>
struct pointer_jumping
  : public decltype(
             skeletons::repeat<Flows>(
               skeletons::elem(
                 skeletons::pointer_jumping_pd(std::declval<Op>())),
               log_ceil_lazysize<2>()))
{
  using skeleton_tag_type = tags::pointer_jumping;
  using base_type = decltype(
                      skeletons::repeat<Flows>(
                        skeletons::elem(
                          skeletons::pointer_jumping_pd(std::declval<Op>())),
                        log_ceil_lazysize<2>()));
  using op_type     = Op;

  explicit pointer_jumping(Op const& op)
    : base_type(
        skeletons::repeat<Flows>(
          skeletons::elem(skeletons::pointer_jumping_pd(op)),
          log_ceil_lazysize<2>()
        )
      )
  { }

  Op get_op(void) const
  {
    return base_type::nested_skeleton().nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}

namespace result_of {

template <typename Flows, typename Op>
using pointer_jumping = skeletons_impl::pointer_jumping<
                          typename std::decay<Op>::type, Flows>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A pointer jumping skeleton consists of \f$log2()\f$ levels
/// \f$(k)\f$ of combining values of a given input with an element
/// with the index \f$i - 2^{k}\f$.
///
/// Pointer jumping is used in various algorithm such as Hillis-Steele
/// scan.
///
/// @param  op    the operation to be used in each level
/// @tparam Flows the flows to be used in @c repeat
///
/// @ingroup skeletonsFunctionalExchange
//////////////////////////////////////////////////////////////////////
template <typename Flows = stapl::use_default,
          typename Op>
result_of::pointer_jumping<Flows, Op>
pointer_jumping(Op&& op)
{
  return result_of::pointer_jumping<Flows, Op>(std::forward<Op>(op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_POINTER_JUMPING_HPP
