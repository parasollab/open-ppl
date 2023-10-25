/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_BUTTERFLY_HPP
#define STAPL_SKELETONS_FUNCTIONAL_BUTTERFLY_HPP

#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/lazy_sizes.hpp>
#include <stapl/skeletons/param_deps/butterfly_pd.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/operators/repeat.hpp>

namespace stapl {
namespace skeletons {

namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a butterfly skeleton
/// by exposing only the necessary information in its representation.
///
/// An butterfly skeleton is a well-known skeleton used especially in
/// network configuration and other applications such as FFT.
///
/// This abstraction not only makes the reconstruction of a butterfly
/// skeleton easier, but also provides access to the underlying
/// operation and filter used in each level. Furthermore, it reduces
/// the symbol size for an butterfly skeleton, hence, reducing the
/// total compilation time.
///
/// @tparam Op        the operation to be used while reducing the input.
/// @tparam is_pos_aware whether the computation needs spatial information
///                   for its computation or not.
/// @tparam is_reversed whether this butterfly is a regular or a reversed
///                   butterfly
/// @tparam Filter    the filter to be applied on the result of each
///                   computation. In some algorithms only a portion of
///                   the produced data by @c Op is passed to the next
///                   computations.
/// @tparam Flows     the flow between levels of the butterfly skeleton.
/// @tparam Span      the iteration space for elements on each level of
///                   the butterfly skeleton
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, bool is_pos_aware, bool is_reversed,
          typename Filter, typename Flows, typename Span>
struct butterfly
  : public decltype(
             skeletons::repeat<Flows>(
               skeletons::elem<Span>(
                 skeletons::butterfly_pd<is_pos_aware, is_reversed>(
                   std::declval<Op>(), std::declval<Filter>())),
               log_lazysize<2>()
             )
           )
{
  using skeleton_tag_type = typename std::conditional<
                              is_reversed,
                              tags::reverse_butterfly<
                                std::is_same<Filter, stapl::use_default>::value
                              >,
                              tags::butterfly<
                                std::is_same<Filter, stapl::use_default>::value
                              >>::type;

  using op_type = Op;

  using base_type = decltype(
                      skeletons::repeat<Flows>(
                        skeletons::elem<Span>(
                          skeletons::butterfly_pd<is_pos_aware, is_reversed>(
                            std::declval<Op>(), std::declval<Filter>())),
                        log_lazysize<2>()));

  butterfly(Op const& op, Filter const& filter)
    : base_type(
        skeletons::repeat<Flows>(
          skeletons::elem<Span>(
            skeletons::butterfly_pd<is_pos_aware, is_reversed>(op, filter)),
          log_lazysize<2>()
        )
      )
  { }

  Op get_op(void) const
  {
    return base_type::nested_skeleton().nested_skeleton().get_op();
  }

  Filter get_filter(void) const
  {
    return base_type::nested_skeleton().nested_skeleton().get_filter();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}

namespace result_of {

template <bool is_pos_aware,
          typename Flows,
          typename Span,
          typename Op,
          typename Filter>
using butterfly = skeletons_impl::butterfly<
                    typename std::decay<Op>::type,
                    is_pos_aware, false,
                    typename std::decay<Filter>::type,
                    Flows, Span>;

template <bool is_pos_aware,
          typename Flows,
          typename Span,
          typename Op,
          typename Filter>
using reverse_butterfly = skeletons_impl::butterfly<
                            typename std::decay<Op>::type,
                            is_pos_aware, true,
                            typename std::decay<Filter>::type, Flows, Span>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A butterfly skeleton is used in various applications
/// including but not limited to a type of allreduce, FFT DIT.
///
/// @tparam is_pos_aware in some algorithms that use butterfly skeleton
///                   knowing the position of the element in the
///                   overall butterfly dependence graph is necessary,
///                   e.g., in FFT DIT. If you set this template
///                   parameter to true, this information will be sent
///                   to your workfunction by invoking its @c set_position
/// @tparam Flows     the flow to be used between the levels of the butterfly
/// @tparam Span      the iteration space for elements on each level of
///                   the butterfly skeleton
/// @param  op        the workfunction to be used in each butterfly
///                   parametric dependency.
/// @return a butterfly skeleton
///
/// @see butterfly_pd
/// @see log_lazysize
///
/// @ingroup skeletonsFunctionalExchange
//////////////////////////////////////////////////////////////////////
template <bool is_pos_aware = false,
          typename Flows    = stapl::use_default,
          typename Span     = stapl::use_default,
          typename Op,
          typename Filter   = skeletons::no_filter>
result_of::butterfly<is_pos_aware, Flows, Span, Op, Filter>
butterfly(Op&& op, Filter&& filter = Filter())
{
  return result_of::butterfly<is_pos_aware, Flows, Span, Op, Filter>(
           std::forward<Op>(op),
           std::forward<Filter>(filter));
}

//////////////////////////////////////////////////////////////////////
/// @brief A reverse butterfly skeleton is used in various applications
/// including but not limited to a type of allreduce, FFT DIT.
///
/// @tparam is_pos_aware in some algorithms that use reverse butterfly
///                   skeleton knowing the position of the element in the
///                   overall reverse butterfly dependence graph is
///                   necessary, e.g., in FFT DIT. If you set this
///                   template parameter to true, this information will
///                   be sent to your workfunction by invoking its
///                   @c set_position
/// @tparam Flows     the flow to be used between the levels of the
///                   reverse butterfly
/// @tparam Span      the iteration space for elements on each level of
///                   the reverse butterfly skeleton
/// @param  op        the workfunction to be used in each reverse butterfly
///                   parametric dependency.
/// @return a reverse butterfly skeleton
///
/// @see butterfly_pd
/// @see log_lazysize
///
/// @ingroup skeletonsFunctionalExchange
//////////////////////////////////////////////////////////////////////
template <bool is_pos_aware = false,
          typename Flows    = stapl::use_default,
          typename Span     = stapl::use_default,
          typename Op,
          typename Filter   = skeletons::no_filter>
result_of::reverse_butterfly<is_pos_aware, Flows, Span, Op, Filter>
reverse_butterfly(Op&& op, Filter&& filter = Filter())
{
  return result_of::reverse_butterfly<is_pos_aware, Flows, Span, Op, Filter>(
           std::forward<Op>(op),
           std::forward<Filter>(filter));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_BUTTERFLY_HPP