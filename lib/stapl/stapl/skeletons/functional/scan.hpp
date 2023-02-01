/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_SCAN_HPP
#define STAPL_SKELETONS_FUNCTIONAL_SCAN_HPP

#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/flows/repeat_flows.hpp>
#include <stapl/skeletons/flows/scan_flows.hpp>
#include <stapl/skeletons/param_deps/scan_broadcast_pd.hpp>
#include <stapl/skeletons/param_deps/scan_blelloch_broadcast_pd.hpp>
#include <stapl/skeletons/param_deps/scan_expand_from_pow_two_pd.hpp>
#include <stapl/skeletons/param_deps/binomial_tree_pd.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/binomial_tree.hpp>
#include <stapl/skeletons/spans/nearest_pow_two.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include "pointer_jumping.hpp"
#include "reduce_to_pow_two.hpp"
#include "binary_tree.hpp"
#include "tree.hpp"

#include <boost/mpl/has_xxx.hpp>

namespace stapl {
namespace skeletons {

BOOST_MPL_HAS_XXX_TRAIT_DEF(first_argument_type)

template <typename Op,
          bool with_arg_type = has_first_argument_type<Op>::value>
struct scan_operand_type
{
  using type = typename std::decay<Op>::type::first_argument_type;
};


template <typename Op>
struct scan_operand_type<Op, false>
{
  using type = typename std::decay<Op>::type::result_type;
};


namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Determines the scan algorithm used.
//////////////////////////////////////////////////////////////////////
template <typename Tag>
struct scan_algorithm_tag_type;


template <typename Tag, typename IE>
struct scan_algorithm_tag_type<tags::scan<Tag, IE>>
{
  using type = Tag;
};

template <typename Op, typename Tag>
struct inclusive_scan;


template <typename Op, typename T, typename Tag>
struct exclusive_scan;

//////////////////////////////////////////////////////////////////////
/// @brief Hillis Steele scan skeleton is a pointer-jumping based
/// inclusive scan algorithm which has half the height of the scan
/// algorithm in Joseph Jaja's book, but it performs \f$O(n logn)\f$
/// operations and hence is not work optimal.
///
/// @tparam Op the operation to be used to compute the scan results
///
/// @see flows::repeat_flows::piped
/// @see flows::repeat_flows::input_wrapper
/// @see pointer_jumping
/// @see scan
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct inclusive_scan<Op, tags::hillis_steele>
  : public decltype(
             skeletons::pointer_jumping<
               flows::repeat_flows::input_wrapper<flows::repeat_flows::piped>
             >(std::declval<Op>()))
{
  using skeleton_tag_type = tags::scan<tags::hillis_steele, tags::inclusive>;
  using base_type = decltype(
                      skeletons::pointer_jumping<
                        flows::repeat_flows::input_wrapper<
                          flows::repeat_flows::piped>
                      >(std::declval<Op>()));

  explicit inclusive_scan(Op const& op)
    : base_type(op)
  { }

  Op get_op() const
  {
    return base_type::get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Binomial scan is one of the common implementations of
/// MPI_Scan.
///
/// A binomial scan consists of an up-phase and a down-phase. In the
/// up-phase, one or more reduction trees (depending on whether the
/// input size is a power-of-two or not) reduce the values and prepare
/// partial results for the down-phase. In the down-phase, partial
/// results are combined to produce the final result.
///
/// @code
/// O O O O O O
/// |\| |\| |\|
/// | O_| O | O
/// | | |\| | |
/// | | | O | |
/// | | | | | |
/// | | | O | |
/// | | | |\|_|
/// | O | O | O
/// | |\| |\| |
/// O O O O O O
/// @endcode
///
/// @tparam Op the operation to be used to compute the scan results
///
/// @see flows::repeat_flows::output_from_all
/// @see flows::repeat_flows::
/// @see flows::scan_f::scan_broadcast
/// @see spans::binomial_tree
/// @see binomial_tree_pd
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct inclusive_scan<Op, tags::binomial>
  : public decltype(
             skeletons::compose<
               flows::repeat_flows::input_wrapper<
                 flows::compose_flows::input_to_all
               >
             >(skeletons::repeat<flows::repeat_flows::output_from_all>(
                 skeletons::elem<
                   spans::binomial_tree<spans::balanced<>, tags::up_phase>
                 >(skeletons::binomial_tree_pd<tags::up_phase>(
                     std::declval<Op>())),
                 log_lazysize<2>()),
               skeletons::repeat<flows::repeat_flows::scan_broadcast>(
                 skeletons::elem<
                   spans::binomial_tree<spans::balanced<>, tags::down_phase>
                 >(skeletons::binomial_tree_pd<tags::down_phase>(
                     std::declval<Op>())),
                 log_ceil_lazysize<2>()))
           )
{
  using skeleton_tag_type = tags::scan<tags::binomial, tags::inclusive>;
  using base_type = decltype(
                      skeletons::compose<
                        flows::repeat_flows::input_wrapper<
                          flows::compose_flows::input_to_all
                        >
                      >(skeletons::repeat<flows::repeat_flows::output_from_all>(
                          skeletons::elem<
                            spans::binomial_tree<
                              spans::balanced<>, tags::up_phase>
                          >(skeletons::binomial_tree_pd<tags::up_phase>(
                              std::declval<Op>())),
                          log_lazysize<2>()),
                        skeletons::repeat<flows::repeat_flows::scan_broadcast>(
                          skeletons::elem<
                            spans::binomial_tree<
                              spans::balanced<>, tags::down_phase>
                          >(skeletons::binomial_tree_pd<tags::down_phase>(
                              std::declval<Op>())),
                          log_ceil_lazysize<2>())));

  explicit inclusive_scan(Op const& op)
    : base_type(
        skeletons::compose<
          flows::repeat_flows::input_wrapper<flows::compose_flows::input_to_all>
        >(skeletons::repeat<flows::repeat_flows::output_from_all>(
            skeletons::elem<
              spans::binomial_tree<spans::balanced<>, tags::up_phase>
            >(skeletons::binomial_tree_pd<tags::up_phase>(op)),
            log_lazysize<2>()),
          skeletons::repeat<flows::repeat_flows::scan_broadcast>(
            skeletons::elem<
              spans::binomial_tree<spans::balanced<>, tags::down_phase>
            >(skeletons::binomial_tree_pd<tags::down_phase>(op)),
            log_ceil_lazysize<2>()))
      )
  { }

  Op get_op() const
  {
    return base_type::template get_skeleton<0>().nested_skeleton().
             nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This scan skeleton is based on the algorithm given in
/// <b>An Introduction to Parallel Algorithms</b> by Joseph Jaja, page
/// 48. This algorithm's time complexity is \f$O(log n)\f$. It
/// consists of a reduction tree followed by a modified broadcast
/// skeleton. This scan skeleton is an inclusive scan.
///
/// @tparam Op the operation to be used to compute the scan results
///
/// @see flows::compose_flows::input_to_all
/// @see flows::repeat_flows::output_from_all
/// @see flows::repeat_flows::scan_broadcast
/// @see scan_broadcast_pd
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct inclusive_scan<Op, tags::jaja>
  : public decltype(
             skeletons::compose<flows::compose_flows::input_to_all>(
               skeletons::binary_tree<
                 tags::left_skewed,
                 flows::repeat_flows::output_from_all,
                 spans::nearest_pow_two<spans::balanced<>>, true
               >(std::declval<Op>()),
               skeletons::reverse_tree<
                 2, flows::repeat_flows::scan_broadcast,
                 spans::reverse_tree<
                   spans::nearest_pow_two<spans::balanced<>>, tags::left_skewed
                 >
               >(skeletons::scan_broadcast_pd(std::declval<Op>()))))
{
  using skeleton_tag_type = tags::scan<tags::jaja, tags::inclusive>;

  using base_type = decltype(
                      skeletons::compose<flows::compose_flows::input_to_all>(
                        skeletons::binary_tree<
                          tags::left_skewed,
                          flows::repeat_flows::output_from_all,
                          spans::nearest_pow_two<spans::balanced<>>, true
                        >(std::declval<Op>()),
                        skeletons::reverse_tree<
                          2, flows::repeat_flows::scan_broadcast,
                          spans::reverse_tree<
                            spans::nearest_pow_two<spans::balanced<>>,
                            tags::left_skewed
                          >
                        >(skeletons::scan_broadcast_pd(std::declval<Op>()))));

  explicit inclusive_scan(Op const& op)
    : base_type(
        skeletons::compose<flows::compose_flows::input_to_all>(
          skeletons::binary_tree<
            tags::left_skewed,
            flows::repeat_flows::output_from_all,
            spans::nearest_pow_two<spans::balanced<>>, true
          >(op),
          skeletons::reverse_tree<
            2, flows::repeat_flows::scan_broadcast,
            spans::reverse_tree<
              spans::nearest_pow_two<spans::balanced<>>, tags::left_skewed
            >
          >(skeletons::scan_broadcast_pd(op)))
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
/// @brief Blelloch scan is an exclusive scan algorithm which is
/// similar to the inclusive scan mentioned in Joseph Jaja's book
/// (@see inclusive_scan(op,tags::jaja)).
///
/// @tparam Op the operation to be used to compute the scan results
/// @tparam T  the type of the initial value for the exclusive scan.
///
/// @see flows::compose_flows::input_to_all
/// @see flows::repeat_flows::output_from_all
/// @see flows::repeat_flows::scan_broadcast
/// @see scan_blelloch_broadcast_pd
/// @see
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename T>
struct exclusive_scan<Op, T, tags::blelloch>
  : public decltype(
             skeletons::compose<
               flows::repeat_flows::input_wrapper<
                 flows::compose_flows::input_to_all>
             >(skeletons::binary_tree<
                 tags::right_aligned,
                 flows::repeat_flows::output_from_all,
                 spans::nearest_pow_two<spans::balanced<>>, true
               >(std::declval<Op>()),
               skeletons::reverse_tree<
                 2, flows::repeat_flows::scan_broadcast,
                 spans::reverse_tree<
                   spans::nearest_pow_two<spans::balanced<>>,
                   tags::right_aligned
                 >
               >(skeletons::scan_blelloch_broadcast_pd(
                  std::declval<Op>(), T())))
           )
{
  using skeleton_tag_type = tags::scan<tags::blelloch, tags::exclusive>;

  using base_type = decltype(
                      skeletons::compose<
                         flows::repeat_flows::input_wrapper<
                           flows::compose_flows::input_to_all
                         >
                      >(skeletons::binary_tree<
                          tags::right_aligned,
                          flows::repeat_flows::output_from_all,
                          spans::nearest_pow_two<spans::balanced<>>, true
                        >(std::declval<Op>()),
                        skeletons::reverse_tree<
                          2, flows::repeat_flows::scan_broadcast,
                          spans::reverse_tree<
                            spans::nearest_pow_two<spans::balanced<>>,
                            tags::right_aligned
                          >
                        >(skeletons::scan_blelloch_broadcast_pd(
                            std::declval<Op>(), T()))));

  exclusive_scan(Op const& op, T const& initial_value)
    : base_type(
        skeletons::compose<
          flows::repeat_flows::input_wrapper<flows::compose_flows::input_to_all>
        >(skeletons::binary_tree<
            tags::right_aligned,
            flows::repeat_flows::output_from_all,
            spans::nearest_pow_two<spans::balanced<>>, true
          >(op),
          skeletons::reverse_tree<
            2, flows::repeat_flows::scan_broadcast,
            spans::reverse_tree<
              spans::nearest_pow_two<spans::balanced<>>, tags::right_aligned
            >
          >(skeletons::scan_blelloch_broadcast_pd(op, initial_value))
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

} // namespace skeletons_impl


namespace result_of {

template <typename Tag, typename Op>
using inclusive_scan = skeletons_impl::inclusive_scan<Op, Tag>;

template <typename Tag, typename Op, typename T>
using exclusive_scan = skeletons_impl::exclusive_scan<Op, T, Tag>;

}

//////////////////////////////////////////////////////////////////////
/// @brief An inclusive scan is a scan in which each element in the
/// output is the result of cumulative application of the operation
/// on the elements before it including itself.
///
/// For example,  if + is used as the operation, each element in the
/// output is the sum of all elements before it, and the first element
/// is the neutral value. An exclusive scan of [1, 2, 3, 4, ...] would
/// be [1, 3, 6, 10, ...].
///
/// @param Tag determines the type of inclusive scan to be used.
/// @param op  the operation to be used to compute the scan results
///
/// @see scan
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Op>
result_of::inclusive_scan<Tag, Op>
inclusive_scan(Op const& op)
{
  return result_of::inclusive_scan<Tag, Op>(op);
}

//////////////////////////////////////////////////////////////////////
/// @brief An exclusive scan is a scan in which each element in the
/// output is the result of cumulative application of the operation
/// on the elements before it.
///
/// @param T   value type of the operator
/// @param op  the operation to be used to compute the scan results
/// @param Tag determines the type of inclusive scan to be used.
///
/// @see scan
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Op, typename T>
result_of::exclusive_scan<Tag, Op, T>
exclusive_scan(Op const& op, T const& initial_value)
{
  return result_of::exclusive_scan<Tag, Op, T>(op, initial_value);
}

namespace skeletons_impl {

template<typename Op, typename Tag>
struct scan;

//////////////////////////////////////////////////////////////////////
/// @brief Most of the inclusive scan skeletons cannot work directly on
/// inputs with non-power-of-two sizes. As a result, we have to first
/// convert the input to the closest power of two and then perform the
/// scan operation.
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template<typename Op, typename Tag>
struct scan<Op, tags::scan<Tag, tags::inclusive>>
  : public decltype(
             skeletons::compose<flows::compose_flows::scan>(
               skeletons::reduce_to_pow_two(std::declval<Op>()),
               skeletons::inclusive_scan<Tag>(std::declval<Op>()),
               skeletons::elem(skeletons::scan_expand_from_pow_two_pd(
                                 std::declval<Op>(),
                                 tags::scan<Tag, tags::inclusive>())))
           )
{
  using skeleton_tag_type = tags::scan<Tag, tags::inclusive>;
  using base_type = decltype(
                      skeletons::compose<flows::compose_flows::scan>(
                        skeletons::reduce_to_pow_two(std::declval<Op>()),
                        skeletons::inclusive_scan<Tag>(std::declval<Op>()),
                        skeletons::elem(skeletons::scan_expand_from_pow_two_pd(
                                          std::declval<Op>(),
                                          tags::scan<Tag, tags::inclusive>())))
                    );

  explicit scan(Op const& op)
    : base_type(
        skeletons::compose<flows::compose_flows::scan>(
          skeletons::reduce_to_pow_two(op),
          skeletons::inclusive_scan<Tag>(op),
          skeletons::elem(skeletons::scan_expand_from_pow_two_pd(
                           op, tags::scan<Tag, tags::inclusive>())))
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
/// @brief Most of the exclusive scan skeletons cannot work directly on
/// inputs with non-power-of-two sizes. As a result, we have to first
/// convert the input to the closest power of two and then perform the
/// scan operation.
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template<typename Op, typename Tag>
struct scan<Op, tags::scan<Tag, tags::exclusive>>
  : public decltype(
             skeletons::compose<flows::compose_flows::scan>(
               skeletons::reduce_to_pow_two(std::declval<Op>()),
               skeletons::exclusive_scan<Tag>(
                 std::declval<Op>(),
                 typename scan_operand_type<Op>::type()),
               skeletons::elem(
                 skeletons::scan_expand_from_pow_two_pd(
                   std::declval<Op>(),
                   tags::scan<Tag, tags::exclusive>()))))
{
  using skeleton_tag_type = tags::scan<Tag, tags::exclusive>;
  using value_t = typename scan_operand_type<Op>::type;
  using base_type = decltype(
                      skeletons::compose<flows::compose_flows::scan>(
                        skeletons::reduce_to_pow_two(std::declval<Op>()),
                        skeletons::exclusive_scan<Tag>(
                          std::declval<Op>(), value_t()),
                        skeletons::elem(
                          skeletons::scan_expand_from_pow_two_pd(
                            std::declval<Op>(),
                            tags::scan<Tag, tags::exclusive>()))));

  scan(Op const& op, value_t const& initial_value)
    : base_type(
        skeletons::compose<flows::compose_flows::scan>(
          skeletons::reduce_to_pow_two(op),
          skeletons::exclusive_scan<Tag>(op, initial_value),
          skeletons::elem(
            skeletons::scan_expand_from_pow_two_pd(
              op, tags::scan<Tag, tags::exclusive>()))))
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


} // namespace skeletons_impl


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Hillis-Steele scan can work on inputs with non-power-of-two
/// sizes, therefore, we do not need to wrap it with extra steps.
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Op>
using scan = typename std::conditional<
               std::is_same<
                 tags::binomial,
                 typename skeletons_impl::scan_algorithm_tag_type<Tag>::type
               >::value ||
               std::is_same<
                 tags::hillis_steele,
                 typename skeletons_impl::scan_algorithm_tag_type<Tag>::type
               >::value,
               skeletons_impl::inclusive_scan<
                 typename std::decay<Op>::type,
                 typename skeletons_impl::scan_algorithm_tag_type<Tag>::type>,
               skeletons_impl::scan<typename std::decay<Op>::type, Tag>>::type;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A scan skeleton which is specialized based on the tag
/// given.
///
/// @ingroup skeletonsFunctionalAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename Tag = tags::binomial,
          typename Op>
result_of::scan<tags::scan<Tag, tags::inclusive>, Op>
scan(Op&& op)
{
  using std::is_same;
  static_assert(
    is_same<Tag, tags::binomial>::value      ||
    is_same<Tag, tags::hillis_steele>::value ||
    is_same<Tag, tags::jaja>::value,
    "The requested inclusive scan algorithm is not supported");
  return result_of::scan<tags::scan<Tag, tags::inclusive>, Op>(
           std::forward<Op>(op));
}

//////////////////////////////////////////////////////////////////////
/// @brief In an exclusive scan each element in the result is the
/// result of the cumulative operation on all elements before it.
///
/// For example,  if + is used as the operation, each element in the
/// output is the sum of all elements before it, and the first element
/// is the neutral value. An exclusive scan of [1, 2, 3, 4, ...] would
/// be [0, 1, 3, 6, 10, ...].
///
/// @param  op            the operation to be used to compute the scan
///                       results
/// @param  initial_value the value for the first element of the output
/// @tparam Tag           the type of exclusive scan to be used
///
/// @return an exclusive scan specified by the @c Tag
///
/// @see exclusive_scan
///
/// @ingroup skeletonsFunctionalAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename Tag = tags::blelloch,
          typename Op, typename T>
result_of::scan<tags::scan<Tag, tags::exclusive>, Op>
scan(Op&& op, T const& initial_value)
{
  using std::is_same;
  static_assert(is_same<Tag, tags::blelloch>::value,
                "The requested exclusive scan algorithm is not supported");

  return result_of::scan<tags::scan<Tag, tags::exclusive>, Op>(
           std::forward<Op>(op), initial_value);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_SCAN_HPP
