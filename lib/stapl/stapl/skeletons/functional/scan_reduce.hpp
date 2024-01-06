/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_SCAN_REDUCE_HPP
#define STAPL_SKELETONS_FUNCTIONAL_SCAN_REDUCE_HPP

#include <stapl/skeletons/functional/copy.hpp>
#include <stapl/skeletons/functional/reduce_to_locs.hpp>
#include <stapl/skeletons/functional/scan.hpp>
#include <stapl/skeletons/flows/inline_flows.hpp>
#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename ValueType, typename Tag, typename Alg, typename Op>
class scan_reduce;

namespace ph = stapl::skeletons::flows::inline_flows::placeholders;

template <typename ValueType, typename Tag, typename Op>
class scan_reduce<ValueType, Tag, tags::exclusive, Op>
  : public decltype(skeletons::compose<skeletons::tags::inline_flow>(
      ph::x<0>() << skeletons::scan<Tag>(std::declval<Op>(),
                                         std::declval<ValueType>()) |
        ph::input<0>(),
      ph::x<1>() << skeletons::copy<ValueType>() | (ph::x<0>(), ph::input<1>()),
      ph::x<2>() << skeletons::reduce_to_locs<true>(std::declval<Op>()) |
        ph::input<0>()))
{
  ValueType m_init_value;
public:
  using op_type = typename std::decay<Op>::type;
  using skeleton_tag_type = tags::scan_reduce<Tag, tags::exclusive>;
  using value_t = typename scan_operand_type<Op>::type;
  using base_type = decltype(skeletons::compose<skeletons::tags::inline_flow>(
    ph::x<0>() << skeletons::scan<Tag>(std::declval<Op>(),
                                       std::declval<ValueType>()) |
      ph::input<0>(),
    ph::x<1>() << skeletons::copy<ValueType>() | (ph::x<0>(), ph::input<1>()),
    ph::x<2>() << skeletons::reduce_to_locs<true>(std::declval<Op>()) |
      ph::input<0>()));

  scan_reduce(Op&& op, ValueType const& initial_value)
    : base_type(skeletons::compose<skeletons::tags::inline_flow>(
        ph::x<0>() << skeletons::scan<Tag>(std::forward<Op>(op),
                                           initial_value) |
          ph::input<0>(),
        ph::x<1>() << skeletons::copy<value_t>() | (ph::x<0>(), ph::input<1>()),
        ph::x<2>() << skeletons::reduce_to_locs<true>(std::forward<Op>(op)) |
          ph::input<0>())),
      m_init_value(initial_value)
  { }

  op_type get_op() const
  {
    return base_type::template get_skeleton<0>().get_op();
  }

  value_t get_init_value() const
  {
    return m_init_value;
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_init_value);
  }
};


template <typename ValueType, typename Tag, typename Op>
class scan_reduce<ValueType, Tag, tags::inclusive, Op>
  : public decltype(skeletons::compose<skeletons::tags::inline_flow>(
      ph::x<0>() << skeletons::scan<Tag>(std::declval<Op>()) | ph::input<0>(),
      ph::x<1>() << skeletons::copy<ValueType>() | (ph::x<0>(), ph::input<1>()),
      ph::x<2>() << skeletons::reduce_to_locs<true>(std::declval<Op>()) |
        ph::input<0>()))
{
public:
  using op_type = typename std::decay<Op>::type;
  using skeleton_tag_type = tags::scan_reduce<Tag, tags::inclusive>;
  using value_t = typename scan_operand_type<Op>::type;
  using base_type = decltype(skeletons::compose<skeletons::tags::inline_flow>(
    ph::x<0>() << skeletons::scan<Tag>(std::declval<Op>()) | ph::input<0>(),
    ph::x<1>() << skeletons::copy<ValueType>() | (ph::x<0>(), ph::input<1>()),
    ph::x<2>() << skeletons::reduce_to_locs<true>(std::declval<Op>()) |
      ph::input<0>()));

  scan_reduce(Op&& op)
    : base_type(skeletons::compose<skeletons::tags::inline_flow>(
        ph::x<0>() << skeletons::scan<Tag>(std::forward<Op>(op)) |
          ph::input<0>(),
        ph::x<1>() << skeletons::copy<value_t>() | (ph::x<0>(), ph::input<1>()),
        ph::x<2>() << skeletons::reduce_to_locs<true>(std::forward<Op>(op)) |
          ph::input<0>()))
  { }

  op_type get_op() const
  {
    return base_type::template get_skeleton<0>().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespaceskeletons_impl

namespace result_of {

template <typename ValueType, typename Tag, typename Alg, typename Op>
using scan_reduce = skeletons_impl::scan_reduce<ValueType, Tag, Alg, Op>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief This skeleton combines the functionality of @c inclusive_scan
///        skeleton and @c reduce_to_locs skeletons.
///
/// This skeleton is useful in cases when we both need the result of
/// scan over the inputs and the reduction of input values. It avoids
/// doing an extra iteration over the data.
///
/// @param  op            the operation to be used to compute the scan
///                       results
/// @tparam Tag           the type of exclusive scan to be used
///
/// @return an exclusive scan specified by the @c Tag
///
/// @see inclusive_scan
/// @see scan
/// @see reduce_to_locs
///
/// @ingroup skeletonsFunctionalAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename ValueType,
          typename Tag = tags::binomial,
          typename Op>
result_of::scan_reduce<ValueType, Tag, tags::inclusive, Op>
scan_reduce(Op&& op)
{
  using std::is_same;
  static_assert(
    is_same<Tag, tags::binomial>::value      ||
    is_same<Tag, tags::hillis_steele>::value ||
    is_same<Tag, tags::jaja>::value,
    "The requested inclusive scan algorithm is not supported");

  return result_of::scan_reduce<ValueType,
                                Tag,
                                tags::inclusive,
                                Op>(std::forward<Op>(op));
}

//////////////////////////////////////////////////////////////////////
/// @brief This skeleton combines the functionality of @c exclusive_scan
///        skeleton and @c reduce_to_locs skeletons.
///
/// This skeleton is useful in cases when we both need the result of
/// scan over the inputs and the reduction of input values. It avoids
/// doing an extra iteration over the data.
///
/// @param  op            the operation to be used to compute the scan
///                       results
/// @param  initial_value the value for the first element of the output
/// @tparam Tag           the type of exclusive scan to be used
///
/// @return an exclusive scan specified by the @c Tag
///
/// @see exclusive_scan
/// @see scan
/// @see reduce_to_locs
///
/// @ingroup skeletonsFunctionalAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename ValueType,
          typename Tag = tags::blelloch,
          typename Op>
result_of::scan_reduce<ValueType, Tag, tags::exclusive, Op>
scan_reduce(Op&& op, ValueType const& initial_value)
{
  return result_of::scan_reduce<ValueType,
                                Tag,
                                tags::exclusive,
                                Op>(std::forward<Op>(op), initial_value);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_SCAN_REDUCE_HPP
