/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_TREE_HPP
#define STAPL_SKELETONS_FUNCTIONAL_TREE_HPP

#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/lazy_sizes.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/tree.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/operators/repeat.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a k-ary tree
/// by exposing only the necessary information in its representation.
///
/// A k-ary tree reduces in size by k as the levels are increased.
///
/// This abstraction not only makes the reconstruction of a
/// a k-ary skeleton easier, but also reduces the symbol size for a
/// k-ary tree skeleton, hence, reducing the total compilation time.
///
/// @tparam PD      the underlying parametric dependency used in each
///                 level of the tree.
/// @tparam Arity   the arity of the tree.
/// @tparam Flows   the flow between the levels of the k-ary tree.
/// @tparam Span    the iteration space for the elements in each level.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename PD, int Arity, typename Flows, typename Span>
struct tree
  : public decltype(
             skeletons::repeat<Flows>(
               skeletons::elem<Span>(std::declval<PD>()),
               log_lazysize<Arity>()))
{
  using skeleton_tag_type = tags::tree<Arity>;
  using base_type = decltype(
                      skeletons::repeat<Flows>(
                        skeletons::elem<Span>(std::declval<PD>()),
                        log_lazysize<Arity>()));

  explicit tree(PD const& pd)
    : base_type(
        skeletons::repeat<Flows>(
          skeletons::elem<Span>(pd),
          log_lazysize<Arity>()
        )
      )
  { }

  auto get_op(void) const ->
    decltype(
      std::declval<base_type>().nested_skeleton().nested_skeleton().get_op()
    )
  {
    return base_type::nested_skeleton().nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a reverse k-ary tree
/// by exposing only the necessary information in its representation.
///
/// A k-ary reverse tree expands in size by k starting from 1 as the
/// levels are increased.
///
/// This skeleton is used in several composed skeletons such as
/// @c broadcast skeleton.
///
/// This abstractions not only makes the reconstruction of a
/// a k-ary skeleton easier, but also reduces the symbol size for a
/// k-ary tree skeleton, hence, reducing the total compilation time.
///
/// @tparam PD the underlying parametric dependency used in each
///                 level of the reverse tree.
/// @tparam Arity   the arity of the reverse tree.
/// @tparam Flows   the flow between the levels of the k-ary reverse tree.
/// @tparam Span    the iteration space for the elements in each level.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename PD, int Arity, typename Flows, typename Span>
struct reverse_tree
  : public decltype(
             skeletons::repeat<Flows>(
               skeletons::elem<Span>(std::declval<PD>()),
               log_lazysize<Arity>()
             )
           )
{
  using skeleton_tag_type = tags::reverse_tree<Arity>;
  using base_type = decltype(
                      skeletons::repeat<Flows>(
                        skeletons::elem<Span>(std::declval<PD>()),
                        log_lazysize<Arity>()));

  explicit reverse_tree(PD const& pd)
    : base_type(
        skeletons::repeat<Flows>(
           skeletons::elem<Span>(pd),
           log_lazysize<Arity>()
        )
      )
  { }

  auto get_op(void) const ->
    decltype(
      std::declval<base_type>().nested_skeleton().nested_skeleton().get_op()
    )
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

template <int      Arity,
          typename Flows,
          typename Span,
          typename PD>
using tree = skeletons_impl::tree<
               typename std::decay<PD>::type, Arity, Flows,
               stapl::default_type<
                 Span,
                 spans::tree<spans::balanced<>, tags::left_aligned>>>;

template <int      Arity,
          typename Flows,
          typename Span,
          typename PD>
using reverse_tree = skeletons_impl::reverse_tree<
                       typename std::decay<PD>::type, Arity, Flows,
                       stapl::default_type<
                         Span,
                         spans::reverse_tree<
                           spans::balanced<>, tags::left_aligned>>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A tree skeleton is used in many skeletons such as reduce
/// a tree consists of a set of levels with shrinking sizes.
///
/// There are variations of trees based on their span including but
/// not limited to:
/// @li @c tree
/// @li @c right_tree
/// @li @c left-tree
///
/// @tparam Arity   the arity of the tree. The default value is 2
/// @tparam Flows   the flow to be used for the @c tree. Some skeletons
///                 need special flows.
/// @tparam Span    the iteration space for elements on each level of
///                 the tree
/// @param  pd      the parametric dependency to be used in the nodes of
///                 this tree
/// @return an n-ary tree with customized flow and span
///
/// @see spans::tree
/// @see spans::left_aligned
/// @see spans::right_aligned
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <int Arity      = 2,
          typename Flows = stapl::use_default,
          typename Span  = stapl::use_default,
          typename PD>
result_of::tree<Arity, Flows, Span, PD>
tree(PD&& pd)
{
  return result_of::tree<Arity, Flows, Span, PD>(std::forward<PD>(pd));
}

//////////////////////////////////////////////////////////////////////
/// @brief A reverse tree skeleton is used in many skeletons such as
/// broadcast. A reverse tree consists of a set of levels with
/// expanding sizes.
///
/// There are variations of trees based on their span including but
/// not limited to:
/// @li @c reverse_tree
/// @li @c right_reverse_tree
/// @li @c left_reverse_tree
///
/// @tparam Arity   the arity of the reverse_tree. The default value is
///                 2
/// @tparam Flows   the flow to be used for the reverse tree. Some
///                 skeletons need special flows.
/// @tparam Span    the iteration space for elements on each level of
///                 the reverse tree
/// @param  pd      the parametric dependency to be used in the nodes of
///                 this reverse tree
/// @return an n-ary reverse tree with customized flow and span
///
/// @see spans::tree
/// @see spans::left_aligned
/// @see spans::right_aligned
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <int Arity      = 2,
          typename Flows = stapl::use_default,
          typename Span  = stapl::use_default,
          typename PD>
result_of::reverse_tree<Arity, Flows, Span, PD>
reverse_tree(PD&& pd)
{
  return result_of::reverse_tree<Arity, Flows, Span, PD>(std::forward<PD>(pd));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_TREE_HPP
