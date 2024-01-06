/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_ALLGATHER_HPP
#define STAPL_SKELETONS_FUNCTIONAL_ALLGATHER_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include "gather.hpp"
#include "allreduce.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an allgather skeleton
/// by exposing only the necessary information in its representation.
///
/// An allgather skeleton gathers data from all partitions and
/// distributes the combined data to all partitions.
///
/// This abstraction not only makes the reconstruction of an
/// allgather skeleton easier, but also provides access to the
/// underlying operations of the enclosed reduction operation. Furthermore,
/// it reduces the symbol size for an allgather skeleton, hence, reducing
/// the total compilation time.
///
/// @tparam T       type of the element used in allgather
/// @tparam Span    the iteration space for elements on each level of
///                 both the reduction and the broadcast tree.
/// @tparam Tag     determines the type of the allgather skeleton
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Span, typename Tag>
struct allgather
  : public decltype(
             skeletons::allreduce<Tag, Span>(gather_impl::gather_op<T>()))
{
  using base_type = decltype(
                      skeletons::allreduce<Tag, Span>(
                        gather_impl::gather_op<T>()));

  using skeleton_tag_type = tags::allgather<Tag>;

  allgather(void)
    : base_type(
        skeletons::allreduce<Tag, Span>(gather_impl::gather_op<T>())
      )
  { }

  gather_impl::gather_op<T> get_op(void) const
  {
    return base_type::get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an allgather skeleton
/// by exposing only the necessary information in its representation.
///
/// This specialization uses a reversed_butterfly to implement
/// the allgather skeleton.
///
/// @tparam T       type of the element used in allgather
/// @tparam Span    the iteration space for elements on each level of
///                 both the reduction and the broadcast tree.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Span, bool B>
struct allgather<T, Span, tags::reverse_butterfly<B>>
  : public decltype(
             skeletons::reverse_butterfly<true>(gather_impl::gather_op<T>())
           )
{
  using base_type = decltype(
                      skeletons::reverse_butterfly<true>(
                        gather_impl::gather_op<T>()));

  using skeleton_tag_type = tags::allgather<tags::reverse_butterfly<B>>;

  allgather(void)
    : base_type(
        skeletons::reverse_butterfly<true>(gather_impl::gather_op<T>())
      )
  { }

  gather_impl::gather_op<T> get_op(void) const
  {
    return base_type::get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespace skeletons_impl

namespace result_of {

template <typename T, typename Tag, typename Span>
using allgather = skeletons_impl::allgather<
                    T, Span,
                    stapl::default_type<Tag, tags::left_aligned>>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Allgather skeleton is a function of type [[T]]->[[T]] which
/// is defined using allreduce with a concatenation operator.
///
/// An allgather skeleton gathers data from all partitions and
/// distributes the combined data to all partitions.
///
/// @tparam T            type of the element used in allgather
/// @tparam Span         the span to be used for @c reduce and
///                      @c broadcast skeletons
/// @param  tag          determines the type of allgather skeleton
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <typename T,
          typename Tag  = stapl::use_default,
          typename Span = stapl::use_default>
skeletons::result_of::allgather<T, Tag, Span>
allgather(void)
{
  static_assert(std::is_same<Tag, tags::left_aligned>::value  ||
                std::is_same<Tag, tags::right_aligned>::value ||
                std::is_same<Tag, tags::left_skewed>::value   ||
                std::is_same<Tag, tags::reverse_butterfly<false>>::value,
                "The supported types of allgather are left_aligned, "
                "right_aligned, left_skewed, and reverse_butterfly");
  return skeletons::result_of::allgather<T, Tag, Span>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_ALLGATHER_HPP
