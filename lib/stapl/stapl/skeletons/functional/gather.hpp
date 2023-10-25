/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_GATHER_HPP
#define STAPL_SKELETONS_FUNCTIONAL_GATHER_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include "allreduce.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

namespace gather_impl {

//////////////////////////////////////////////////////////////////////
/// @brief The concatenation of input vectors used in the @c gather
/// and @c allgather skeletons.
///
/// @tparma T type of the elments to be combined.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct gather_op
{
private:
  bool m_is_downedge;

public:

  using result_type = std::vector<T>;

  gather_op()
    : m_is_downedge(false)
  { }

  void set_position(std::size_t butterfly_size,
                    std::size_t index1,
                    std::size_t index2, std::size_t /* ignored */)
  {
    m_is_downedge = index1 >= index2;
  }

  template <typename V>
  result_type operator()(V&& v1, V&& v2)
  {
    if (m_is_downedge) {
      result_type res(v2);
      res.insert(res.end(), v1.begin(), v1.end());
      return res;
    }
    else {
      result_type res(v1);
      res.insert(res.end(), v2.begin(), v2.end());
      return res;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_is_downedge);
  }
};

} // namespace gather_impl

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a gather skeleton
/// by exposing only the necessary information in its representation.
///
/// An gather skeletons gathers data from all partitions into one.
///
/// This abstraction not only makes the reconstruction of a
/// gather skeleton easier, but also provides access to the
/// underlying operations of the enclosed reduction operation. Furthermore,
/// it reduces the symbol size for a gather skeleton, hence, reducing
/// the total compilation time.
///
/// @tparam T       type of the element used in gather
/// @tparam Span    the iteration space for elements on each level of
///                 both the reduction and the broadcast tree.
/// @tparam Tag     determines the type of the gather skeleton
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Span, typename Tag>
struct gather
  : public decltype(
             skeletons::reduce<Tag, Span>(gather_impl::gather_op<T>()))
{
  using skeleton_tag_type = tags::gather<Tag>;
  using base_type = decltype(
                      skeletons::reduce<Tag, Span>(gather_impl::gather_op<T>())
                    );

  gather(void)
    : base_type(
        skeletons::reduce<Tag, Span>(gather_impl::gather_op<T>()))
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

}

namespace result_of {

template<typename T,
         typename Tag,
         typename Span>
using gather = skeletons_impl::gather<
                 T, Span,
                 stapl::default_type<Tag, tags::left_aligned>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Allgather skeleton is a function of type [[T]]->[[T]] which
/// is defined using allreduce with a concatenation operator.
///
/// An gather skeletons, gathers data from all partitions and
/// distributes the combined data to all partitions.
///
/// @tparam T            type of the element used in gather
/// @tparam Span         the span to be used for @c reduce and
///                      @c broadcast skeletons
/// @param  tag          determines the type of gather skeleton
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <typename T,
          typename Tag  = stapl::use_default,
          typename Span = stapl::use_default>
result_of::gather<T, Tag, Span>
gather(void)
{
  static_assert(std::is_same<Tag, stapl::use_default>::value  ||
                std::is_same<Tag, tags::left_aligned>::value  ||
                std::is_same<Tag, tags::right_aligned>::value ||
                std::is_same<Tag, tags::left_skewed>::value,
                "The supported types of gather are left_aligned, "
                "right_aligned, and left_skewed");

  return result_of::gather<T, Tag, Span>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_GATHER_HPP
