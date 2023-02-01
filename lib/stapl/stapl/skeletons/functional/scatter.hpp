/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_SCATTER_HPP
#define STAPL_SKELETONS_FUNCTIONAL_SCATTER_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/algorithms/functional.hpp>
#include "reverse_binary_tree.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

namespace scatter_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A scatter skeleton based on @c reverse_binary_tree is
/// a no-op. The filtering of the input to this operation is done
/// with the help of filters to reduce communication.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct scatter_op
  : public stapl::identity<std::vector<T>>
{
  void set_position(std::size_t index, bool is_downedge) { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Filters the inputs in scatter tree into two halves. If the
/// edge requesting the value from this filter is a downedge then
/// the upper half of the input is returned. If not, the lower-half is
/// returned
///
/// @tparam T the value type of the elements being scattered
//////////////////////////////////////////////////////////////////////
template <typename T>
struct scatter_filter
{
  using result_type = std::vector<T>;
private:
  bool m_is_downedge;

public:
  scatter_filter(void)
    : m_is_downedge(false)
  { }

  void set_position(std::size_t, bool is_downedge)
  {
    m_is_downedge = is_downedge;
  }

  template <typename V>
  result_type operator()(V&& v) const
  {
    std::size_t size = v.size();

    // first we find the point we would like to divide it in half. If size
    // is a power of two, then it is easy. If not we group the first
    // 2 * size % nearest_pow_two elements in pairs of two. For example,
    // for 6 elements we would have
    // o o o o o o -> (o o) (o o) o o

    std::size_t half = size / 2;
    //if it is not power of two
    if (size != 0 and (size & (size-1)) != 0) {
      std::size_t nearest_pow_2 = 1;
      std::size_t n = size;

      while (n != 1) {
        nearest_pow_2 <<= 1;
        n >>= 1;
      }

      std::size_t r = size - nearest_pow_2;
      half = 2 * r;
    }

    auto&& begin_it = v.begin();
    auto&& end_it = v.end();
    if (m_is_downedge) {
      std::advance(begin_it, half);
    }
    else {
      end_it = begin_it;
      std::advance(end_it, half);
    }

    return result_type(begin_it, end_it);
  }

  bool operator==(scatter_filter const& other) const
  {
    return m_is_downedge == other.m_is_downedge;
  }

  void define_type(typer& t)
  {
    t.member(m_is_downedge);
  }
};

} // namespace scatter_impl

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a scatter skeleton
/// by exposing only the necessary information in its representation.
/// A scatter skeleton is simply a reverse binary tree with an operation
/// which splits the input into two halves.
///
/// This abstraction not only makes the reconstruction of a scatter
/// skeleton easier, but also provides access to the underlying
/// operation of a broadcast skeleton. Furthermore, it reduces the
/// symbol size for a scatter skeleton, hence, reducing the
/// total compilation time.
///
/// @tparam T the type of elements to be scattered
/// @tparam Span      the iteration space for elements to be scattered
/// @tparam Tag       determines the type of scatter
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename T, typename Span, typename Tag>
struct scatter
  : public decltype(
             skeletons::reverse_binary_tree<
               false, Tag, stapl::use_default, Span, false, true
             >(scatter_impl::scatter_op<T>(),
               scatter_impl::scatter_filter<T>())
           )
{
  using skeleton_tag_type = tags::scatter<Tag>;
  using base_type = decltype(
                      skeletons::reverse_binary_tree<
                        false, Tag, stapl::use_default, Span, false, true
                      >(scatter_impl::scatter_op<T>(),
                        scatter_impl::scatter_filter<T>()));

  scatter(void)
    : base_type(
        skeletons::reverse_binary_tree<
          false, Tag, stapl::use_default, Span, false, true
        >(scatter_impl::scatter_op<T>(),
          scatter_impl::scatter_filter<T>())
      )
  { }

  scatter_impl::scatter_op<T>
  get_op(void) const
  {
    return base_type::get_op();
  }

  scatter_impl::scatter_filter<T>
  get_filter(void) const
  {
    return base_type::get_filter();
  }
};

} // namespace skeletons_impl

namespace result_of {

template <typename T,
          typename Tag,
          typename Span>
using scatter = skeletons_impl::scatter<
                 T, Span,
                 stapl::default_type<Tag, tags::left_aligned>>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief This sink skeleton assumes a default span for the created
/// skeleton
///
/// @tparam T             the type of elements to be copied
/// @param  skeleton      the skeleton to read the input from
/// @param  dest_skeleton a customized sink skeleton. By default this
///                      is assumed to be a copy skeleton
/// @return a sink skeleton with a customized destination skeleton
///
/// @see copy
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename T,
          typename Tag  = stapl::use_default,
          typename Span = stapl::use_default>
result_of::scatter<T, Tag, Span>
scatter(void)
{
  static_assert(std::is_same<Tag, stapl::use_default>::value  ||
                std::is_same<Tag, tags::left_aligned>::value  ||
                std::is_same<Tag, tags::right_aligned>::value ||
                std::is_same<Tag, tags::left_skewed>::value,
                "The supported types of scatter are left_aligned, "
                "right_aligned, and left_skewed");


  return result_of::scatter<T, Tag, Span>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_SCATTER_HPP
