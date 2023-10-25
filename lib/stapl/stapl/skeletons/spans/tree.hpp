/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_TREE_HPP
#define STAPL_SKELETONS_SPANS_TREE_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

template <typename OnSpan, typename Alignment>
struct tree;

template <typename OnSpan, typename Alignment>
struct reverse_tree;

//////////////////////////////////////////////////////////////////////
/// @brief This is a span for a left-skewed binary tree that looks
/// like the following:
/// @code
/// O O O O O O O O
/// |/ _|/  |/  |/
/// | |  ___|   |
/// | | |  _____|
/// | | | |
/// O O O O
/// |/ _|/
/// | |
/// O O
/// |/
/// O
///
/// @endcode
///
/// @tparam OnSpan the span on which this tree is defined
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct tree<OnSpan, tags::left_skewed>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    stapl_assert(
      (tuple_ops::front(skeleton_size) == 1) ||
      (1ul << stapl::get<1>(skeleton_size) == tuple_ops::front(skeleton_size)),
      "Skeleton size should be power of two");
    const std::size_t tree_depth = stapl::get<1>(skeleton_size);
    const std::size_t current_depth = stapl::get<1>(coord);

    return stapl::get<0>(coord) <
             (std::size_t(1) << (tree_depth - current_depth - 1));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A span for right-aligned binary tree that looks like the
/// following:
/// @code
/// O O O O O O O O
///  \|  \|  \|  \|
///   O__ O   O__ O
///      \|      \|
///       O______ O
///              \|
///               O
/// @endcode
///
/// @tparam OnSpan the span on which this right tree is defined
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct tree<OnSpan, tags::right_aligned>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    stapl_assert(
      (stapl::get<0>(skeleton_size) == 1) ||
      (1ul << stapl::get<1>(skeleton_size) == stapl::get<0>(skeleton_size)),
      "Skeleton size should be power of two");
    const std::size_t current_depth = stapl::get<1>(coord);

    return (stapl::get<0>(skeleton_size) == 1) ||
           ((stapl::get<0>(coord) + 1) %
            (std::size_t(1) << (current_depth + 1)) == 0);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A span for a left-aligned reverse binary tree that looks
/// like the following:
/// @code
/// O O O O O O O O
/// |/  |/  |/  |/
/// O __O   O __O
/// |/      |/
/// O ______O
/// |/
/// O
/// @endcode
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct tree<OnSpan, tags::left_aligned>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    stapl_assert(
      (stapl::get<0>(skeleton_size) == 1) ||
      (1ul << stapl::get<1>(skeleton_size) == stapl::get<0>(skeleton_size)),
      "Skeleton size should be power of two");
    const std::size_t current_depth = stapl::get<1>(coord);

    return (stapl::get<0>(coord) %
             (std::size_t(1) << (current_depth + 1))) == 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A span for a left-skewed reverse binary tree that looks
/// like the following:
/// @code
/// O
/// |\.
/// O O
/// | |_
/// |\  |\.
/// O O O O
/// | | | |_____
/// | | |___    |
/// | |_    |   |
/// |\  |\  |\  |\.
/// O O O O O O O O
/// @endcode
///
/// @tparam OnSpan the span on which this tree is defined
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct reverse_tree<OnSpan, tags::left_skewed>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    stapl_assert(
      (stapl::get<0>(skeleton_size) == 1) ||
        (1ul << stapl::get<1>(skeleton_size) == stapl::get<0>(skeleton_size)),
      "Skeleton size should be power of two");
    const std::size_t current_depth = stapl::get<1>(coord);
    //remember that the first level of the tree is provided by the previous
    //skeleton that this one is connected to
    return stapl::get<0>(coord) < (1ul << (current_depth + 1));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A span for a right-aligned reverse binary tree that looks
/// like the following:
/// @code
///               O
///        ______/|
///       O       O
///    __/|    __/|
///   O   O   O   O
///  /|  /|  /|  /|
/// O O O O O O O O
/// @endcode
///
/// @tparam OnSpan the span on which this tree is defined
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct reverse_tree<OnSpan, tags::right_aligned>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    stapl_assert(
      (stapl::get<0>(skeleton_size) == 1) ||
      (1ul << stapl::get<1>(skeleton_size) == stapl::get<0>(skeleton_size)),
      "Skeleton size should be power of two");
    const std::size_t current_depth = stapl::get<1>(coord);
    const std::size_t depth = stapl::get<1>(skeleton_size);
    //remember that the first level of the tree is provided by the previous
    //skeleton that this one is connected to
    return (stapl::get<0>(skeleton_size) == 1) ||
           ((stapl::get<0>(coord) + 1) %
            (std::size_t(1) << (depth - current_depth - 1)) == 0);
  }
};


/////////////////////////////////////////////////////////////////////////
/// @brief A span for a left-aligned reverse binary tree that look like
/// the following:
/// @code
/// O
/// |\______
/// O       O
/// |\__    |\__
/// O   O   O   O
/// |\  |\  |\  |\.
/// O O O O O O O O
/// @endcode
///
/// @tparam OnSpan the span on which this tree is defined
///
/// @ingroup skeletonsSpans
/////////////////////////////////////////////////////////////////////////
template <typename OnSpan>
struct reverse_tree<OnSpan, tags::left_aligned>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    stapl_assert(
      (stapl::get<0>(skeleton_size) == 1) ||
      (1ul << stapl::get<1>(skeleton_size) == stapl::get<0>(skeleton_size)),
      "Skeleton size should be power of two");
    const std::size_t current_depth = stapl::get<1>(coord);
    const std::size_t depth = stapl::get<1>(skeleton_size);
    //remember that the first level of the tree is provided by the previous
    //skeleton that this one is connected to
    return (stapl::get<0>(coord) %
              (std::size_t(1) << (depth - current_depth - 1))) == 0;
  }
};

} // namespace spans
} // namespace skeletons
} // namepsace stapl

#endif // STAPL_SKELETONS_SPANS_TREE_HPP
