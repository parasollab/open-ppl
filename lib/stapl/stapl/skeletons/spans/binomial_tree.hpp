/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_BINOMIAL_TREE_HPP
#define STAPL_SKELETONS_SPANS_BINOMIAL_TREE_HPP

#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

/////////////////////////////////////////////////////////////////////////
/// @brief A span for binomial trees which is used in algorithms such
/// as binomial tree scan.
///
/// A binomial tree based algorithm consists of two phases: an up-phase
/// and a down-phase. The up-phase looks similar to a @c right_aligned
/// tree if the input size is power-of-two. In the case of arbitrary size
/// inputs the @c right_aligned tree is extended with partial right_trees
/// in order to make the up-phase. Both phases for an input of size 10 can
/// be depicted as:
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
/// @tparam OnSpan the span on which this tree is defined
/// @tparam Phase  phase of the binomial tree (up_phase or down_phase)
/// @see inclusive_scan(op,tags::binomial_tree)
///
/// @ingroup skeletonsSpans
/////////////////////////////////////////////////////////////////////////
template <typename OnSpan, typename Phase>
struct binomial_tree;


/////////////////////////////////////////////////////////////////////////
/// @brief The up-phase span for the up-phase of a binomial tree.
///
/// @tparam OnSpan the span on which this tree is defined
/// @see spans::binomail_tree
///
/// @ingroup skeletonsSpans
/////////////////////////////////////////////////////////////////////////
template<typename OnSpan>
struct binomial_tree<OnSpan, tags::up_phase>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const&, Coord const& coord) const
  {
    std::size_t index = tuple_ops::front(coord);
    std::size_t level = stapl::get<1>(coord);

    // Only spawn if (index & (2^(level+1) - 1) = 2^(level+1)-1
    std::size_t mask = (1ul << (level+1)) - 1;
    return (mask & index) == mask;
  }
};


/////////////////////////////////////////////////////////////////////////
/// @brief The down-phase span for the down-phase of a binomial tree.
///
/// @tparam OnSpan the span on which this tree is defined
/// @see spans::binomail_tree
///
/// @ingroup skeletonsSpans
/////////////////////////////////////////////////////////////////////////
template<typename OnSpan>
struct binomial_tree<OnSpan, tags::down_phase>
  : public OnSpan
{
  using size_type      = typename OnSpan::size_type;
  using dimension_type = typename OnSpan::dimension_type;

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    std::size_t index = tuple_ops::front(coord);
    std::size_t level = stapl::get<1>(skeleton_size) - stapl::get<1>(coord) - 1;

    // Only spawn if index & (2^(level) - 1) = 2^level - 1
    std::size_t mask = (1ul << level) - 1;
    return (index & mask) == mask;
  }
};

} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_BINOMIAL_TREE_HPP
