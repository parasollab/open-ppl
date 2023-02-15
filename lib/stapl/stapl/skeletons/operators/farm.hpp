/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_FARM_HPP
#define STAPL_SKELETONS_FUNCTIONAL_FARM_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/farm_pd.hpp>
#include <stapl/skeletons/spans/misc.hpp>

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename Generator, typename Span>
using farm = result_of::elem<
               stapl::default_type<Span, spans::only_once>,
               stapl::use_default,
               skeletons_impl::farm_pd<
                 typename std::decay<Generator>::type>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Given a generator, creates a farm skeleton. The generator
/// can then generate the subsequent tasks of the program.
///
/// An example of this type of farm can be used in the implementation
/// of BFS (Breadth First Search), in which the initial seed define
/// the starting point of the program.
///
/// @tparam Span      the iteration space for the elements in this
///                   skeleton. The default value is only once. Other
///                   spans can be used. In such cases the redundant
///                   additions of elements to farm should be handled
///                   by the generator.
/// @param  generator the generator to be used to create the initial
///                   seeds for this farm. The generator receives
///                   user provides the farm as the first argument to
///                   the generator.
///
/// @see spans::only_once
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Span = spans::only_once,
          typename Generator>
result_of::farm<Generator, Span>
farm(Generator&& generator)
{
  return result_of::farm<Generator, Span>(
           farm_pd(std::forward<Generator>(generator)));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_FARM_HPP
