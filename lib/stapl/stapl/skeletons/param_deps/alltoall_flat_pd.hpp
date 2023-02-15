/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_ALLTOALL_FLAT_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_ALLTOALL_FLAT_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include "alltoall_helpers.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief In a flat alltoall, parametric dependencies are defined in
/// such a way that each node depends on every node in the producer.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename T>
class alltoall_pd<T, tags::flat>
  : public param_deps_defaults
{
public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using op_type = alltoall_merge<T, tags::flat>;

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, h, ...> it wraps the @c WF with the
  /// following inputs and sends it to the visitor along with the @c m_wf
  /// @li in<0>[0, ..., n]
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about WF and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    std::size_t index = tuple_ops::front(coord);
    std::size_t n = tuple_ops::front(skeleton_size);

    if (stapl::get<0>(skeleton_size) == 1) {
      visitor(stapl::identity<std::vector<T>>(), no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(0)));
    }
    else {
      std::vector<stapl::tuple<std::size_t>> deps;

      deps.reserve(n);

      for (std::size_t i = 0; i < n; ++i) {
        if (i != index){
          deps.push_back(make_tuple(i));
        }
      }
      std::random_shuffle(deps.begin(), deps.end());

      visitor(alltoall_merge<T, tags::flat>(deps),
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(index)),
              stapl::get<0>(in_flow).consume_from_many(
                                       deps,
                                       alltoall_filter<T, tags::flat>(index)));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @param  skeleton_size  the size of this skeleton
  /// @param  producer_coord the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const& skeleton_size,
                             Coord const& /*producer_coord*/,
                             FlowIndex const& /*flow_index*/) const
  {
    return stapl::get<0>(skeleton_size);
  }
};

} // namespace skeletons_impl
} // namespace skeletons
} // namespace stapl

#endif //STAPL_SKELETONS_PARAM_DEPS_ALLTOALL_FLAT_PD_HPP
