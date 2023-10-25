/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_ALLTOALL_HYBRID_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_ALLTOALL_HYBRID_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include "alltoall_helpers.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief The hybrid @c alltoall parametric dependency is used
/// for alltoall skeleton in the cases of having large messages. The
/// parametric dependency in this case is defined as a hybrid of the
/// flat and butterfly-based version of alltoall.
///
/// In this case, there are log2(n) stages. In each stage,
/// each participant at the upper half of a butterfly communicates with
/// every participant in the lower half of the same butterfly, and vice versa.
///
/// @note The dependencies at each step are permuted to avoid network pressure
/// on a single node at the same time.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename T>
class alltoall_pd<T, tags::hybrid>
  : public param_deps_defaults
{
public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using consumer_count_filter_type = skeletons::filters::filter<2>;
  using op_type                    = alltoall_merge<T, tags::hybrid>;

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

    if (stapl::get<0>(skeleton_size) == 1) {
      visitor(stapl::identity<std::vector<T>>(), no_mapper(),
              stapl::get<0>(in_flow).consume_from(stapl::make_tuple(0)));
    }
    else {
      std::size_t const i = tuple_ops::front(coord);
      std::size_t const bflied_index = butterflied_index(skeleton_size, coord);
      std::size_t const bfly_size = butterfly_size(skeleton_size, coord);

      std::size_t const offset = bflied_index  - bflied_index % bfly_size;
      std::size_t const max = offset + bfly_size;

      std::vector<stapl::tuple<std::size_t>> deps;
      deps.reserve(bfly_size);

      for (std::size_t i = offset; i < max; ++i) {
        deps.push_back(stapl::make_tuple(i));
      }
      std::random_shuffle(deps.begin(), deps.end());

      visitor(alltoall_merge<T, tags::hybrid>(deps),
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(stapl::make_tuple(i)),
              stapl::get<0>(in_flow).consume_from_many(
                deps, alltoall_filter<T, tags::hybrid>(i)));
    }
  }

  template <typename Coord>
  std::size_t
  butterfly_size(Coord const& skeleton_size, Coord const& coord) const
  {
    return 1ul << (stapl::get<1>(skeleton_size) - stapl::get<1>(coord) - 1);
  }

  template <typename Coord>
  std::size_t
  butterflied_index(Coord const& skeleton_size, Coord const& coord) const
  {
    std::size_t cur_index = stapl::get<0>(coord);
    std::size_t bfly_size = butterfly_size(skeleton_size, coord);
    const int direction = cur_index % (bfly_size * 2) >= bfly_size ? -1 : 1;
    return (cur_index + direction * bfly_size);
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
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_index*/) const
  {
    if (tuple_ops::front(skeleton_size) == 1) {
      return 1;
    }
    else {
      //the number of consumers are set as the butterfly size
      //since we are reading from the consumer side, it is similar to the
      //original butterfly size + 1
      std::size_t depth = static_cast<std::size_t>(
                            log(stapl::get<0>(skeleton_size)) / log(2));

      return (1ul << (depth - stapl::get<1>(producer_coord) - 2)) + 1;
    }
  }
};

} // namespace skeletons_impl
} // namespace skeletons
} // namespace stapl

#endif //STAPL_SKELETONS_PARAM_DEPS_ALLTOALL_HYBRID_PD_HPP
