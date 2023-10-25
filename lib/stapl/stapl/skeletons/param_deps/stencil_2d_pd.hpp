/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_STENCIL_2D_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_STENCIL_2D_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief In a 2D stencil, parametric dependencies are defined in
/// such a way that each node depends on every 9 node in its surroundings
/// including itself from the previous timestep.
///
/// @tparam Op the operation to be applied at each point on the 9 points
///            received.
///
/// @ingroup skeletonsParamDepsInternal
///
/// @todo This version of the stencil parametric dependency should be
/// changed. Currently, all the neighboring cells of a cell are passed
/// in a linearized vector to the user's workfunction. In order to
/// distinguish these elements from each other a @c values_order is
/// also passed to the user's workfunction. This abstraction should be
/// ideally replaced with  a 2D view over the neighboring cells instead.
//////////////////////////////////////////////////////////////////////
template <typename Op>
class stencil_2d_pd
  : public param_deps_defaults
{
  Op                  m_op;
public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 1;

  using op_type      = Op;

  explicit stencil_2d_pd(Op op)
    : m_op(std::move(op))
  { }

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
    using stapl::get;
    auto index = get<0>(coord);
    auto dims = get<0>(skeleton_size);

    std::size_t i = get<0>(index);
    std::size_t j = get<1>(index);

    std::size_t m = get<0>(dims);
    std::size_t n = get<1>(dims);

    if (m == 1 and n == 1) {
      auto d = make_tuple(0, 0);
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              get<0>(in_flow).consume_from(make_tuple(d)));
    }
    else {
      std::vector<stapl::tuple<stapl::tuple<std::size_t, std::size_t>>> deps;
      deps.reserve(9);
      for (std::size_t i_off = 0; i_off < 3; ++i_off) {
        for (std::size_t j_off = 0; j_off < 3; ++j_off) {
          auto d = make_tuple((i + n - 1 + i_off) % n,
                              (j + m - 1 + j_off) % m);
          deps.push_back(make_tuple(d));
        }
      }

      const_cast<Op&>(m_op).set_values_order(deps);
      const_cast<Op&>(m_op).set_center(make_tuple(i, j));
      visitor(m_op,
              no_mapper(),
              get<0>(in_flow).consume_from_many(deps, stapl::identity_op()));
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
    /// it is a 9-point stencil computation
    return 9;
  }

  Op get_op() const
  {
    return m_op;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a 9-point stencil parametric dependency.
///
/// @copybrief skeletons_impl::stencil_2d_pd
///
/// @ingroup skeletonsParamDepsExchange
//////////////////////////////////////////////////////////////////////
template <typename Op>
skeletons_impl::stencil_2d_pd<Op>
stencil_2d_pd(Op const& op)
{
  return skeletons_impl::stencil_2d_pd<Op>(op);
}

} // namespace skeletons
} // namespace stapl

#endif //STAPL_SKELETONS_PARAM_DEPS_STENCIL_2D_PD_HPP
