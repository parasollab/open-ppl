/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_STENCIL_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_STENCIL_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/position.hpp>
#include <stapl/skeletons/utility/nd_traits.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include <stapl/skeletons/param_deps/stencil_utils.hpp>

namespace stapl {
namespace skeletons {

namespace skeletons_impl {

using stencil_impl::point;

//////////////////////////////////////////////////////////////////////
/// @brief Base class for stencil parametric dependencies that provides
/// all necessary member variables as well as their accessors. Child
/// classes only need to provide an output_type metafunction, the
/// case_of() and consumer_count() member functions, and in_port_size.
///
/// @see stencil_pd
///
/// @note This class is meant to reduce symbol size, especially when
/// both periodic and nonperiodic versions of a stencil are instantiated.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename F, int N, int numPoints>
struct stencil_pd_base
  : public param_deps_defaults
{
protected:
  Op        m_op;
  F         m_filter;

public:
  static constexpr size_t center_port_index = 0;
  using filter_type = F;
  using op_type     = Op;

  stencil_pd_base(Op op, F f)
    : m_op(std::move(op)),
      m_filter(std::move(f))
  { }

  Op get_op() const
  {
    return m_op;
  }

  F get_filter() const
  {
    return m_filter;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_filter);
  }

protected:
  static constexpr size_t P = (numPoints - 1)/N/2;

  using dimensions  = make_index_sequence<N>;
  using offsets     = make_index_sequence<P>;

  //Helper that returns a copy of m_filter with the direction set
  template<typename Pt>
  F filter_for(Pt p) const
  {
    using stencil_utils::get_stencil_filter;
    return get_stencil_filter(p, m_filter, dimensions());
  }

};

/////////////////////////////////////////////////////////////////////
/// @brief An n-D, 2np+1 point stencil is one where each node depends
/// on itself and p nodes in the negative and positive directions for
/// each dimension.
///
/// @tparam Op the operation to be applied at each point of the points
///            received
/// @tparam F  the filter function to be applied on the user side
/// @tparam isPeriodic determines whether dependency between points is
///         periodic or not for boundary points
/// @tparam N specifies the number of input dimensions (1D,
///         2D, ... , nD)
/// @tparam numPoints  specifies the total number of points each node
///         reads from, including itself
/// @ingroup skeletonsParamDepsInternal
///
//////////////////////////////////////////////////////////////////////
template <typename Op, typename F, bool isPeriodic, int N,
  int numPoints = 2*N + 1>
class stencil_pd;

template <typename Op, typename F, int N, int numPoints>
class stencil_pd<Op, F, false, N,  numPoints>
  : public stencil_pd_base<Op, F, N, numPoints>
{
  using base_type = stencil_pd_base<Op, F, N, numPoints>;
  using typename base_type::dimensions;
  using typename base_type::offsets;
  using points    = stencil_impl::traversal_order<dimensions, offsets>;

  using base_type::P;
  using base_type::center_port_index;

  //compute the boundary port for a direction
  static constexpr size_t boundary_port(int sign, size_t n)
  {
    return (sign < 0 ? 1 : 2) + 2*n;
  }

public:
  static constexpr std::size_t in_port_size = 2*N + 1;
  static constexpr std::size_t op_arity     = numPoints;

  using op_type = Op;

  using typename base_type::filter_type;

  stencil_pd(Op const& op, F const& f)
    : base_type(op, f)
  { }

  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const& skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_index*/) const
  {
    auto m = tuple_ops::front(skeleton_size);
    auto i = tuple_ops::front(producer_coord);
    return apply_consumer_count(i, m, points());
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, j, ...> it passes the @c Op with the
  /// following inputs and to the visitor along with the @c m_op
  /// @li in<0>[0, ..., n]
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                      potentially multi-dimensional
  /// @param coord         <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor       the information about WF and input is passed
  ///                      so that later this information can be
  ///                      converted to a node in the dependence graph
  /// @param in_flow       a tuple of input flows to consume from
  ////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord&& skeleton_size, Coord&& coord, Visitor&& visitor,
    In&& in_flow) const
  {
    const auto idxv = nd_get<0>(coord);
    const auto idx  = make_tuple(idxv);
    const auto dim  = nd_get<0>(skeleton_size);

    visit_left(visitor, idx, dim, std::forward<In>(in_flow), dimensions(),
      offsets(), get<center_port_index>(in_flow).consume_from(idx));
  }

protected:

  //base case: no other points to read, apply visitor
  template<typename Visitor, typename Coord, typename Size, typename In,
    typename... Args>
  void visit_right(Visitor& v, Coord const&, Size const&, In&&,
      index_sequence<>, offsets, Args&&... args) const
  {
    v(this->m_op, no_mapper(), std::forward<Args>(args)...);
  }

  //base case: no points on left to read, go to right
  template<typename Visitor, typename Coord, typename Size, typename In,
    typename... Args>
  void visit_left(Visitor& v, Coord const& idx, Size const& size, In&& in_flow,
      offsets, Args&&... args) const
  {
    visit_left(v, idx, size, std::forward<In>(in_flow), dimensions(), offsets(),
        std::forward<Args>(args)...);
  }
  // No more points on the left for this dim, go to next dim
  template<typename Visitor, typename Coord, typename Size, typename In,
    size_t Dim, size_t... Dims, typename... Args>
  void visit_left(Visitor& v, Coord const& idx, Size const& size, In&& in_flow,
      index_sequence<Dim, Dims...> dims, index_sequence<>, Args&&... args) const
  {
    // Go on to the next dim on the left, resetting offsets
    using NewDims = index_sequence<Dims...>;

    visit_left(v, idx, size, std::forward<In>(in_flow), NewDims(), offsets(),
        std::forward<Args>(args)...);
  }

  // No more points on the right for this dim, go to next dim
  template<typename Visitor, typename Coord, typename Size, typename In,
    size_t... Dims, typename... Args>
  void visit_right(Visitor& v, Coord const& idx, Size const& size, In&& in_flow,
      index_sequence<Dims...> dims, index_sequence<>, Args&&... args) const
  {
    using NewDims = index_sequence<Dims...>;
    // Go on to the next dim on the right, resetting offsets
    visit_right(v, idx, size, std::forward<In>(in_flow), NewDims{}, offsets(),
        std::forward<Args>(args)...);
  }

  template<typename Visitor, typename Coord, typename Size, typename In,
    size_t Dim, size_t... Dims, size_t Off0, size_t... Offs, typename... Args>
  void visit_left(Visitor& v, Coord const& idx, Size const& size, In&& in_flow,
    index_sequence<Dim, Dims...> dims,
    index_sequence<Off0, Offs...>, Args&&... args) const
  {
    // Offs ranges from [0, P), but we read from [1,P]
    constexpr auto Off = Off0 + 1;
    constexpr auto boundary_port_index = boundary_port(-1, Dim);

    const auto idxv = get<0>(idx);

    using stencil_utils::left_boundary_index;

    //past boundary as are rest of offs for this dim
    if (nd_get<Dim>(idxv) < Off)
    {
      using NewDims = index_sequence<Dims...>;
      visit_left(v, idx, std::forward<In>(in_flow), NewDims{}, offsets(),
          std::forward<Args>(args)...,
        get<boundary_port_index>(in_flow).consume_from(make_tuple(
          left_boundary_index<P, Dim, Off>(idxv, dimensions()), -1)),
        get<boundary_port_index>(in_flow).consume_from(make_tuple(
          left_boundary_index<P, Dim, Offs + 1>(idxv, dimensions()), -1))...
       );
    }
    else
    {
      point<-1, Dim, Off> p;
      using stencil_utils::nonperiodic_dep;

      visit_left(v, idx, size, std::forward<In>(in_flow), dims,
        index_sequence<Offs...>(), std::forward<Args>(args)...,
        get<center_port_index>(in_flow).consume_from(
          make_tuple(nonperiodic_dep(p, idxv, dimensions()), -1)),
          this->filter_for(p)
      );
    }

  }

  template<typename Visitor, typename Coord, typename Size, typename In,
    size_t Dim, size_t... Dims, size_t Off0, size_t... Offs, typename... Args>
  void visit_right(Visitor& v, Coord const& idx, Size const& size, In&& in_flow,
    index_sequence<Dim, Dims...> dims,
    index_sequence<Off0, Offs...>, Args&&... args) const
  {
  // Offs ranges from [0, P), but we read from [1,P]
    constexpr auto Off = Off0 + 1;
    constexpr auto boundary_port_index = boundary_port(+1, Dim);

    const auto idxv = get<0>(idx);
    const auto m = nd_get<Dim>(size);

    using stencil_utils::right_boundary_index;

    //past boundary as are rest of offs for this dim
    if (nd_get<Dim>(idxv) + Off >= m)
    {
      using NewDims = index_sequence<Dims...>;

      visit_right(v, idx, size, std::forward<In>(in_flow), NewDims(), offsets(),
          std::forward<Args>(args)...,
        get<boundary_port_index>(in_flow).consume_from(make_tuple(
          right_boundary_index<Dim, Off>(idxv, m, dimensions()), -1)),
        get<boundary_port_index>(in_flow).consume_from(make_tuple(
          right_boundary_index<Dim, Offs + 1>(idxv, m, dimensions()), -1))...
       );
    }
    else
    {
      point<+1, Dim, Off> p;
      using stencil_utils::nonperiodic_dep;

      visit_right(v, idx, size, std::forward<In>(in_flow), dims,
        index_sequence<Offs...>(), std::forward<Args>(args)...,
        get<center_port_index>(in_flow).consume_from(
          make_tuple(nonperiodic_dep(p, idxv, dimensions()), -1)),
          this->filter_for(p)
      );
    }


  }

  //back-end to consumer_count for the nonperiodic case
  template<typename Coord, typename Size, typename... Ps>
  std::size_t apply_consumer_count(Coord const& c, Size const& s, tuple<Ps...>)
  const
  {
    //the number of consumers is equal to the number of points this index
    //reads from that are not past the boundary, including itself
    auto invalids = {
      stencil_utils::past_boundary(c, nd_get<Ps::dimension>(s), Ps())...
    };
    return std::count(invalids.begin(), invalids.end(), false) + 1;
  }


};

template <typename Op, typename F, int N, int numPoints>
class stencil_pd<Op, F, true, N, numPoints>
  : public stencil_pd_base<Op, F, N, numPoints>
{

  using base_type = stencil_pd_base<Op, F, N, numPoints>;

  using typename base_type::dimensions;
  using typename base_type::offsets;

  using base_type::center_port_index;

  using points      = stencil_impl::traversal_order<dimensions, offsets>;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = numPoints;

  using op_type = Op;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructs a stencil parametric dependency with the given
  /// operation and filter functions.
  ///
  /// @param op the operation to apply at each point of the points
  ///           received
  /// @param f  the filter function to apply on the center view
  ////////////////////////////////////////////////////////////////////
  stencil_pd(Op const& op, F const& f)
    : base_type(op, f)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, j, ...> it passes the @c Op with the
  /// following inputs and to the visitor along with the @c m_op
  /// @li in<0>[0, ..., n]
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                      potentially multi-dimensional
  /// @param coord         <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor       the information about WF and input is passed
  ///                      so that later this information can be
  ///                      converted to a node in the dependence graph
  /// @param in_flow       a tuple of input flows to consume from
  ////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord&& skeleton_size, Coord&& coord, Visitor&& visitor,
    In&& in_flow) const
  {
    apply_case_of(std::forward<Coord>(skeleton_size),
                  std::forward<Coord>(coord),
                  std::forward<Visitor>(visitor),
                  std::forward<In>(in_flow),
                  points());
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given
  /// coordinate. This is a reverse query as compared to case_of.
  ///
  /// @param  skeleton_size  the size of this skeleton
  /// @param  producer_coord the coordinate of the producer element
  ///                        which is providing data to this
  ///                        parametric dependency
  /// @tparam FlowIndex      the flow index on which this request
  ///                        is sent
  ////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  constexpr std::size_t consumer_count(Size const& skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_index*/) const
  {
    return numPoints;
  }


private:

  //the periodic back_end to case_of
  template <typename Coord, typename Visitor, typename In, typename... Ps>
  void apply_case_of(Coord const& skeleton_size, Coord const& coord,
                     Visitor& visitor, In&& in_flow, tuple<Ps...>) const
  {
    auto idxv = get<0>(coord);
    auto idx  = make_tuple(idxv);
    auto dims = get<0>(skeleton_size);

    visitor(
      this->m_op,
      no_mapper(),
      get<center_port_index>(in_flow).consume_from(idx),
      get<center_port_index>(in_flow).consume_from(
        make_tuple(
          periodic_dep(
            Ps{}, idxv, nd_get<Ps::dimension>(dims), dimensions()), -1),
        this->filter_for(Ps{})
      )...
    );
  }

  //compute the index of the dependence specified by Pt and idx, wrapping over
  template<typename Pt, typename Index, size_t... Is>
  Index periodic_dep(Pt, Index const& idx,
    size_t i_max, index_sequence<Is...>) const
  {
    return Index((Is != Pt::dimension
                  ? nd_get<Is>(idx)
                  : (nd_get<Is>(idx) + i_max + Pt::signed_offset) % i_max)...
    );
  }


};


//////////////////////////////////////////////////////////////////////
/// @brief In a 1D 3 point stencil, each node depends on itself and
/// left and right neighbors.
///
/// @tparam Op         the operation to be applied at each point of
///                    the points received
/// @tparam F          the filter function to be applied on the user
///                    side
/// @ingroup skeletonsParamDepsInternal
///
//////////////////////////////////////////////////////////////////////
template <typename Op, typename F>
class stencil_pd<Op, F, true, 1, 3>
  : public stencil_pd_base<Op, F, 1, 3>
{
  using base_type = stencil_pd_base<Op, F, 1, 3>;

  static constexpr size_t center_port_index          = 0;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 3;

  using op_type = Op;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructs a stencil parametric dependency with the given
  /// operation and filter functions.
  ///
  /// @param op the operation to apply at each point of the points
  ///           received
  /// @param f  the filter function to apply on the center view
  ////////////////////////////////////////////////////////////////////
  stencil_pd(Op const& op, F const& f)
    : base_type(op, f)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief This specialization of case_of is used for the cases where
  /// the stencil computation is periodic.
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional
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
    auto i = get<0>(coord);
    auto m = get<0>(skeleton_size);

    using stencil_utils::get_stencil_filter;
    auto&& center_flow = get<        center_port_index>(in_flow);

    visitor(
      this->m_op,
      no_mapper(),
      center_flow.consume_from(make_tuple(i)),
      center_flow.consume_from(make_tuple((i + m -1) % m),
                               get_stencil_filter<tags::direction<-1>>(
                                 this->m_filter)),
      center_flow.consume_from(make_tuple((i + 1) % m),
                               get_stencil_filter<tags::direction<+1>>(
                                 this->m_filter)));
  }

  ///////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given
  /// coordinate. This is a reverse query as compared to case_of.
  ///
  /// @param  skeleton_size  the size of this skeleton
  /// @param  producer_coord the coordinate of the producer element
  ///                        which is providing data to this
  ///                        parametric dependency
  /// @tparam FlowIndex      the flow index on which this request
  ///                        is sent
  ///////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  constexpr std::size_t consumer_count(Size const& skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_index*/) const
  {
    return 3;
  }

};

template <typename Op, typename F>
class stencil_pd<Op, F, false, 1, 3>
  : public stencil_pd_base<Op, F, 1, 3>
{
  static constexpr size_t center_port_index          = 0;
  static constexpr size_t left_boundary_port_index   = 1;
  static constexpr size_t right_boundary_port_index  = 2;

  using base_type = stencil_pd_base<Op, F, 1, 3>;

public:
  static constexpr std::size_t in_port_size = 3;
  static constexpr std::size_t op_arity     = 3;

  using op_type = Op;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructs a stencil parametric dependency with the given
  /// operation and filter functions.
  ///
  /// @param op the operation to apply at each point of the points
  ///           received
  /// @param f  the filter function to apply on the center view
  ////////////////////////////////////////////////////////////////////
  stencil_pd(Op const& op, F const& f)
    : base_type(op, f)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief This specialization of case_of is used for the cases where
  /// the stencil computation is non-periodic.
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional
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
    auto i = get<0>(coord);
    auto m = get<0>(skeleton_size);

    using stencil_utils::get_stencil_filter;

    auto  left_filter = get_stencil_filter<tags::direction<-1>>(this->m_filter);
    auto right_filter = get_stencil_filter<tags::direction<+1>>(this->m_filter);

    auto&& center_flow = get<        center_port_index>(in_flow);
    auto&&   left_flow = get< left_boundary_port_index>(in_flow);
    auto&&  right_flow = get<right_boundary_port_index>(in_flow);

    if (m == 1) {
      visitor(
        this->m_op,
        no_mapper(),
        center_flow.consume_from(make_tuple(i)),
          left_flow.consume_from(make_tuple(0)),
         right_flow.consume_from(make_tuple(0)));
    }
    else {

      switch(find_position(i, 0, m-1))
      {
        case position::first:
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(i)),
              left_flow.consume_from(make_tuple(0)),
            center_flow.consume_from(make_tuple(i + 1), right_filter));
          break;
        case position::last:
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(i)),
            center_flow.consume_from(make_tuple(i - 1), left_filter),
             right_flow.consume_from(make_tuple(0)));
          break;
        case position::middle:
        default:
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(i)),
            center_flow.consume_from(make_tuple(i - 1), left_filter),
            center_flow.consume_from(make_tuple(i + 1), right_filter));
          break;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given
  /// coordinate. This is a reverse query as compared to case_of.
  ///
  /// @param  skeleton_size  the size of this skeleton
  /// @param  producer_coord the coordinate of the producer element
  ///                        which is providing data to this
  ///                        parametric dependency
  /// @tparam FlowIndex      the flow index on which this request
  ///                        is sent
  ///////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const& skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_index*/) const
  {
    auto i = tuple_ops::front(producer_coord);
    auto m = tuple_ops::front(skeleton_size);
    return m == 1 ? 1 : (i == 0 or i == (m-1) ? 2 : 3);
  }
};



/////////////////////////////////////////////////////////////////////
/// @brief In a 2D 5 point stencil, parametric dependencies are
/// defined in such a way that each node depends on every
/// node in its surroundings (top, left, down, right) including itself.
///
/// @tparam Op         the operation to be applied at each point
///                    on the points received
/// @tparam F          the filter function to be applied on the user
///                    side
/// @tparam isPeriodic determines whether dependency between points
///                    is periodic or not for boundary points
/// @ingroup skeletonsParamDepsInternal
///
/////////////////////////////////////////////////////////////////////
template <typename Op, typename F>
class stencil_pd<Op, F, true, 2, 5>
  : public stencil_pd_base<Op, F, 2, 5>
{
  static constexpr size_t center_port_index = 0;

  using base_type = stencil_pd_base<Op, F, 2, 5>;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 5;

  using op_type = Op;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructs a stencil parametric dependency with the given
  /// operation and filter functions.
  ///
  /// @param op the operation to apply at each point of the points
  ///           received
  /// @param f  the filter function to apply on the center view
  ////////////////////////////////////////////////////////////////////
  stencil_pd(Op const& op, F const& f)
    : base_type(op, f)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief This specialization of case_of is used for the cases where
  /// the stencil computation is periodic.
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord&& skeleton_size, Coord&& coord,
               Visitor&& visitor, In&& in_flow) const
  {
    using stapl::get;
    using stencil_utils::get_stencil_filter;

    auto index = get<0>(coord);
    auto dims  = get<0>(skeleton_size);

    std::size_t i = get<0>(index);
    std::size_t j = get<1>(index);

    std::size_t m = get<0>(dims);
    std::size_t n = get<1>(dims);

    auto&& up     = make_tuple((i + m -1) % m, j);
    auto&& left   = make_tuple(i, (j + n -1) % n);
    auto&& center = make_tuple(i,j);
    auto&& down   = make_tuple((i + 1) % m, j);
    auto&& right  = make_tuple(i, (j + 1) % n);

    using    up_dir = tags::direction<-1,  0>;
    using  left_dir = tags::direction< 0, -1>;
    using  down_dir = tags::direction<+1,  0>;
    using right_dir = tags::direction< 0, +1>;
    auto&& center_flow = get<center_port_index>(in_flow);
    visitor(
      this->m_op,
      no_mapper(),
      center_flow.consume_from(make_tuple(center)),
      center_flow.consume_from(make_tuple(up),
                               get_stencil_filter<   up_dir>(this->m_filter)),
      center_flow.consume_from(make_tuple(left),
                               get_stencil_filter< left_dir>(this->m_filter)),
      center_flow.consume_from(make_tuple(down),
                               get_stencil_filter< down_dir>(this->m_filter)),
      center_flow.consume_from(make_tuple(right),
                               get_stencil_filter<right_dir>(this->m_filter)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  ///        dependency will be consuming from a producer with a given
  ///        coordinate.This is a reverse query as compared to case_of.
  ///
  /// @param  skeleton_size  the size of this skeleton
  /// @param  producer_coord the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  constexpr std::size_t consumer_count(Size const& skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_index*/) const
  {
    return 5;
  }
};

template <typename Op, typename F>
class stencil_pd<Op, F, false, 2, 5>
  : public stencil_pd_base<Op, F, 2, 5>
{


  static constexpr size_t center_port_index          = 0;
  static constexpr size_t up_boundary_port_index     = 1;
  static constexpr size_t left_boundary_port_index   = 2;
  static constexpr size_t down_boundary_port_index   = 3;
  static constexpr size_t right_boundary_port_index  = 4;

  using base_type = stencil_pd_base<Op, F, 2, 5>;

public:
  static constexpr std::size_t in_port_size = 5;
  static constexpr std::size_t op_arity     = 5;

  using op_type = Op;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructs a stencil parametric dependency with the given
  /// operation and filter functions.
  ///
  /// @param op the operation to apply at each point of the points
  ///           received
  /// @param f  the filter function to apply on the center view
  ////////////////////////////////////////////////////////////////////
  stencil_pd(Op const& op, F const& f)
    : base_type(op, f)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief This specialization of case_of is used for the cases where
  /// the stencil computation is non-periodic.
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord&& skeleton_size, Coord&& coord,
               Visitor&& visitor, In&& in_flow) const
  {
    auto index = std::get<0>(coord);
    auto dims  = std::get<0>(skeleton_size);

    auto i = std::get<0>(index);
    auto j = std::get<1>(index);

    auto m = std::get<0>(dims);
    auto n = std::get<1>(dims);

    auto&& up     = make_tuple(i - 1, j);
    auto&& left   = make_tuple(i, j - 1);
    auto&& center = make_tuple(i,j);
    auto&& down   = make_tuple(i + 1, j);
    auto&& right  = make_tuple(i, j + 1);

    using stapl::get;
    using stencil_utils::get_stencil_filter;
    auto&& center_flow = get<        center_port_index>(in_flow);
    auto&&     up_flow = get<   up_boundary_port_index>(in_flow);
    auto&&   down_flow = get< down_boundary_port_index>(in_flow);
    auto&&   left_flow = get< left_boundary_port_index>(in_flow);
    auto&&  right_flow = get<right_boundary_port_index>(in_flow);
    if (m == 1 and n == 1) {

      visitor(
        this->m_op,
        no_mapper(),
        center_flow.consume_from(make_tuple(center)),
            up_flow.consume_from(make_tuple(center)),
          left_flow.consume_from(make_tuple(center)),
          down_flow.consume_from(make_tuple(center)),
         right_flow.consume_from(make_tuple(center)));
    }
    else
    {
      auto cur_pos0 = find_position(i, 0, m-1);
      auto cur_pos1 = find_position(j, 0, n-1);

      auto    up_filter =
        get_stencil_filter<tags::direction<-1,  0>>(this->m_filter);
      auto  left_filter =
        get_stencil_filter<tags::direction< 0, -1>>(this->m_filter);
      auto  down_filter =
        get_stencil_filter<tags::direction<+1,  0>>(this->m_filter);
      auto right_filter =
        get_stencil_filter<tags::direction< 0, +1>>(this->m_filter);

      switch(condition_value( cur_pos0, cur_pos1))
      {
        case condition_value(position::first, position::first) :
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
                up_flow.consume_from(make_tuple(make_tuple(0, j))),
              left_flow.consume_from(make_tuple(make_tuple(i, 0))),
            center_flow.consume_from(make_tuple(down), down_filter),
            center_flow.consume_from(make_tuple(right), right_filter));
            break;
        }
        case condition_value( position::middle, position::first) :
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
            center_flow.consume_from(make_tuple(up), up_filter),
              left_flow.consume_from(make_tuple(make_tuple(i, 0))),
            center_flow.consume_from(make_tuple(down), down_filter),
            center_flow.consume_from(make_tuple(right), right_filter));
            break;
        }
        case condition_value(position::last, position::first) :
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
            center_flow.consume_from(make_tuple(up), up_filter),
              left_flow.consume_from(make_tuple(make_tuple(i, 0))),
              down_flow.consume_from(make_tuple(make_tuple(0, j))),
            center_flow.consume_from(make_tuple(right), right_filter));
            break;
        }
        case condition_value(position::last, position::middle) :
        {
         visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
            center_flow.consume_from(make_tuple(up), up_filter),
            center_flow.consume_from(make_tuple(left), left_filter),
              down_flow.consume_from(make_tuple(make_tuple(0, j))),
            center_flow.consume_from(make_tuple(right), right_filter));
          break;
        }
        case condition_value(position::last, position::last) :
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
            center_flow.consume_from(make_tuple(up), up_filter),
            center_flow.consume_from(make_tuple(left), left_filter),
              down_flow.consume_from(make_tuple(make_tuple(0, j))),
             right_flow.consume_from(make_tuple(make_tuple(i, 0))));
          break;
        }
        case condition_value(position::middle, position::last) :
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
            center_flow.consume_from(make_tuple(up), up_filter),
            center_flow.consume_from(make_tuple(left), left_filter),
            center_flow.consume_from(make_tuple(down), down_filter),
             right_flow.consume_from(make_tuple(make_tuple(i, 0))));
          break;
        }
        case condition_value( position::first, position::last):
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
                up_flow.consume_from(make_tuple(make_tuple(0, j))),
            center_flow.consume_from(make_tuple(left), left_filter),
            center_flow.consume_from(make_tuple(down), down_filter),
             right_flow.consume_from(make_tuple(make_tuple(i, 0))));
          break;
        }
        case condition_value( position::first, position::middle) :
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
                up_flow.consume_from(make_tuple(make_tuple(0, j))),
            center_flow.consume_from(make_tuple(left), left_filter),
            center_flow.consume_from(make_tuple(down), down_filter),
            center_flow.consume_from(make_tuple(right),right_filter));
          break;
        }
        case condition_value( position::middle, position::middle) :
        default:
        {
          visitor(
            this->m_op,
            no_mapper(),
            center_flow.consume_from(make_tuple(center)),
            center_flow.consume_from(make_tuple(up), up_filter),
            center_flow.consume_from(make_tuple(left), left_filter),
            center_flow.consume_from(make_tuple(down), down_filter),
            center_flow.consume_from(make_tuple(right), right_filter));
          break;
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  ///        dependency will be consuming from a producer with a given
  ///        coordinate.This is a reverse query as compared to case_of.
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
    auto index = tuple_ops::front(producer_coord);
    auto size  = tuple_ops::front(skeleton_size);

    auto i = std::get<0>(index);
    auto j = std::get<1>(index);

    auto m = std::get<0>(size);
    auto n = std::get<1>(size);

    if (m == 1 and n == 1) {
      return 1;
    }
    else if ((i == 0 or i == m-1) and (j == 0 or j == n-1)) {
      return 3;
    }
    else if (i == 0 or i == m-1 or j == 0 or j == n-1) {
      return 4;
    }
    else { // middle point
      return 5;
    }
  }
};


} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates an nD stencil parametric dependency
///
/// @tparam Op         the operation to be applied at each point
///                    on the points received
/// @tparam F          the filter function to be applied on the user
///                    side
/// @tparam isPeriodic determines whether dependency between points
///                    is periodic or not for boundary points
/// @tparam dimensions specifies the number of input
///                    dimension(1D, 2D, ... , nD)
/// @ingroup skeletonsParamDepsInternal
///
//////////////////////////////////////////////////////////////////////
template <bool isPeriodic = true,
          int dimensions = 1,
          int numPoints = 3,
          typename Op,
          typename F    = skeletons::no_filter>
skeletons_impl::stencil_pd<Op, F, isPeriodic, dimensions, numPoints>
stencil_pd(Op const& op, F const& f = F())
{
  return {op, f};
}

} // namespace skeletons
} // namespace stapl

#endif //STAPL_SKELETONS_PARAM_DEPS_STENCIL_PD_HPP
