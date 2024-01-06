/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_VERTEX_OPERATOR_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_VERTEX_OPERATOR_HPP

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/views/localize_element.hpp>
#include <stapl/containers/graph/algorithms/paradigms/visitor.hpp>
#include <stapl/containers/graph/algorithms/paradigms/neighbor_operator.hpp>
#include <stapl/utility/for_each_range.hpp>
#include <stapl/utility/conditional_lock_guard.hpp>

namespace stapl {

namespace kla_detail {

struct false_predicate
{
  template<typename Vertex>
  bool operator()(Vertex&&) const
  { return false;}
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
///
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
///
/// @tparam ReinvokeBehavior Enum to specify under which conditions to reinvoke
/// the vertex operator from the neighor operator
/// vertex is visited with the neighbor operator, the vertex operator will
/// not be invoked on it if its degree exceeds a user-defined threshold.
/// @tparam Graph Type of the input graph view.
/// @tparam VertexOp Type of the user provided work function.
/// @tparam Derived vertex_operator_apply class, which uses CRTP
/// to provide certain interfaces (such as hub avoidance)
/// @tparam Predicate Type of a predicate work function that recieves a vertex
/// and returns a bool indicating whether or not to terminate. This
/// defaults to a false_predicate which always returns false.
/// @ingroup pgraphAlgoDetails
/// @todo This work function stores a view to the input graph that is
/// needed for calling apply_set on target vertices inorder to apply
/// the neighbor-operator. This is currently passed in through the operator() as
/// a @ref repeat_view, with the pointer being set to the view received in
/// the operator(). However, this is slower than passing the view through
/// the constructor by ~7% on Hopper (using the g500 benchmark test). This
/// is documented in GForge to-do task #1207.
//////////////////////////////////////////////////////////////////////
template <reinvoke_behavior ReinvokeBehavior,
          class Graph,
          class VertexOp,
          class Derived,
          class Predicate = kla_detail::false_predicate>
class vertex_operator_apply_base
{
protected:
  Graph* m_graph;
  VertexOp m_wf;
  size_t   m_curr_level;
  size_t   m_max_level;
  Predicate m_pred;

public:
  using graph_type = Graph;
  using derived_type = Derived;
  using vertex_operator_type = VertexOp;


  //////////////////////////////////////////////////////////////////////
  /// @brief Helper class used to create a wrapped neighbor operator, which
  ///        may or may not include a copy of the vertex operator, depending
  ///        on the reinvoke behavior.
  //////////////////////////////////////////////////////////////////////
  template<typename NeighborOp>
  struct make_wrapped_neighbor_operator
  {
    /// Type of the wrapped neighbor operator
    using type = neighbor_operator_apply<ReinvokeBehavior, Derived,
      typename std::decay<NeighborOp>::type>;

    template<typename Parent, typename Nop>
    static type create(Parent&& parent, Nop&& nop)
    {
      return { *static_cast<derived_type const*>(&parent),
               std::forward<NeighborOp>(nop) };
    }
  };

  using result_type = std::pair<bool, bool>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Vertex Operator Apply Base
  /// @param wf vertex operator
  /// @param curr_level The current KLA-level of this visit.
  /// @param max_level The maximum KLA-level of this visit.
  /// @param pred a predicate which recieves a vertex and is
  /// used to decide whether to terminate early
  //////////////////////////////////////////////////////////////////////
  vertex_operator_apply_base(Graph* g,
                             VertexOp wf,
                             size_t curr_level,
                             size_t max_level,
                             Predicate pred = Predicate())
    : m_graph(g)
    , m_wf(std::move(wf))
    , m_curr_level(curr_level)
    , m_max_level(max_level)
    , m_pred(std::move(pred))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator that invokes the vertex operator.
  ///
  /// @return A pair of bools, the first representing this vertex's vote
  ///         to continue and the second representing this vertex's vote
  ///         to halt.
  //////////////////////////////////////////////////////////////////////
  template<class Vertex>
  result_type operator()(Vertex&& v) const
  {
    using mutex_type = decltype(m_graph->distribution());
    using visitor_type = kla_visitor<vertex_operator_apply_base>;

    sgl::visit_logger::pre_vertex_op(v, this->level(), this->max_level());

    // Perform an RMI lock if this vertex operator follows a strong
    // concurrency model
    constexpr bool should_lock =
      sgl::has_strong_concurrency_model<VertexOp>::type::value;
    conditional_lock_guard<mutex_type, should_lock> lg{m_graph->distribution()};

    // Execute vertex operator and record whether or not it voted to continue
    const bool should_continue = m_wf(
      std::forward<Vertex>(v), visitor_type{*this});

    sgl::visit_logger::post_vertex_op(
      v, should_continue, this->level(), this->max_level());

    // Execute the halt predicate and record whether or not it voted to halt
    const bool should_halt = m_pred(v);

    return {should_continue, should_halt};
  }

  void increment_level()
  { ++m_curr_level; }

  size_t const& level() const
  { return m_curr_level; }

  size_t const& max_level() const
  { return m_max_level; }

  void increment_iteration(size_t k)
  {
    m_curr_level += (k + 1);
    m_max_level = m_curr_level + k;
  }

  graph_type* graph() const
  {
    return m_graph;
  }

  void define_type(typer& t)
  {
    t.member(m_graph);
    t.member(m_wf);
    t.member(m_curr_level);
    t.member(m_max_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
///
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
///
/// This is a facade for @see vertex_operator_apply_base that does not provide
/// the hub avoidance optimization.
///
///
/// @tparam ReinvokeBehavior Flag used to determine under which conditions
/// a vertex operator would be reinvoked from the neighbor operator
/// @tparam Graph Type of the input graph view.
/// @tparam VertexOp Type of the user provided work function.
/// @tparam Predicate Predicate function which recieves a vertex and is used to
/// check whether to terminate.
//////////////////////////////////////////////////////////////////////
template <reinvoke_behavior ReinvokeBehavior,
          class Graph,
          class VertexOp,
          class Predicate = kla_detail::false_predicate>
class vertex_operator_apply
  : public vertex_operator_apply_base<ReinvokeBehavior,
                                      Graph,
                                      VertexOp,
                                      vertex_operator_apply<ReinvokeBehavior,
                                                            Graph,
                                                            VertexOp,
                                                            Predicate>,
                                      Predicate>
{
  typedef vertex_operator_apply_base<ReinvokeBehavior, Graph, VertexOp,
    vertex_operator_apply<ReinvokeBehavior, Graph, VertexOp, Predicate>,
    Predicate
  > base_type;

public:
  template<typename... Args>
  vertex_operator_apply(Args&&... args)
   : base_type(std::forward<Args>(args)...)
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
///
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
///
/// This is a specialization for @see vertex_operator_apply to provide
/// the hub avoidance optimization.
///
/// @tparam Graph Type of the input graph view.
/// @tparam VertexOp Type of the user provided work function.
/// @tparam Predicate Predicate function which recieves a vertex and is used to
/// check whether to terminate.
//////////////////////////////////////////////////////////////////////
template <class Graph, class VertexOp, class Predicate>
class vertex_operator_apply<reinvoke_behavior::avoid_hubs,
                            Graph,
                            VertexOp,
                            Predicate>
  : public vertex_operator_apply_base<reinvoke_behavior::avoid_hubs,
                                      Graph,
                                      VertexOp,
                                      vertex_operator_apply<reinvoke_behavior::
                                                              avoid_hubs,
                                                            Graph,
                                                            VertexOp,
                                                            Predicate>,
                                      Predicate>
{
  typedef vertex_operator_apply_base<
    reinvoke_behavior::avoid_hubs, Graph, VertexOp,
      vertex_operator_apply<
        reinvoke_behavior::avoid_hubs, Graph, VertexOp, Predicate
      >, Predicate
  > base_type;

  std::size_t m_degree_threshold;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the work function.
  /// @param wf User's vertex operator
  /// @param curr_level The current level of the algorithm
  /// @param max_level The maximum level that this KLA SS will reach
  /// @param degree_threshold The out-degree of a vertex to consider it a hub
  //////////////////////////////////////////////////////////////////////
  template<typename Op>
  vertex_operator_apply(Graph* g, Op&& wf, size_t curr_level, size_t max_level,
                        std::size_t degree_threshold,
                        Predicate pred=Predicate())
    : base_type(g, std::forward<Op>(wf), curr_level, max_level, pred),
      m_degree_threshold(degree_threshold)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the degree threshold to consider a vertex a hub for
  ///        hub optimization.
  //////////////////////////////////////////////////////////////////////
  size_t const& degree_threshold() const
  { return m_degree_threshold; }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_degree_threshold);
  }
};

} // namespace kla_detail

} // namespace stapl

#endif
