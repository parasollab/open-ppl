/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_P_GRAPH_HIERARCHICAL_VIEW_HPP
#define STAPL_CONTAINERS_GRAPH_P_GRAPH_HIERARCHICAL_VIEW_HPP

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/utility/use_default.hpp>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Hierarchical view for the pGraph.
/// Reflects all operations of a graph, and provides information about
/// the level in the hierarchy.
/// @tparam PG The container for the view.
/// @tparam Dom The domain for the view.
/// @tparam MapFunc The mapping function to be used in the view.
/// Default is @ref f_ident.
/// @tparam Derived Most derived class for use with CRTP.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename PG,
          typename Dom     = typename PG::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>,
          typename Derived = use_default>
class hierarchical_graph_view
  : public graph_view<PG, Dom, MapFunc, Derived>
{
  typedef graph_view<PG, Dom, MapFunc, Derived>          base_type;

  typedef typename select_derived<
    Derived,
    hierarchical_graph_view<PG, Dom, MapFunc, Derived>
  >::type                                                derived_type;

  size_t m_level;

 public:
  STAPL_VIEW_REFLECT_TRAITS(hierarchical_graph_view)

  hierarchical_graph_view(size_t level = 0)
    : m_level(level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param level The current level in the hierarchy.
  /// @param vcont Pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph_view(size_t level, view_container_type* vcont,
                          domain_type const& dom, map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc),
      m_level(level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param level The current level in the hierarchy.
  /// @param vcont Reference to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph_view(size_t level, view_container_type const& vcont,
                          domain_type const& dom, map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc),
      m_level(level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view takes ownership of the container.
  ///
  /// @param level The current level in the hierarchy.
  /// @param vcont Pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph_view(size_t level, view_container_type* vcont)
    : base_type(vcont, vcont->domain()),
      m_level(level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container.
  ///
  /// @param level The current level in the hierarchy.
  /// @param vcont Pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph_view(size_t level, view_container_type& vcont)
    : base_type(vcont, vcont.domain()),
      m_level(level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container.
  ///
  /// @param level The current level in the hierarchy.
  /// @param vcont Pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph_view(size_t level, view_container_type const& vcont)
    : base_type(vcont, vcont.domain()),
      m_level(level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the passed view is not the most
  ///        derived view.
  //////////////////////////////////////////////////////////////////////
  template<typename Derived1>
  hierarchical_graph_view(hierarchical_graph_view<PG, Dom, MapFunc, Derived1>
                          const& other)
    : base_type(other.get_container(), other.domain(), other.mapfunc()),
      m_level(other.level())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container. This constructor assumes that it is the first level
  ///        of the hierarchy.
  ///
  /// @param vcont Reference to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph_view(view_container_type const& vcont,
                          domain_type const& dom, map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc),
      m_level(0)
  { }

  hierarchical_graph_view(view_container_type const& vcont,
                          domain_type const& dom, map_func_type mfunc,
                          hierarchical_graph_view const& other)
    : base_type(vcont, dom, mfunc),
      m_level(other.m_level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the level of the current graph in the hierarchy.
  //////////////////////////////////////////////////////////////////////
  size_t level() const
  {
    return m_level;
  }
}; // class hierarchical_graph_view


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for the @ref hierarchical_graph_view
/// @tparam Accessor The accessor for the proxy.
/// @tparam PG The container for the view.
/// @tparam Dom The domain for the view.
/// @tparam MapFunc The mapping function to be used in the view.
/// Default is @ref f_ident.
/// @tparam Derived Most derived class for use with CRTP.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename Accessor,
          typename PG,
          typename Dom,
          typename MapFunc,
          typename Derived>
class proxy<hierarchical_graph_view<PG, Dom, MapFunc, Derived>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef hierarchical_graph_view<PG, Dom, MapFunc, Derived> target_t;
  typedef typename target_t::domain_type domain_type;
  typedef typename target_t::vertex_iterator vertex_iterator;
  typedef typename target_t::vertex_property vertex_property;
  typedef typename target_t::vertex_descriptor vertex_descriptor;
  typedef typename target_t::edge_descriptor edge_descriptor;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  domain_type get_domain(void)
  { return Accessor::invoke(&target_t::get_domain); }

  void set_domain(domain_type const& domain)
  { Accessor::invoke(&target_t::set_domain, domain); }

  vertex_iterator begin()
  { return Accessor::invoke(&target_t::begin); }

  vertex_iterator end()
  { return Accessor::invoke(&target_t::end); }

  size_t num_vertices() const
  { return Accessor::const_invoke(&target_t::num_vertices); }

  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    using fn_t = vertex_descriptor (target_t::*)(vertex_property const&);
    constexpr fn_t pmf = &target_t::add_vertex;
    vertex_descriptor vid = Accessor::invoke(pmf, vp);
    return vid;
  }

  void add_edge_async(vertex_descriptor const& src,
                      vertex_descriptor const& tgt)
  { return this->ref().add_edge_async(src, tgt); }

  edge_descriptor add_edge(vertex_descriptor const& src,
                           vertex_descriptor const& tgt)
  {
    return this->ref().add_edge(src, tgt);
  }

  future<bool> has_edge(vertex_descriptor const& source,
                        vertex_descriptor const& target)
  {
    return this->ref().has_edge(source, target);
  }

  vertex_iterator find_vertex(vertex_descriptor const& vd) const
  {
    return this->ref().find_vertex(vd);
  }

}; //struct proxy



//////////////////////////////////////////////////////////////////////
/// @brief Work-function to add a vertex containing the domain of the
/// child-vertices, with the given descriptor to the output graph.
/// @tparam G0ViewType The view over the child-level graph.
/// @tparam PropertyType The type of the vertex property of the output graph.
/// @param dom The domain over the child-vertices for the vertex being added.
/// @param vd The descriptor of the vertex being added.
/// @param gv The view over the output graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G0ViewType, typename PropertyType>
struct add_vertex_wf
{
  G0ViewType const*     m_gvw0;
  size_t                m_level;

  typedef void result_type;

  add_vertex_wf(G0ViewType const& gvw0, size_t level)
    : m_gvw0(&gvw0), m_level(level)
  { }

  template<typename Domain, typename Descriptor, typename GraphView>
  void operator() (Domain dom, Descriptor vd, GraphView& gv) const
  {
    gv.add_vertex(
      vd,
      PropertyType(m_level, m_gvw0->container(), dom, m_gvw0->mapfunc())
    );
  }

  void define_type(typer& t)
  {
    abort("add_vertex_wf unexpectedly serialized");

    t.member(m_gvw0);
    t.member(m_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to extract the domain-type from a view.
/// @tparam bool Indicates whether to use the VertexPartitioner's domain_type,
/// if true, or the @ref domset1D, if false.
/// @tparam GraphVw The type of the view over the graph.
/// @tparam VertexPartitioner The type of the vertex partitioner.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<bool, typename GraphVw, typename VertexPartitioner>
struct extracted_domain_type
{
  typedef typename VertexPartitioner::domain_type type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for helper to extract the domain-type from a view,
/// when bool is false. Uses @ref domset1D as domain.
/// @tparam GraphVw The type of the view over the graph.
/// @tparam VertexPartitioner The type of the vertex partitioner.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename GraphVw, typename VertexPartitioner>
struct extracted_domain_type<false, GraphVw, VertexPartitioner>
{
  typedef domset1D<typename GraphVw::vertex_descriptor> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to extract the domain-type from a view.
/// @tparam GraphVw The type of the view over the graph.
/// @tparam VertexPartitioner The type of the vertex partitioner.
///
/// Uses the VertexPartitioner's domain_type, if it has one, or
/// @ref domset1D otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename GraphVw, typename VertexPartitioner>
struct extract_domain_type
{
  typedef typename extracted_domain_type<
                     has_domain_type<VertexPartitioner>::value,
                     GraphVw, VertexPartitioner>::type type;
};



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create a level in the hierarchy of graphs,
/// using the user-provided vertex partitioner and edge-functor.
/// @param graph A @ref graph_view over the input graph.
/// @param partitioner The vertex paritioner which takes an input graph
/// and returns the partition of vertices on next level, along with
/// descriptors corresponding to them.
/// @param ef The edge functor which takes graph of the next level,
/// and adds the required edges.
/// @param level The level of this view in the hierarchy.
/// @return A @ref hierarchical_graph_view over the graph of the next-level.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphVw, typename VertexPartitioner, typename EdgeFunctor>
hierarchical_graph_view<
  dynamic_graph<DIRECTED, MULTIEDGES,
    hierarchical_graph_view<typename GraphVw::view_container_type,
      typename extract_domain_type<GraphVw, VertexPartitioner>::type >,
    typename EdgeFunctor::value_type> >
create_level (const GraphVw& graph,
              const VertexPartitioner& partitioner, const EdgeFunctor& ef,
              size_t level)
{
  typedef typename GraphVw::view_container_type                 graph_cont_t;
  typedef hierarchical_graph_view<graph_cont_t,
    typename extract_domain_type<GraphVw, VertexPartitioner>::type> property_t;
  typedef typename EdgeFunctor::value_type                      edge_property_t;
  typedef dynamic_graph<DIRECTED, MULTIEDGES,
                        property_t, edge_property_t>            level_graph_t;
  typedef hierarchical_graph_view<level_graph_t>                level_view_t;
  typedef typename VertexPartitioner::view_type                 PartitionedView;
  typedef typename VertexPartitioner::descriptor_view_type      DescriptorView;

  // create a graph for that level
  level_graph_t *level_graph = new level_graph_t();
  // put it in a view that'll manage it
  level_view_t level_view_temp(level, *level_graph);

  // call user's vertex-partitioner on graph of last-level:
  std::pair<PartitionedView, DescriptorView> pv = partitioner(graph, level);

  // populate it with vertices
  map_func(add_vertex_wf<GraphVw, property_t>(graph, level-1),
           pv.first,
           pv.second,
           make_repeat_view(level_view_temp));

  // add edges
  ef(level_graph, level);

  rmi_fence();  // Needed because of comm. generated during edge-addition.

  return level_view_t(level, level_graph);
}


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create a level in the hierarchy of graphs,
/// using the user-provided vertex partitioner and edge-functor.
/// @param graph A @ref hierarchical_graph_view over the input graph.
/// @param partitioner The vertex paritioner which takes an input graph
/// and returns the partition of vertices on next level, along with
/// descriptors corresponding to them.
/// @param ef The edge functor which takes graph of the next level,
/// and adds the required edges.
/// @return A @ref hierarchical_graph_view over the graph of the next-level.
/// @ingroup pgraphAlgo
/// @todo Directed and Multiedges should be taken from the base graph
//////////////////////////////////////////////////////////////////////
template<typename VertexPartitioner, typename EdgeFunctor,
         typename PG, typename Dom, typename MapFunc, typename Derived>
hierarchical_graph_view<
  dynamic_graph<DIRECTED, MULTIEDGES,
    hierarchical_graph_view<PG,
      typename extract_domain_type<PG, VertexPartitioner>::type>,
    typename EdgeFunctor::value_type>
>
create_level(const hierarchical_graph_view<PG, Dom, MapFunc, Derived>& graph,
             const VertexPartitioner& partitioner, const EdgeFunctor& ef)
{
  return create_level(graph, partitioner, ef, graph.level()+1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create a level in the hierarchy of graphs,
/// using the user-provided vertex partitioner and edge-functor.
/// @param graph A @ref graph_view over the input graph.
/// @param partitioner The vertex paritioner which takes an input graph
/// and returns the partition of vertices on next level, along with
/// descriptors corresponding to them.
/// @param ef The edge functor which takes graph of the next level,
/// and adds the required edges.
/// @return A @ref hierarchical_graph_view over the graph of the next-level.
/// @ingroup pgraphAlgo
/// @todo Directed and Multiedges should be taken from the base graph
//////////////////////////////////////////////////////////////////////
template<typename VertexPartitioner, typename EdgeFunctor,
         typename PG, typename Dom, typename MapFunc, typename Derived>
hierarchical_graph_view<
  dynamic_graph<DIRECTED, MULTIEDGES,
    hierarchical_graph_view<PG,
      typename extract_domain_type<PG, VertexPartitioner>::type>,
    typename EdgeFunctor::value_type>
>
create_level (const graph_view<PG, Dom, MapFunc, Derived>& graph,
              const VertexPartitioner& partitioner, const EdgeFunctor& ef)
{
  return create_level(graph, partitioner, ef, 1);
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct to figure-out the hierarchical view type.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename GraphVw, typename VertexPartitioner, typename EdgeProperty>
struct hierarchical_view_type
{
  typedef hierarchical_graph_view<
            dynamic_graph<DIRECTED, MULTIEDGES,
              hierarchical_graph_view<typename GraphVw::view_container_type,
                typename extract_domain_type<GraphVw, VertexPartitioner>::type>,
              EdgeProperty>
          > type;
};

} // namespace stapl
#endif
