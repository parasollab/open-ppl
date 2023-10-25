/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_HPP
#define STAPL_CONTAINERS_GRAPH_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/graph/traits/static_graph_traits.hpp>
#include <stapl/containers/distribution/composed_specification.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/containers/type_traits/container_wrapper_ref.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/utility/use_default.hpp>

#include <stapl/containers/distribution/is_distribution_view.hpp>
#include <boost/utility/enable_if.hpp>

#include "graph_fwd.hpp"
#include "proxy.hpp"

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object called by a container whose value type is another
///  container to properly populate the former's base container.
/// @param dist The distribution of the outer container.
/// @param factory Functor that returns pointer to heap allocated instances
///   of the inner container.
/// @todo Possibly use rmi_synchronize or rmi_fence (likely not fence) on the
///   function object (was commented out in previous versions of this code
///   segment.
//////////////////////////////////////////////////////////////////////
class graph_nested_initializer
{
private:
  /// @brief True if container is to be created in an spmd fashion by all
  /// locations (even though only one will store it in the base container).
  bool m_b_spmd;

public:
  void define_type(typer& t)
  {
    t.member(m_b_spmd);
  }

  graph_nested_initializer(bool b_spmd)
    : m_b_spmd(b_spmd)
  { }

  template<typename Distribution, typename Factory>
  void operator()(Distribution& dist, Factory const& factory) const
  {
    typedef typename Distribution::base_container_type base_container_type;

    typedef typename boost::remove_pointer<
      decltype(std::declval<Factory>()(size_t(), bool()))
    >::type                                            value_type;

    const size_t n = dist.size();

    for (size_t i=0; i<n; ++i)
    {
      const bool b_contains = dist.container_manager().contains(i);

      value_type* elem_ptr = 0;

      if (m_b_spmd || b_contains)
        elem_ptr = factory(i, b_contains);

      if (b_contains)
        dist.container_manager().invoke(
          i, &base_container_type::vp_set,
          i, container_wrapper_ref<value_type>(*elem_ptr)
        );
    }
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Class for selecting between default and optional template parameters.
/// @ingroup pgraphImpl
/// @tparam D graph-attribute specifying Directedness. (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP (Optional) type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP (Optional) type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam PS (Optional) Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam M (Optional) Mapper that defines how to map the subdomains produced
/// by the partition to locations.
/// @tparam Traits (Optional) A traits class that defines customizable
/// components of graph, such as the domain type, base container type, storage,
/// etc.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M, typename ...OptionalParams>
struct graph_param_selector
{
  typedef tuple<
    properties::no_property,
    properties::no_property,
    balanced_partition<indexed_domain<size_t> >, // Partition
    mapper<size_t>,                              // Mapper
    static_graph_traits<D, M, properties::no_property, properties::no_property,
                        balanced_partition<indexed_domain<size_t> >,
                        mapper<size_t> >
    >                                                   default_value_types;

  typedef typename compute_type_parameters<
    default_value_types, OptionalParams...
  >::type                                               param_types;

  typedef typename tuple_element<0, param_types>::type  vertex_prop;
  typedef typename tuple_element<1, param_types>::type  edge_prop;
  typedef typename tuple_element<2, param_types>::type  partition_t;
  typedef typename tuple_element<3, param_types>::type  mapper_t;

  typedef tuple<
    vertex_prop, edge_prop, partition_t, mapper_t,
    static_graph_traits<D, M, vertex_prop, edge_prop, partition_t, mapper_t>
    >                                                   default_traits_type;

  typedef typename compute_type_parameters<
    default_traits_type, OptionalParams...
  >::type                                               traits_param_type;

  typedef typename tuple_element<4, traits_param_type>::type  traits_t;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base-class for graph that implements all functionality needed
/// for a directed static graph. Deriving classes may over-write
/// functionality for adding/deleting edges (for Undirectedness).
/// @ingroup pgraphBase
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgePx type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PSx Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam Mapx Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref static_graph_traits.
/// @tparam Derived Most derived class for use with CRTP.
//////////////////////////////////////////////////////////////////////
template<typename Derived, graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
class directed_graph_base
  : public container<Derived>
{
protected:
  using VertexP =
   typename graph_param_selector<D, M, OptionalParams...>::vertex_prop;

  using EdgeP =
    typename graph_param_selector<D, M, OptionalParams...>::edge_prop;

  using PS =
    typename graph_param_selector<D, M, OptionalParams...>::partition_t;

  using Map =
    typename graph_param_selector<D, M, OptionalParams...>::mapper_t;

  using Traits =
    typename graph_param_selector<D, M, OptionalParams...>::traits_t;

  typedef Derived                                      derived_type;
  typedef container<derived_type>                      base_type;

public:
  typedef typename base_type::value_type               value_type;
  typedef typename base_type::partition_type           partition_type;
  typedef typename base_type::mapper_type              mapper_type;
  typedef typename partition_type::value_type          domain_type;
  typedef typename mapper_type::domain_type            map_dom_t;
  typedef typename domain_type::index_type             index_type;
  typedef typename domain_type::index_type             gid_type;
  typedef typename domain_type::size_type              size_type;

  typedef typename base_type::distribution_type        distribution_type;
  typedef typename distribution_type::reference        reference;
  typedef typename distribution_type::const_reference        const_reference;
  typedef typename distribution_type::accessor_type    accessor_type;
  typedef typename distribution_type::iterator         iterator;

  /// Distribution metadata type used for coarsening
  typedef typename distribution_type::loc_dist_metadata loc_dist_metadata;

  typedef typename Traits::vertex_property             vertex_property;
  typedef typename Traits::edge_property               edge_property;

  /// @brief Type of the descriptor of the vertex. Must be convertible to
  /// @ref simple_vertex_descriptor.
  typedef typename Traits::vertex_descriptor           vertex_descriptor;

  /// An integral type that is convertible from/to @ref vertex_descriptor.
  typedef typename Traits::simple_vertex_descriptor    simple_vertex_descriptor;
  typedef typename Traits::edge_descriptor             edge_descriptor;
  typedef typename Traits::directness_type             directness_type;
  typedef typename Traits::multiplicity_type           multiplicity_type;

  /// Iterator over the vertices of the graph. Same as @ref iterator
  /// for compatibility.
  typedef typename distribution_type::iterator         vertex_iterator;
  typedef typename distribution_type::const_iterator   const_vertex_iterator;

  /// Iterator over the adjacent edges of a vertex.
  typedef typename distribution_type::adj_edge_iterator adj_edge_iterator;
  typedef typename distribution_type::const_adj_edge_iterator
                                                  const_adj_edge_iterator;

  /// Iterator over the edges of a graph.
  typedef typename distribution_type::edge_iterator    edge_iterator;

  /// Reference to a vertex of the graph. Same as @ref reference
  /// for compatibility.
  typedef typename distribution_type::reference        vertex_reference;

  /// @name Nested Container Construction
  /// @{

  /// @}

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an empty graph.
  //////////////////////////////////////////////////////////////////////
  directed_graph_base(void)
    : base_type(partition_type(domain_type(),
                               get_num_locations()),
                mapper_type(partition_type(domain_type(),
                              get_num_locations()).domain())
                )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size.
  //////////////////////////////////////////////////////////////////////
  directed_graph_base(size_t const& n)
    : base_type(partition_type(domain_type(0, n-1, true),
                               get_num_locations()),
                mapper_type(partition_type(domain_type(0, n-1, true),
                              get_num_locations()).domain())
                )
  { stapl_assert(n != 0, "Size should be non-zero"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and
  /// constructs all elements with a default value for vertex property.
  //////////////////////////////////////////////////////////////////////
  directed_graph_base(size_t const& n, vertex_property const& default_value)
    : base_type(partition_type(domain_type(0, n-1, true),
                               get_num_locations()),
                mapper_type(partition_type(domain_type(0, n-1, true),
                              get_num_locations()).domain()),
                value_type(index_bounds<vertex_descriptor>::invalid(),
                             default_value
                           ))
  { stapl_assert(n != 0, "Size should be non-zero"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and mapper
  /// constructs all elements with a default value for vertex property.
  /// @param n Size of the graph.
  /// @param m Mapper for the data distribution.
  //////////////////////////////////////////////////////////////////////
  directed_graph_base(size_t const& n, mapper_type const& m,
                   vertex_property const& default_value=vertex_property())
    : base_type(partition_type(domain_type(0, n-1, true),
                               get_num_locations()),
                m,
                value_type(index_bounds<vertex_descriptor>::invalid(),
                             default_value
                           ))
  { stapl_assert(n != 0, "Size should be non-zero"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and partition, constructing
  /// all vertices with the value provided for vertex property.
  //////////////////////////////////////////////////////////////////////
  directed_graph_base(partition_type const& ps,
                      vertex_property const& default_value=vertex_property())
    : base_type(ps, mapper_type(indexed_domain<size_t>(0,ps.size()-1)),
                value_type(index_bounds<vertex_descriptor>::invalid(),
                             default_value
                           ))
  { stapl_assert(ps.size() != 0, "Size should be non-zero"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size, partition and mapper.
  //////////////////////////////////////////////////////////////////////
  directed_graph_base(partition_type const& ps, mapper_type const& m)
    : base_type(ps, m)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size, partition and mapper,
  /// constructing all vertices with the value provided for vertex property.
  //////////////////////////////////////////////////////////////////////
  directed_graph_base(partition_type const& ps, mapper_type const& m,
                      vertex_property const& default_value)
    : base_type(ps, m,
                value_type(index_bounds<vertex_descriptor>::invalid(),
                             default_value
                           ))
  { }

  template <typename DistSpecsView>
  directed_graph_base(DistSpecsView const& dist_view,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(std::shared_ptr<DistSpecsView>(new DistSpecsView(dist_view)))
  { }

  template <typename DistSpecsView>
  directed_graph_base(DistSpecsView const& dist_view,
    vertex_property const& default_value,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(std::shared_ptr<DistSpecsView>(new DistSpecsView(dist_view)),
        value_type(index_bounds<vertex_descriptor>::invalid(), default_value))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and default
  /// value where the vertex_property may itself be a parallel container.
  /// Required for pC composition.
  /// @param n The number of vertices
  /// @param default_value The initial value of the vertices' properties.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename DP, typename VP>
  directed_graph_base(size_t const& n, VP const& default_value, DP dis_policy,
                      typename boost::enable_if<is_p_object<VP> >::type* = 0)
    : base_type(partition_type(domain_type(0, n-1, true),
                               get_num_locations()),
                mapper_type(partition_type(domain_type(0, n-1, true),
                              get_num_locations()).domain()),
                value_type(index_bounds<vertex_descriptor>::invalid(),
                             default_value),
                dis_policy)
  { stapl_assert(n != 0, "Size should be non-zero"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  directed_graph_base(boost::tuples::cons<X,Y> dims)
    : base_type(
        partition_type(domain_type(dims.get_head()),get_num_locations()),
        mapper_type(map_dom_t(get_num_locations())), dims
      )
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  directed_graph_base(boost::tuples::cons<X,Y> dims, DP const& dis_policy)
    : base_type(
        partition_type(domain_type(dims.get_head()), get_num_locations()),
        mapper_type(map_dom_t(get_num_locations())), dims, dis_policy)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  directed_graph_base(SizesView const& sizes_view,
                      typename std::enable_if<
                        boost::is_same<size_type,
                                       typename SizesView::size_type>::value &&
                        !is_distribution_view<SizesView>::value &&
                        !detail::has_is_composed_dist_spec<SizesView>::value
                      >::type* = 0)
    : base_type(
        partition_type(domain_type(sizes_view.size()), get_num_locations()),
        mapper_type(map_dom_t(get_num_locations())),
        sizes_view
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers. For an m level composed
  /// container, @p comp_spec contains specifications of the distributions
  /// of the current container and each of its elements.
  ///
  /// The specification of the current container's distribution is accessed
  /// by calling the @p spec() method, while the distribution specification
  /// of an element is accessed via @p operator[].
  ///
  /// @param comp_spec Instance of @ref composed_dist_spec representing the
  /// distribution specifications for each nested container.
  //////////////////////////////////////////////////////////////////////
  template <typename ComposedSpec>
  directed_graph_base(ComposedSpec const& comp_spec,
                      typename std::enable_if<
                        detail::has_is_composed_dist_spec<
                          ComposedSpec>::value>::type* = 0)
    : base_type(
        std::shared_ptr<typename ComposedSpec::distribution_spec>(
          new typename ComposedSpec::distribution_spec(comp_spec.spec())),
        comp_spec
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  /// container, @p dist_specs contains specifications of the distributions
  /// to be used at each level of the composed container.
  ///
  /// The first element of the vector specifies the distribution of the outer
  /// container, the second the distribution of the containers at the first
  /// level of composition, etc.  The number of elements in @p dist_specs must
  /// be at least the same as the number of levels of container composition.
  ///
  /// The result of the constructor is a container composition where the size
  /// and distribution of the container elements at a given level of the
  /// composition are the same.
  ///
  /// @param dist_specs distribution specifications that are used to construct
  /// the nested containers at a given level of the composition.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  directed_graph_base(std::vector<DistSpecView> const& dist_specs,
                      typename std::enable_if<
                        is_distribution_view<DistSpecView>::value &&
                        !detail::has_is_composed_dist_spec<DistSpecView>::value
                      >::type* = 0)
    : base_type(
        std::shared_ptr<DistSpecView>(new DistSpecView(dist_specs[0])),
        make_composed_dist_spec(
          [&](std::vector<size_t> const& index) {
            stapl_assert(index.size() < dist_specs.size(),
              "dimensionality of index exceeds depth of distribution specs");
            return dist_specs[index.size()];
        })
      )
  { }
  /// @}

  /// @name Trait Reflection
  /// @{

  bool is_directed(void) const
  { return true; }


  //////////////////////////////////////////////////////////////////////
  /// @brief Sorts edges of each vertex by home-location of target-vertex.
  //////////////////////////////////////////////////////////////////////
  void sort_edges(void)
  {
    this->distribution().sort_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sorts edges of each vertex by user-defined comparison function.
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp const& comp)
  {
    this->distribution().sort_edges(comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges that match a user-defined predicate
  /// @param pred A unary predicate that receives a single edge
  //////////////////////////////////////////////////////////////////////
  template<typename Pred>
  void erase_edges_if(Pred&& pred)
  {
    this->distribution().erase_edges_if(std::forward<Pred>(pred));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all duplicate edges with the same (source, target) pair
  //////////////////////////////////////////////////////////////////////
  void remove_duplicate_edges()
  {
    this->distribution().remove_duplicate_edges();
  }

  /// @}

  /// @name Vertex Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reference to the vertex with the given descriptor.
  /// @param gid Descriptor of the desired vertex.
  /// @return reference to the desired vertex.
  //////////////////////////////////////////////////////////////////////
  reference operator[](vertex_descriptor const& gid)
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reference to the vertex with the given descriptor.
  /// @param gid Descriptor of the desired vertex.
  /// @return reference to the desired vertex.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(vertex_descriptor const& gid)
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::make_reference(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  const_reference make_reference(vertex_descriptor const& gid) const
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a vertex iterator to the global beginning of the pGraph.
  /// @return vertex_iterator to the global beginning of the pGraph.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator begin(void)
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const vertex iterator to the global beginning of
  /// the pGraph.
  /// @return vertex_iterator to the global beginning of the pGraph.
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator begin(void) const
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a vertex iterator to the global end of the pGraph.
  /// @return vertex_iterator to the global end of the pGraph.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator end(void)
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const vertex iterator to the global end of the pGraph.
  /// @return vertex_iterator to the global end of the pGraph.
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator end(void) const
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a vertex iterator to the given descriptor.
  /// @param gid Descriptor of the desired vertex.
  /// @return vertex_iterator to the desired vertex.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return this->distribution().make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a vertex iterator to the given descriptor with
  /// a specified domain for subsequent iteration through gids.
  /// @param domain The domain to iterate through.
  /// @param gid Descriptor of the desired vertex.
  /// @return vertex_iterator to the desired vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  iterator make_iterator(Domain const& domain, gid_type const& gid)
  {
    return this->distribution().make_iterator(domain, gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a global vertex iterator to the given descriptor.
  /// @param gid Descriptor of the desired vertex.
  /// @return vertex_iterator to the desired vertex.
  ///
  /// Wrapper around make_iterator for compatibility. Does not enforce
  /// the existence of the vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find_vertex(vertex_descriptor const& gid)
  {
    return this->distribution().find_vertex(gid);
  }

  /// @}

  /// @name Edge Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and property.
  /// @warning Synchronous. Edge is added before returning.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// The id of the edge_descriptor returned is set to
  /// numeric_limits<size_t>::max() if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed,
                           edge_property const& ep = edge_property())
  {
    return this->distribution().add_edge(ed, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices with given property.
  /// @warning Synchronous. Edge is added before returning.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// The id of the edge_descriptor returned is set to
  /// numeric_limits<size_t>::max() if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& ep = edge_property())
  {
    return this->distribution().add_edge(edge_descriptor(source,target), ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge with given descriptor and property.
  /// @warning Synchronous. Edge is inserted before returning.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @param comp Comparator workfunction passed to std::lower_bound to
  /// determine where to insert the edge.
  /// @return edge_descriptor of the added edge.
  /// The id of the edge_descriptor returned is set to
  /// numeric_limits<size_t>::max() if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(edge_descriptor const& ed,
                              edge_property const& ep, Comp const& comp)
  {
    return this->distribution().insert_edge(ed, ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices with given property.
  /// @warning Synchronous. Edge is added before returning.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  /// @param ep Property of the edge.
  /// @param comp Comparator workfunction passed to std::lower_bound to
  /// determine where to insert the edge.
  /// @return edge_descriptor of the added edge.
  /// The id of the edge_descriptor returned is set to
  /// numeric_limits<size_t>::max() if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(vertex_descriptor const& source,
                              vertex_descriptor const& target,
                              edge_property const& ep, Comp const& comp)
  {
    return this->distribution().insert_edge(edge_descriptor(source,target),
                                            ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and property.
  /// The edge is added asynchronously and method returns immediately.
  /// Edge is not guaranteed to be added until after a global synchronization.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed,
                      edge_property const& ep = edge_property())
  {
    this->distribution().add_edge_async(ed, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices with given property.
  /// The edge is added asynchronously and method returns immediately.
  /// Edge is not guaranteed to be added until after a global synchronization.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(vertex_descriptor const& source,
                      vertex_descriptor const& target,
                      edge_property const& ep = edge_property())
  {
    this->distribution().add_edge_async(edge_descriptor(source,target), ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge with given descriptor and property.
  /// The edge is inserted asynchronously and method returns immediately.
  /// Edge is not guaranteed to be inserted until after a global
  /// synchronization.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& ed,
                         edge_property const& ep, Comp const& comp)
  {
    this->distribution().insert_edge_async(ed, ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge with given descriptor and property.
  /// The edge is inserted asynchronously and method returns immediately.
  /// Edge is not guaranteed to be inserted until after a global
  /// synchronization.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(vertex_descriptor const& source,
                         vertex_descriptor const& target,
                         edge_property const& ep, Comp const& comp)
  {
    this->distribution().insert_edge_async(edge_descriptor(source,target),
                                           ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the edge with given descriptor. Asynchronous.
  /// @param ed Descriptor of the desired edge.
  //////////////////////////////////////////////////////////////////////
  void delete_edge(edge_descriptor const& ed)
  {
    this->distribution().delete_edge(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the edge between the given source and target vertices.
  /// The edge is deleted asynchronously. The edge is not guaranteed to have
  /// been deleted until after a global synchronization.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  void delete_edge(vertex_descriptor const& source,
                   vertex_descriptor const& target)
  {
    this->distribution().delete_edge(edge_descriptor(source, target));
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Reserves space for adjacent-edges of the given vertex.
  /// @param vd Descriptor of the desired vertex.
  /// @param num_adjacents Amount of space to be reserved for said vertex.
  ///
  //////////////////////////////////////////////////////////////////////
  void reserve_adjacency(vertex_descriptor const& vd, size_t num_adjacents)
  {
    this->distribution().reserve_adjacency(vd, num_adjacents);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of vertices in the graph.
  ///
  /// This is the same as calling g.size().
  /// This method is one-sided, If other locations may be concurrently
  /// performing operations that change their local size and the effects
  /// are desired to be observed in a deterministic way, then appropriate
  /// synchronization, e.g. a fence, may be required before or after the
  /// call to size, to enforce appropriate ordering.
  //////////////////////////////////////////////////////////////////////
  size_t num_vertices(void) const
  {
    return this->size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the graph. This resets internal counters for
  /// vertex-descriptor and edge-id assignments, and clears graph storage.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->distribution().clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges in the pGraph.
  /// This method is a non-collective version of num_edges.
  /// This must be used when not all locations are calling num_edges.
  ///
  /// For a faster collective, use @ref num_edges_collective() below.
  /// @return size_t number of edges in the pGraph.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return
      this->distribution().num_edges() +
      this->distribution().num_self_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges in the pGraph.
  /// @warning This method is blocking, collective version of num_edges.
  /// Use when calling num-edges from all locations, which is faster than
  /// calling @ref num_edges() above.
  /// @return size_t number of edges in the pGraph.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges_collective(void) const
  {
    return
      this->distribution().num_edges_collective() +
      this->distribution().num_self_edges_collective();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of local outgoing edges in the pGraph.
  /// This is a blocking method.
  /// @return size_t total number of outgoing adjacent-edges for all
  /// local vertices.
  //////////////////////////////////////////////////////////////////////
  size_t num_local_edges(void) const
  {
    return
      this->distribution().num_local_edges() +
      this->distribution().num_self_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Redistribute the data stored in the container to match the
  /// distribution specified by the distribution view provided.
  ///
  /// @param dist_view View-based specification of the desired distribution.
  ///
  /// @note The method is only available in graph instances that use
  /// @ref view_based_partition and @ref view_based_mapper as their
  /// partition and mapper types, respectively.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  void redistribute(DistSpecView const& dist_view,
    typename boost::enable_if<boost::mpl::and_<
      is_distribution_view<DistSpecView>,
      is_view_based<partition_type>,
      is_view_based<mapper_type> > >::type* =0)
  {
    this->distribution().redistribute(
      std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)));
  }

  /// @}

  /// @name Vertex Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function to the vertex with the given descriptor.
  /// @param gid Descriptor of the vertex.
  /// @param f Functor to be applied to the target vertex.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void apply_set(vertex_descriptor const& gid, F const& f)
  {
    this->distribution().apply_set(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function to the property of vertex with given descriptor.
  /// This method is asynchronous.
  /// @param gid Descriptor of the vertex.
  /// @param f Functor to be applied to the target vertex's property.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void vp_apply_async(vertex_descriptor const& gid, F const& f)
  {
    this->distribution().vp_apply_async(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function to the property of vertex with given descriptor.
  /// @warning This method is synchronous.
  /// @param gid Descriptor of the vertex.
  /// @param f Functor to be applied to the target vertex's property.
  /// @return The result of applying f to the vertex's property.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type vp_apply(vertex_descriptor const& gid, F const& f)
  {
    return this->distribution().vp_apply(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function to the property of vertex with given descriptor.
  /// @warning This method is synchronous.
  /// @param gid Descriptor of the vertex.
  /// @param f Functor to be applied to the target vertex's property.
  /// @return The result of applying f to the vertex's property.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type
  vp_apply(vertex_descriptor const& gid, F const& f) const
  {
    return this->distribution().vp_apply(gid, f);
  }

  /// @}

  /// @name Edge Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function to the property of edge with given descriptor.
  /// This method is asynchronous.
  /// @param ed Descriptor of the edge.
  /// @param f Functor to be applied to the target edge's property.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void ep_apply_async(edge_descriptor const& ed, F const& f)
  {
    this->distribution().ep_apply_async(ed, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function to the property of edge with given descriptor.
  /// @warning This method is synchronous.
  /// @param ed Descriptor of the edge.
  /// @param f Functor to be applied to the target edge's property.
  /// @return The result of applying f to the edge's property.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type ep_apply(edge_descriptor const& ed, F const& f)
  {
    return this->distribution().ep_apply(ed, f);
  }

  /// @}

  /// @name Manipulator Aggregation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies functors to respective vertex-properties on the desired
  /// location. For internal use by Aggregators. This method is asynchronous.
  /// @param loc Location where functors are to be executed.
  /// @param fcont Container of functors to be applied. Each element of fcont
  /// should provide a target() method that returns the descriptor of the
  /// target vertex.
  /// Each element of fcont is given the property of its corresponding vertex.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void aggregate_vp_apply_async(size_t const& loc, F const& fcont)
  {
    this->distribution().aggregate_vp_apply_async(loc, fcont);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor on elements at the desired location.
  ///
  /// For internal use by Aggregators. This method is asynchronous.
  /// @param loc Location where functors are to be executed.
  /// @param cont Container of elements to be passed-in to f.
  /// @param f Functor to be applied to each element of cont on the desired
  /// location. f is passed an element of cont.
  //////////////////////////////////////////////////////////////////////
  template <typename Cont, typename F>
  void aggregate_apply_async(size_t const& loc, Cont const& cont, F const& f)
  {
    this->distribution().aggregate_apply_async(loc, cont, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor on the desired location.
  /// For internal use by Aggregators. This method is asynchronous.
  /// @param loc Location where functors are to be executed.
  /// @param cont Container of elements to be passed-in to f.
  /// @param f Functor to be applied to each element of v on the desired
  /// location. f is passed an element of v as well as a pointer to the pGraph.
  //////////////////////////////////////////////////////////////////////
  template <typename Cont, typename F>
  void aggregate_async(size_t const& loc, Cont const& cont, F const& f)
  {
    this->distribution().aggregate_async(loc, cont, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines whether there exists an edge between two vertices
  /// @return A future representing the answer to this query
  //////////////////////////////////////////////////////////////////////
  future<bool> has_edge(vertex_descriptor const& source,
                        vertex_descriptor const& target)
  {
    return this->distribution().has_edge(source, target);
  }

  /// @}
};


//////////////////////////////////////////////////////////////////////
/// @brief Parallel directed-graph container. Inherits all functionality
/// from @ref directed_graph_base.
/// @ingroup pgraphBase
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgePx type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PSx Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam Mapx Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traitsx A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref static_graph_traits.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M, typename ...OptionalParams>
class directed_graph
  : public directed_graph_base<
      directed_graph<D, M, OptionalParams...>, D, M, OptionalParams...>
{
protected:
  typedef directed_graph_base<
    directed_graph, D, M, OptionalParams...> base_type;

public:
  STAPL_IMPORT_TYPE(typename base_type, size_type)
  STAPL_IMPORT_TYPE(typename base_type, partition_type)
  STAPL_IMPORT_TYPE(typename base_type, mapper_type)
  STAPL_IMPORT_TYPE(typename base_type, domain_type)
  STAPL_IMPORT_TYPE(typename base_type, value_type)
  STAPL_IMPORT_TYPE(typename base_type, vertex_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, edge_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, vertex_property)
  STAPL_IMPORT_TYPE(typename base_type, edge_property)

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an empty graph.
  //////////////////////////////////////////////////////////////////////
  directed_graph(void)
    : base_type()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size.
  //////////////////////////////////////////////////////////////////////
  directed_graph(size_t const& n)
    : base_type(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and
  /// constructs all elements with a default value for vertex property.
  //////////////////////////////////////////////////////////////////////
  directed_graph(size_t const& n, vertex_property const& default_value)
    : base_type(n, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and default value for
  /// vertex property.
  //////////////////////////////////////////////////////////////////////
  directed_graph(partition_type const& ps,
                 vertex_property const& default_value=vertex_property())
    : base_type(ps, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and mapper.
  //////////////////////////////////////////////////////////////////////
  directed_graph(partition_type const& ps,
                 mapper_type const& m)
    : base_type(ps, m)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and mapper,
  /// with default value for vertex property.
  //////////////////////////////////////////////////////////////////////
  directed_graph(partition_type const& ps,
                 mapper_type const& m,
                 vertex_property const& default_value)
    : base_type(ps, m, default_value)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and mapper,
  /// with default value for vertex property.
  /// @param n Size of the graph.
  /// @param m Mapper for the data distribution.
  //////////////////////////////////////////////////////////////////////
  directed_graph(size_t const& n, mapper_type const& m,
                 vertex_property const& default_value=vertex_property())
    : base_type(n, m, default_value)
  { }

  template <typename DistSpecsView>
  directed_graph(DistSpecsView const& dist_view,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(dist_view)
  { }

  template <typename DistSpecsView>
  directed_graph(DistSpecsView const& dist_view,
    vertex_property const& default_value,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(dist_view, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and default
  /// value where the vertex_property may itself be a parallel container.
  /// Required for pC composition.
  /// @param n The number of vertices
  /// @param default_value The initial value of the vertices' properties.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename DP>
  directed_graph(size_t n, vertex_property const& default_value,
                 DP const& dis_policy)
    : base_type(n, default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  directed_graph(boost::tuples::cons<X,Y> dims)
    : base_type(dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  directed_graph(boost::tuples::cons<X,Y> dims, const DP& dis_policy)
    : base_type(dims, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  directed_graph(SizesView const& sizes_view,
                 typename boost::enable_if<
                   boost::mpl::and_<
                     boost::is_same<size_type,
                       typename SizesView::size_type>,
                     boost::mpl::not_<is_distribution_view<SizesView> >
                   >
                 >::type* = 0)
    : base_type(sizes_view)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers. For an m level composed
  /// container, @p comp_spec contains specifications of the distributions
  /// of the current container and each of its elements.
  ///
  /// The specification of the current container's distribution is accessed
  /// by calling the @p spec() method, while the distribution specification
  /// of an element is accessed via @p operator[].
  ///
  /// @param comp_spec Instance of @ref composed_dist_spec representing the
  /// distribution specifications for each nested container.
  //////////////////////////////////////////////////////////////////////
  template <typename ComposedSpec>
  directed_graph(ComposedSpec const& comp_spec,
                 typename std::enable_if<
                   detail::has_is_composed_dist_spec<
                     ComposedSpec>::value>::type* = 0)
    : base_type(comp_spec)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  /// container, @p dist_specs contains specifications of the distributions
  /// to be used at each level of the composed container.
  ///
  /// The first element of the vector specifies the distribution of the outer
  /// container, the second the distribution of the containers at the first
  /// level of composition, etc.  The number of elements in @p dist_specs must
  /// be at least the same as the number of levels of container composition.
  ///
  /// The result of the constructor is a container composition where the size
  /// and distribution of the container elements at a given level of the
  /// composition are the same.
  ///
  /// @param dist_specs distribution specifications that are used to construct
  /// the nested containers at a given level of the composition.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  directed_graph(std::vector<DistSpecView> const& dist_specs,
                 typename std::enable_if<
                   is_distribution_view<DistSpecView>::value &&
                   !detail::has_is_composed_dist_spec<
                     DistSpecView>::value>::type* = 0)
    : base_type(dist_specs)
  { }
  /// @}
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for @ref directed_graph.
/// @ingroup pgraphTraits
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M, typename ...OptionalParams>
struct container_traits<directed_graph<D, M, OptionalParams...>>
  : public graph_param_selector<D, M, OptionalParams...>::traits_t
{
  using container_t = directed_graph<D, M, OptionalParams...>;

  using partition_t =
    typename graph_param_selector<D, M, OptionalParams...>::partition_t;

  using mapper_t =
    typename graph_param_selector<D, M, OptionalParams...>::mapper_t;

  using traits_t =
    typename graph_param_selector<D, M, OptionalParams...>::traits_t;

  using dist_t =
    typename traits_t::template construct_distribution<container_t>::type;

  using value_type    = typename traits_t::value_type;
  using domain_type   = typename partition_t::value_type;
  using accessor_type = graph_accessor<dist_t>;
  using reference     = proxy<value_type, accessor_type>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for undirected-graph.
/// @ingroup pgraphBase
/// Inherits all functionality from @ref directed_graph_base, over-writing
/// edge-related methods to provide undirected behaviour.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgePx type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PSx Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam Mapx Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref static_graph_traits.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
class undirected_graph
  : public directed_graph_base<
      undirected_graph<D, M, OptionalParams...>, D, M, OptionalParams...>
{
private:
  typedef directed_graph_base<
    undirected_graph, D, M, OptionalParams...> base_type;

public:
  STAPL_IMPORT_TYPE(typename base_type, size_type)
  STAPL_IMPORT_TYPE(typename base_type, partition_type)
  STAPL_IMPORT_TYPE(typename base_type, mapper_type)
  STAPL_IMPORT_TYPE(typename base_type, domain_type)
  STAPL_IMPORT_TYPE(typename base_type, value_type)
  STAPL_IMPORT_TYPE(typename base_type, vertex_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, edge_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, vertex_property)
  STAPL_IMPORT_TYPE(typename base_type, edge_property)
  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an empty graph.
  //////////////////////////////////////////////////////////////////////
  undirected_graph(void)
    : base_type()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size.
  //////////////////////////////////////////////////////////////////////
  undirected_graph(size_t const& n)
    : base_type(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and
  /// constructs all elements with a default value for vertex property.
  //////////////////////////////////////////////////////////////////////
  undirected_graph(size_t const& n, vertex_property const& default_value)
    : base_type(n, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and default value for
  /// vertex property.
  //////////////////////////////////////////////////////////////////////
  undirected_graph(partition_type const& ps,
                   vertex_property const& default_value=vertex_property())
    : base_type(ps, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and mapper,
  /// with default value for vertex property.
  //////////////////////////////////////////////////////////////////////
  undirected_graph(partition_type const& ps,
                   mapper_type const& m,
                   vertex_property const& default_value=vertex_property())
    : base_type(ps, m, default_value)
  { }

  template <typename DistSpecsView>
  undirected_graph(DistSpecsView const& dist_view,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(dist_view)
  { }

  template <typename DistSpecsView>
  undirected_graph(DistSpecsView const& dist_view,
    vertex_property const& default_value,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(dist_view, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and mapper,
  /// with default value for vertex property.
  /// @param n Size of the graph.
  /// @param m Mapper for the data distribution.
  //////////////////////////////////////////////////////////////////////
  undirected_graph(size_t const& n, mapper_type const& m,
                 vertex_property const& default_value=vertex_property())
    : base_type(n, m, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and default
  /// value where the vertex_property may itself be a parallel container.
  /// Required for pC composition.
  /// @param n The number of vertices
  /// @param default_value The initial value of the vertices' properties.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename DP>
  undirected_graph(size_t n, vertex_property const& default_value,
                   DP const& dis_policy)
    : base_type(n, default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  undirected_graph(boost::tuples::cons<X,Y> dims)
    : base_type(dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  undirected_graph(boost::tuples::cons<X,Y> dims, const DP& dis_policy)
    : base_type(dims, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  undirected_graph(SizesView const& sizes_view,
                   typename boost::enable_if<
                     boost::mpl::and_<
                       boost::is_same<size_type,
                         typename SizesView::size_type>,
                       boost::mpl::not_<is_distribution_view<SizesView> >
                     >
                   >::type* = 0)
    : base_type(sizes_view)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers. For an m level composed
  /// container, @p comp_spec contains specifications of the distributions
  /// of the current container and each of its elements.
  ///
  /// The specification of the current container's distribution is accessed
  /// by calling the @p spec() method, while the distribution specification
  /// of an element is accessed via @p operator[].
  ///
  /// @param comp_spec Instance of @ref composed_dist_spec representing the
  /// distribution specifications for each nested container.
  //////////////////////////////////////////////////////////////////////
  template <typename ComposedSpec>
  undirected_graph(ComposedSpec const& comp_spec,
                   typename std::enable_if<
                     detail::has_is_composed_dist_spec<
                       ComposedSpec>::value>::type* = 0)
    : base_type(comp_spec)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  /// container, @p dist_specs contains specifications of the distributions
  /// to be used at each level of the composed container.
  ///
  /// The first element of the vector specifies the distribution of the outer
  /// container, the second the distribution of the containers at the first
  /// level of composition, etc.  The number of elements in @p dist_specs must
  /// be at least the same as the number of levels of container composition.
  ///
  /// The result of the constructor is a container composition where the size
  /// and distribution of the container elements at a given level of the
  /// composition are the same.
  ///
  /// @param dist_specs distribution specifications that are used to construct
  /// the nested containers at a given level of the composition.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  undirected_graph(std::vector<DistSpecView> const& dist_specs,
                   typename std::enable_if<
                     is_distribution_view<DistSpecView>::value &&
                     !detail::has_is_composed_dist_spec<
                       DistSpecView>::value>::type* = 0)
    : base_type(dist_specs)
  { }

  /// @}

  /// @name Trait Reflection
  /// @{

  bool is_directed(void) const
  { return false; }

  /// @}

  /// @name Edge Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and property.
  /// @warning This method is synchronous.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// Descriptor.id() is set to numeric_limits<size_t>::max() if edge was
  /// not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed,
                           edge_property const& ep = edge_property())
  {
    edge_descriptor ned = this->distribution().add_edge(ed, ep);
    return ned;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices with given property.
  /// @warning This method is synchronous.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// Descriptor.id() is set to numeric_limits<size_t>::max() if edge was
  /// not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& ep = edge_property())
  {
    return this->add_edge(edge_descriptor(source,target), ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge with given descriptor and property.
  /// @warning Synchronous. Edge is inserted before returning.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @param comp Comparator workfunction passed to std::lower_bound to
  /// determine where to insert the edge.
  /// @return edge_descriptor of the added edge.
  /// The id of the edge_descriptor returned is set to
  /// numeric_limits<size_t>::max() if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(edge_descriptor const& ed,
                           edge_property const& ep, Comp const& comp)
  {
    return this->distribution().insert_edge(ed, ep, comp);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices with given property.
  /// @warning Synchronous. Edge is added before returning.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  /// @param ep Property of the edge.
  /// @param comp Comparator workfunction passed to std::lower_bound to
  /// determine where to insert the edge.
  /// @return edge_descriptor of the added edge.
  /// The id of the edge_descriptor returned is set to
  /// numeric_limits<size_t>::max() if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& ep, Comp const& comp)
  {
    return this->insert_edge(edge_descriptor(source,target), ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and property.
  /// This method is asynchronous.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed,
                      edge_property const& ep = edge_property())
  {
    this->distribution().add_edge_async(ed, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices with given property.
  /// This method is asynchronous.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  inline void add_edge_async(vertex_descriptor const& source,
                             vertex_descriptor const& target,
                             edge_property const& ep = edge_property())
  {
    this->add_edge_async(edge_descriptor(source,target), ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge with given descriptor and property.
  /// The edge is inserted asynchronously and method returns immediately.
  /// Edge is not guaranteed to be inserted until after a global
  /// synchronization.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& ed,
                         edge_property const& ep, Comp const& comp)
  {
    this->distribution().insert_edge_async(ed, ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge with given descriptor and property.
  /// The edge is inserted asynchronously and method returns immediately.
  /// Edge is not guaranteed to be inserted until after a global
  /// synchronization.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(vertex_descriptor const& source,
                         vertex_descriptor const& target,
                         edge_property const& ep, Comp const& comp)
  {
    this->distribution().insert_edge_async(edge_descriptor(source,target),
                                           ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the edge with given descriptor.
  /// This method is asynchronous.
  /// @param ed Descriptor of the desired edge.
  //////////////////////////////////////////////////////////////////////
  inline void delete_edge(edge_descriptor const& ed)
  {
    this->distribution().delete_edge(ed);
    this->distribution().delete_edge(reverse(ed));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the edge between the given source and target vertices.
  /// This method is asynchronous.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  inline void delete_edge(vertex_descriptor const& source,
                          vertex_descriptor const& target)
  {
    this->delete_edge(edge_descriptor(source, target));
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of edges in the pGraph.
  /// This is a blocking, non-collective version of num_edges.
  /// For faster collective, use num_edges_collective() below.
  /// @return size_t number of edges in the pGraph.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return
      this->distribution().num_edges() / 2 +
      this->distribution().num_self_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges in the pGraph.
  /// @warning This is a blocking, collective version of @ref num_edges().
  /// Use when calling num-edges from all locations, which is faster than
  /// calling @ref num_edges() above.
  /// @return size_t number of edges in the pGraph.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges_collective(void) const
  {
    return
      this->distribution().num_edges_collective() / 2 +
      this->distribution().num_self_edges_collective();
  }

  /// @}
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for @ref undirected_graph.
/// @ingroup pgraphTraits
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam M Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M, typename ...OptionalParams>
struct container_traits<undirected_graph<D, M, OptionalParams...>>
  : public graph_param_selector<D, M, OptionalParams...>::traits_t
{
  using container_t = undirected_graph<D, M, OptionalParams...>;

  using partition_t =
    typename graph_param_selector<D, M, OptionalParams...>::partition_t;

  using mapper_t =
    typename graph_param_selector<D, M, OptionalParams...>::mapper_t;

  using traits_t =
    typename graph_param_selector<D, M, OptionalParams...>::traits_t;

  using dist_t =
    typename traits_t::template construct_distribution<container_t>::type;

  using value_type    = typename traits_t::value_type;
  using domain_type   = typename partition_t::value_type;
  using accessor_type = graph_accessor<dist_t>;
  using reference     = proxy<value_type, accessor_type>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for selecting between directed and undirected graphs.
/// @ingroup pgraphImpl
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam M Mapper that defines how to map the subdomains produced
/// by the partition to locations.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc.
/// @tparam DirectedType graph-attribute specifying Directedness
/// (DIRECTED/UNDIRECTED).
//////////////////////////////////////////////////////////////////////
template<graph_attributes DirectedType, graph_attributes M,
         typename ...OptionalParams>
struct graph_directedness_container_selector
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref graph_directedness_container_selector for
/// DIRECTED graph.
/// @ingroup pgraphImpl
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
//////////////////////////////////////////////////////////////////////
template<graph_attributes M, typename ...OptionalParams>
struct graph_directedness_container_selector<DIRECTED, M, OptionalParams...>
{
  using type = directed_graph<DIRECTED, M, OptionalParams...>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref graph_directedness_container_selector for
/// UNDIRECTED graph.
/// @ingroup pgraphImpl
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
//////////////////////////////////////////////////////////////////////
template<graph_attributes M, typename ...OptionalParams>
struct graph_directedness_container_selector<UNDIRECTED, M, OptionalParams...>
{
  using type = undirected_graph<UNDIRECTED, M, OptionalParams...>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Parallel static graph container. Inherits all functionality
/// from either @ref undirected_graph or @ref directed_graph.
/// @ingroup pgraphGeneral
///
/// Static graphs do not allow addition or deletion of vertices. The number of
/// vertices must be known at construction. Edges may be added/deleted.
/// Uses directedness selector to inherit from correct directed/undirected base.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgePx type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PSx Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam Mapx Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traitsx A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref static_graph_traits.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
class graph
  : public graph_directedness_container_selector<
      D, M, OptionalParams...>::type
{
protected:
  using base_type = typename graph_directedness_container_selector<
      D, M, OptionalParams...>::type;
  using this_type = graph<D, M, OptionalParams...>;

public:
  STAPL_IMPORT_TYPE(typename base_type, size_type)
  STAPL_IMPORT_TYPE(typename base_type, partition_type)
  STAPL_IMPORT_TYPE(typename base_type, mapper_type)
  STAPL_IMPORT_TYPE(typename base_type, distribution_type)
  STAPL_IMPORT_TYPE(typename base_type, domain_type)
  STAPL_IMPORT_TYPE(typename base_type, value_type)
  STAPL_IMPORT_TYPE(typename base_type, vertex_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, edge_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, vertex_property)
  STAPL_IMPORT_TYPE(typename base_type, edge_property)

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an empty graph.
  //////////////////////////////////////////////////////////////////////
  graph(void)
    : base_type()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size.
  //////////////////////////////////////////////////////////////////////
  graph(size_t const& n)
    : base_type(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and
  /// constructs all elements with a default value for vertex property.
  //////////////////////////////////////////////////////////////////////
  graph(size_t const& n, vertex_property const& default_value)
    : base_type(n, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and default value for
  /// vertex property.
  //////////////////////////////////////////////////////////////////////
  graph(partition_type const& ps,
        vertex_property const& default_value=vertex_property())
    : base_type(ps, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and mapper.
  //////////////////////////////////////////////////////////////////////
  graph(partition_type const& ps, mapper_type const& m)
    : base_type(ps, m)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given partition and mapper,
  /// with default value for vertex property.
  //////////////////////////////////////////////////////////////////////
  graph(partition_type const& ps, mapper_type const& m,
        vertex_property const& default_value)
    : base_type(ps, m, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and mapper,
  /// with default value for vertex property.
  /// @param n Size of the graph.
  /// @param m Mapper for the data distribution.
  //////////////////////////////////////////////////////////////////////
  graph(size_t const& n, mapper_type const& m,
                 vertex_property const& default_value=vertex_property())
    : base_type(n, m, default_value)
  { }

  template <typename DistSpecsView>
  graph(DistSpecsView const& dist_view,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(dist_view)
  { }

  template <typename DistSpecsView>
  graph(DistSpecsView const& dist_view,
    vertex_property const& default_value,
    typename std::enable_if<
      is_distribution_view<DistSpecsView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecsView>::value
    >::type* =0)
    : base_type(dist_view, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph with a given size and default
  /// value where the vertex_property may itself be a parallel container.
  /// Required for pC composition.
  /// @param n The number of vertices
  /// @param default_value The initial value of the vertices' properties.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename DP>
  graph(size_t n, vertex_property const& default_value,
        DP const& dis_policy)
    : base_type(n, default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given distribution policy.
  /// Required for pC composition.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename DP>
  graph(size_t n, DP const& dis_policy)
    : base_type(n, define_value_type<vertex_property>::default_value(),
                dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  graph(boost::tuples::cons<X,Y> dims)
    : base_type(dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates composed pContainers with a given size-specifications.
  /// Required for pC composition.
  /// @param dims dimensions of the internal containers.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  graph(boost::tuples::cons<X,Y> dims, const DP& dis_policy)
    : base_type(dims, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  graph(SizesView const& sizes_view,
        typename boost::enable_if<
          boost::mpl::and_<
            boost::is_same<size_type,
              typename SizesView::size_type>,
            boost::mpl::not_<is_distribution_view<SizesView> >
          >
        >::type* = 0)
    : base_type(sizes_view)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers. For an m level composed
  /// container, @p comp_spec contains specifications of the distributions
  /// of the current container and each of its elements.
  ///
  /// The specification of the current container's distribution is accessed
  /// by calling the @p spec() method, while the distribution specification
  /// of an element is accessed via @p operator[].
  ///
  /// @param comp_spec Instance of @ref composed_dist_spec representing the
  /// distribution specifications for each nested container.
  //////////////////////////////////////////////////////////////////////
  template <typename ComposedSpec>
  graph(ComposedSpec const& comp_spec,
        typename std::enable_if<
          detail::has_is_composed_dist_spec<ComposedSpec>::value>::type* = 0)
    : base_type(comp_spec)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  /// container, @p dist_specs contains specifications of the distributions
  /// to be used at each level of the composed container.
  ///
  /// The first element of the vector specifies the distribution of the outer
  /// container, the second the distribution of the containers at the first
  /// level of composition, etc.  The number of elements in @p dist_specs must
  /// be at least the same as the number of levels of container composition.
  ///
  /// The result of the constructor is a container composition where the size
  /// and distribution of the container elements at a given level of the
  /// composition are the same.
  ///
  /// @param dist_specs distribution specifications that are used to construct
  /// the nested containers at a given level of the composition.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  graph(std::vector<DistSpecView> const& dist_specs,
        typename std::enable_if<
          is_distribution_view<DistSpecView>::value &&
          !detail::has_is_composed_dist_spec<DistSpecView>::value>::type* = 0)
    : base_type(dist_specs)
  { }

  boost::shared_ptr<this_type> shared_from_this()
  {
    return boost::static_pointer_cast<this_type>(
             boost::enable_shared_from_this<detail::container_impl<base_type
               >>::shared_from_this());
  }
  /// @}
}; // class graph


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for @ref stapl::graph.
/// @ingroup pgraphTraits
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
struct container_traits<graph<D,M,OptionalParams...> >
  : graph_param_selector<D,M,OptionalParams...>::traits_t
{
  typedef graph<D,M,OptionalParams...>                   container_t;

  typedef typename graph_param_selector<
    D, M, OptionalParams...>::partition_t                partition_t;
  typedef typename graph_param_selector<
    D, M, OptionalParams...>::mapper_t                   mapper_t;
  typedef typename graph_param_selector<
    D, M, OptionalParams...>::traits_t                   traits_t;

  typedef typename traits_t::value_type                  value_type;
  typedef typename traits_t::
    template construct_distribution<container_t>::type   dist_t;
  typedef typename partition_t::value_type               domain_type;
  typedef graph_accessor<dist_t>                         accessor_type;
  typedef proxy<value_type, accessor_type>               reference;
};

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_HPP
