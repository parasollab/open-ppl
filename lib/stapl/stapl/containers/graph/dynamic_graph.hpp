/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DYNAMIC_GRAPH_HPP
#define STAPL_CONTAINERS_DYNAMIC_GRAPH_HPP

#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/traits/dynamic_graph_traits.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>

#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for selecting between default and optional template parameters
/// for @ref dynamic_graph.
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
struct dgraph_param_selector
{
  typedef tuple<
    properties::no_property,
    properties::no_property,
    balanced_partition<indexed_domain<size_t> >, // Partition
    mapper<size_t>,                              // Mapper
    dynamic_graph_traits<D, M, properties::no_property, properties::no_property,
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
    dynamic_graph_traits<D, M, vertex_prop, edge_prop, partition_t, mapper_t>
    >                                                   default_traits_type;

  typedef typename compute_type_parameters<
    default_traits_type, OptionalParams...
  >::type                                               traits_param_type;

  typedef typename tuple_element<4, traits_param_type>::type  traits_t;
};


//////////////////////////////////////////////////////////////////////
/// @brief Implementation of dynamic graph that supports addition and deletion
/// of vertices and edges.
/// @ingroup pgraphGeneral
///
/// Inherits from @ref stapl::graph and adds functionality to add/delete
/// vertices.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgePx type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PSx Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition over  an @ref indexed_domain.
/// @tparam Mapx Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref dynamic_graph_traits.
//////////////////////////////////////////////////////////////////////
/*
template <graph_attributes D, graph_attributes M,
          typename VertexP = properties::no_property,
          typename EdgeP   = properties::no_property,
          typename PS      = balanced_partition< indexed_domain<size_t> > ,
          typename Map     = mapper<size_t>,
          typename Traits = dynamic_graph_traits<D,M, VertexP, EdgeP, PS, Map> >
*/
template<graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
class dynamic_graph
  : public graph<D, M,
  typename dgraph_param_selector<D, M, OptionalParams...>::vertex_prop,
  typename dgraph_param_selector<D, M, OptionalParams...>::edge_prop,
  typename dgraph_param_selector<D, M, OptionalParams...>::partition_t,
  typename dgraph_param_selector<D, M, OptionalParams...>::mapper_t,
  typename dgraph_param_selector<D, M, OptionalParams...>::traits_t>
{
protected:

  typedef typename dgraph_param_selector<
    D, M, OptionalParams...>::vertex_prop        VertexP;
  typedef typename dgraph_param_selector<
    D, M, OptionalParams...>::edge_prop          EdgeP;
  typedef typename dgraph_param_selector<
    D, M, OptionalParams...>::partition_t        PS;
  typedef typename dgraph_param_selector<
    D, M, OptionalParams...>::mapper_t           Map;
  typedef typename dgraph_param_selector<
    D, M, OptionalParams...>::traits_t           Traits;

  typedef graph<D, M, VertexP, EdgeP, PS, Map, Traits>       base_type;
  typedef dynamic_graph<D, M, OptionalParams...>             this_type;
  typedef typename graph_directedness_container_selector<
    D, M, VertexP, EdgeP, PS, Map, Traits>::type             directed_base_type;

public:
  STAPL_IMPORT_TYPE(typename base_type, size_type)
  STAPL_IMPORT_TYPE(typename base_type, partition_type)
  STAPL_IMPORT_TYPE(typename base_type, mapper_type)
  STAPL_IMPORT_TYPE(typename base_type, distribution_type)
  STAPL_IMPORT_TYPE(typename base_type, value_type)
  STAPL_IMPORT_TYPE(typename base_type, vertex_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, edge_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, vertex_property)
  STAPL_IMPORT_TYPE(typename base_type, edge_property)

  typedef typename partition_type::domain_type   descriptor_domain_type;
  typedef domainset1D<distribution_type>         domain_type;
  typedef typename mapper_type::domain_type      map_dom_t;

  /// @name Constructors
  /// @{

  dynamic_graph(void)
    : base_type(partition_type(descriptor_domain_type(), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t const&)
  //////////////////////////////////////////////////////////////////////
  dynamic_graph(size_t const& n)
    : base_type(n)
  { stapl_assert(n != 0, "Size should be non-zero"); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  dynamic_graph(size_t const& n, vertex_property const& default_value)
    : base_type(partition_type(descriptor_domain_type(0, n-1, true),
                             get_num_locations()),
              mapper_type(map_dom_t(get_num_locations())),
              default_value)
  { stapl_assert(n != 0, "Size should be non-zero"); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(partition_type const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  dynamic_graph(partition_type const& ps,
                vertex_property const& default_value=vertex_property())
    : base_type(ps, mapper_type(ps.domain()), default_value)
  { }

  dynamic_graph(partition_type const& ps,
                mapper_type const& mapper,
                vertex_property const& default_value=vertex_property())
    : base_type(ps, mapper, default_value)
  { }

  template <typename DistSpecsView>
  dynamic_graph(DistSpecsView const& dist_view,
    typename boost::enable_if<is_distribution_view<DistSpecsView> >::type* =0)
    : base_type(dist_view)
  { }

  template <typename DistSpecsView>
  dynamic_graph(DistSpecsView const& dist_view,
                vertex_property const& default_value,
    typename boost::enable_if<is_distribution_view<DistSpecsView> >::type* =0)
    : base_type(dist_view, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t, vertex_property const&, DP const&)
  //////////////////////////////////////////////////////////////////////
  template <typename DP>
  dynamic_graph(size_t n, vertex_property const& default_value,
                DP const& dis_policy)
    : base_type(n, default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(boost::tuples::cons<X,Y>)
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  dynamic_graph(boost::tuples::cons<X,Y> dims)
    : base_type(dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(boost::tuples::cons<X,Y>, DP const&)
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  dynamic_graph(boost::tuples::cons<X,Y> dims, DP const& dis_policy)
    : base_type(dims, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  dynamic_graph(SizesView const& sizes_view,
                typename boost::enable_if<
                  boost::mpl::and_<
                    boost::is_same<size_type, typename SizesView::size_type>,
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
  dynamic_graph(ComposedSpec const& comp_spec,
                typename std::enable_if<
                  detail::has_is_composed_dist_spec<ComposedSpec>::value
                >::type* = 0)
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
  dynamic_graph(std::vector<DistSpecView> const& dist_specs,
                typename std::enable_if<
                  is_distribution_view<DistSpecView>::value &&
                  !detail::has_is_composed_dist_spec<DistSpecView>::value
                >::type* = 0)
    : base_type(dist_specs)
  { }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an @ref iterator_domain of this pGraph.
  /// @return iterator_domain over the pGraph.
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(this->distribution());
  }

  /// @}

  /// @name Vertex Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to the pGraph with a default-constructed property.
  /// @return vertex_descriptor of the added vertex.
  ///
  /// Vertex-descriptor is assigned automatically by the pGraph.
  /// This method is asynchronous. The vertex is added at the calling location,
  /// which is also the home location for the vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(void)
  {
    this->incr_version();
    return this->distribution().add_vertex(vertex_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to the pGraph with the given property.
  /// @param vp Property of the vertex.
  /// @return vertex_descriptor of the added vertex.
  ///
  /// Vertex-descriptor is assigned automatically by the pGraph.
  /// This method is asynchronous. The vertex is added at the calling location,
  /// which is also the home location for the vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    this->incr_version();
    return this->distribution().add_vertex(vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to the pGraph with the given property and descriptor.
  /// @param gid descriptor of the vertex.
  /// @param vp Property of the vertex.
  /// @return vertex_descriptor of the added vertex.
  ///
  /// This method is asynchronous. The vertex is added at the calling location.
  /// An async is sent to the home-location of the vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor const& gid,
                               vertex_property const& vp)
  {
    this->incr_version();
    return this->distribution().add_vertex(gid, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to the pGraph with the given property and descriptor,
  /// if the vertex does not exist, or applies the given functor to the existing
  /// vertex. The vertex is added to the home location, unlike the other
  /// add_vertex calls that add the vertex at the current location.
  /// @param gid descriptor of the vertex.
  /// @param vp Property of the vertex.
  /// @param f Function to apply to the vertex if it already exists.
  ///
  /// This method is asynchronous. The vertex is added at the home-location.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void add_vertex(vertex_descriptor const& gid, vertex_property const& vp,
                  Functor const& f)
  {
    this->incr_version();
    return this->distribution().add_vertex(gid, vp, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to the pGraph with the given property to a
  ///        location based on the vertex descriptor assigned by the graph.
  ///        This method is asynchronous. This method differs from the
  ///        typical add_vertex as it inserts the vertex into a potentially
  ///        remote location, rather than the calling location.
  ///
  /// @param vp Property of the vertex.
  /// @return vertex_descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex_uniform(vertex_property const& vp)
  {
    this->incr_version();
    return this->distribution().add_vertex_uniform(vp);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a vertex from the pGraph with the given descriptor.
  /// This method is asynchronous.
  /// @param gid descriptor of the vertex.
  //////////////////////////////////////////////////////////////////////
  void delete_vertex(vertex_descriptor const& gid)
  {
    this->incr_version();
    this->distribution().delete_vertex(gid);
  }

  boost::shared_ptr<this_type> shared_from_this()
  {
    return boost::static_pointer_cast<this_type>(
             boost::enable_shared_from_this<detail::container_impl<
               directed_base_type>>::shared_from_this());
  }
  /// @}
}; // class dynamic_graph


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for
/// @ref stapl::dynamic_graph.
/// @ingroup pgraphTraits
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
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
struct container_traits<dynamic_graph<D,M,OptionalParams...> >
  : graph_param_selector<D,M, OptionalParams...>::traits_t
{
  typedef dynamic_graph<D,M,OptionalParams...>         container_t;

  typedef typename graph_param_selector<
    D, M, OptionalParams...>::partition_t              partition_t;
  typedef typename graph_param_selector<
    D, M, OptionalParams...>::mapper_t                 mapper_t;
  typedef typename graph_param_selector<
    D, M, OptionalParams...>::traits_t                 traits_t;

  typedef typename container_t::value_type             value_type;
  typedef typename container_t::domain_type            domain_type;

  typedef typename traits_t::
    template construct_distribution<container_t>::type dist_t;
  typedef graph_accessor<dist_t>                       accessor_type;
  typedef proxy<value_type, accessor_type>             reference;
};

} // namespace stapl

#endif // STAPL_CONTAINERS_DYNAMIC_GRAPH_HPP
