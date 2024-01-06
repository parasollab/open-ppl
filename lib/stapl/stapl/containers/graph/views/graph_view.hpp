/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_GRAPH_VIEW_HPP
#define STAPL_GRAPH_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/operations/read_write.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/views/operations/subscript.hpp>
#include <stapl/views/type_traits/select_derive.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/utility/use_default.hpp>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Constructs a vertex iterator on the graph_view using the
/// information from the iterator provided by the underlying container.
///
/// This helper struct is used in find_vertex to handle the cases where
/// the view is defined over a base container and the graph view's
/// vertex iterator type matches the base container's vertex iterator type.
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator>
struct build_iterator
{
  template <typename Container, typename ContainerIterator>
  static VertexIterator apply(Container const* c, ContainerIterator i)
  { return VertexIterator(c, (*i).descriptor()); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for local_iterator used when view is defined
/// over a base container and the vertex iterator types match.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename... OptionalParams>
struct build_iterator<local_iterator<Container, OptionalParams...>>
{
  template <typename GraphView, typename ContainerIterator>
  static local_iterator<Container, OptionalParams...>
  apply(GraphView const* c, ContainerIterator i)
  { return i; }
};

} // namespace view_impl

//////////////////////////////////////////////////////////////////////
/// @brief View for the pGraph.
/// Reflects all operations of a graph.
/// @tparam PG The container for the view.
/// @tparam Dom The domain for the view.
/// @tparam MapFunc The mapping function to be used in the view.
/// Default is @ref f_ident.
/// @tparam Derived Most derived class for use with CRTP.
/// @ingroup graph_view
//////////////////////////////////////////////////////////////////////
template <typename PG,
          typename Dom     = typename container_traits<PG>::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>,
          typename Derived = use_default>
class graph_view
  : public core_view<PG,Dom,MapFunc>,
    public view_operations::readwrite<graph_view<PG,Dom,MapFunc,Derived> >,
    public view_operations::subscript<graph_view<PG,Dom,MapFunc,Derived> >,
    public view_operations::sequence<
                 typename select_derived<
                   Derived,
                   graph_view<PG,Dom,MapFunc,Derived> >::type>
{
  typedef core_view<PG,Dom,MapFunc>                      base_type;

  typedef typename select_derived<
    Derived,
    graph_view<PG, Dom, MapFunc, Derived>
  >::type                                                derived_type;

  typedef view_operations::sequence<derived_type>        sequence_op_type;

 public:
  STAPL_VIEW_REFLECT_TRAITS(graph_view)

 private:
  /// Work function to convert iterator_domain view to domset1D view
  template<typename SparseDomain>
  struct get_sparse_domain
  {
    typedef SparseDomain result_type;
    template<typename Bc>
    result_type operator()(Bc bc)
    {
      typename Bc::domain_type dom = bc.domain();
      typedef typename SparseDomain::gid_type gid_type;
      SparseDomain result;
      gid_type i = dom.first();
      gid_type i_last = dom.last();
      for (; i != i_last; i = dom.advance(i, 1))
      {
        result += i;
      }
      result += i_last;
      return result;
    }
  };

 public:
  /// Type for global-iterator over vertices (compatibility).
  typedef typename sequence_op_type::iterator  iterator;
  typedef typename sequence_op_type::const_iterator  const_iterator;
  /// Type for global-iterator over vertices.
  typedef typename sequence_op_type::iterator  vertex_iterator;
  typedef typename sequence_op_type::const_iterator  const_vertex_iterator;
  /// Type for iterator over adjacent edges of a vertex.
  typedef typename PG::adj_edge_iterator       adj_edge_iterator;
  typedef typename PG::const_adj_edge_iterator const_adj_edge_iterator;
  typedef typename PG::vertex_property         vertex_property;
  typedef typename PG::edge_property           edge_property;
  typedef typename PG::vertex_descriptor       vertex_descriptor;
  typedef typename PG::edge_descriptor         edge_descriptor;
  typedef typename PG::vertex_reference        vertex_reference;

  typedef array_view<PG,Dom>                   vertices_view_type;

  graph_view(void)                         = default;
  graph_view(graph_view const&)            = default;
  graph_view(graph_view&&)                 = default;
  graph_view& operator=(graph_view const&) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  graph_view(view_container_type* vcont, domain_type const& dom,
             map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  /// @param other View to copy from.
  //////////////////////////////////////////////////////////////////////
  template <typename OV>
  graph_view(view_container_type* vcont, domain_type const& dom,
             map_func_type mfunc,
             OV const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  graph_view(view_container_type const& vcont, domain_type const& dom,
             map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  graph_view(view_container_type const& vcont, domain_type const& dom,
             map_func_type mfunc, graph_view const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  /// @param other View to copy from.
  //////////////////////////////////////////////////////////////////////
  template <typename OV>
  graph_view(view_container_type const& vcont, domain_type const& dom,
             map_func_type mfunc,
             OV const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view takes ownership of the container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  graph_view(view_container_type* vcont)
    : base_type(vcont, vcont->domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  graph_view(view_container_type& vcont)
    : base_type(vcont, vcont.domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  graph_view(view_container_type const& vcont)
    : base_type(vcont, vcont.domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the passed view is not the most
  ///        derived view.
  //////////////////////////////////////////////////////////////////////
  template<typename Derived1>
  graph_view(graph_view<PG, Dom, MapFunc, Derived1> const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the passed view has an iterator domain.
  /// This constructor converts a view using the default graph domain
  /// (iterator_domain) to view using a gid-based domain (domset1D) as
  /// iterator domain can't handle a selective set of elements.
  /// @todo We need to replace the default graph domain with
  /// a gid-based domain and this constructor can die. However, performance
  /// might be an issue when using gid-based domains for dynamic graphs
  /// with sparse gids.
  //////////////////////////////////////////////////////////////////////
  template <typename T1, typename T2>
  graph_view(graph_view<PG, iterator_domain<T1, T2>, MapFunc, Derived>
               const& other)
    : base_type(other.container(), domain_type(), other.mapfunc())
  {
    domain_type sparse_domain =
      map_reduce(get_sparse_domain<domain_type>(),plus<domain_type>(),
                 stapl::native_view(other));
    this->set_domain(sparse_domain);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(void)
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(void)
  {
    this->incr_version();
    return this->container().add_vertex();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    this->incr_version();
    return this->container().add_vertex(vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex_uniform(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex_uniform(vertex_property const& vp)
  {
    return this->container().add_vertex_uniform(vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_descriptor const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_descriptor const& vd, vertex_property const& vp)
  {
    this->incr_version();
    this->container().add_vertex(vd, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_descriptor const&, vertex_property const&, Functor const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void add_vertex(vertex_descriptor const& vd, vertex_property const& vp,
                  Functor const& f)
  {
    this->incr_version();
    this->container().add_vertex(vd, vp, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::delete_vertex(vertex_descriptor)
  //////////////////////////////////////////////////////////////////////
  void delete_vertex(vertex_descriptor const& vd)
  {
    this->incr_version();
    this->container().delete_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices with given property.
  /// The edge is added asynchronously and method returns immediately.
  /// Edge is not guaranteed to be added until after a global synchronization.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(vertex_descriptor const& src,
                      vertex_descriptor const& tgt)
  {
    this->container().add_edge_async(src, tgt);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::add_edge_async(vertex_descriptor const&, vertex_descriptor const&, edge_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(vertex_descriptor const& src,
                      vertex_descriptor const& tgt,
                      edge_property const& p)
  {
    this->container().add_edge_async(src, tgt, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor.
  /// The edge is added asynchronously and method returns immediately.
  /// Edge is not guaranteed to be added until after a global synchronization.
  /// @param ed Descriptor of the desired edge.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed)
  {
    this->container().add_edge_async(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::add_edge_async(edge_descriptor const&, edge_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed, edge_property const& p)
  {
    this->container().add_edge_async(ed, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge between the two given vertices.
  /// @warning Synchronous. Edge is added before returning.
  /// @param source Descriptor of the source vertex.
  /// @param target Descriptor of the target vertex.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& src,
                           vertex_descriptor const& tgt)
  {
    return this->container().add_edge(src, tgt);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::add_edge(vertex_descriptor const&, vertex_descriptor const&, edge_property const&)
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& src,
                           vertex_descriptor const& tgt,
                           edge_property const& p)
  {
    return this->container().add_edge(src, tgt, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor.
  /// @warning Synchronous. Edge is added before returning.
  /// @param ed Descriptor of the desired edge.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return this->container().add_edge(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::add_edge(edge_descriptor const&, edge_property const&)
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed, edge_property const& p)
  {
    return this->container().add_edge(ed, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::insert_edge(vertex_descriptor const&, vertex_descriptor const&, edge_property const&, Comp const&)
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(vertex_descriptor const& src,
                              vertex_descriptor const& tgt,
                              edge_property const& p, Comp const& comp)
  {
    return this->container().insert_edge(src, tgt, p, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::insert_edge(vertex_descriptor const&, vertex_descriptor const&, edge_property const&, Comp const&)
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(edge_descriptor const& ed,
                              edge_property const& p, Comp const& comp)
  {
    return this->container().insert_edge(ed, p, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::insert_edge_async(vertex_descriptor const&, vertex_descriptor const&, edge_property const&, Comp const&)
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(vertex_descriptor const& src,
                         vertex_descriptor const& tgt,
                         edge_property const& p, Comp const& comp)
  {
    this->container().insert_edge_async(src, tgt, p, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::insert_edge_async(edge_descriptor const&, edge_property const&, Comp const&)
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& ed,
                         edge_property const& p, Comp const& comp)
  {
    this->container().insert_edge_async(ed, p, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::delete_edge(vertex_descriptor const&, vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void delete_edge(vertex_descriptor const& src, vertex_descriptor const& tgt)
  {
    this->container().delete_edge(src, tgt);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::delete_edge(edge_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void delete_edge(edge_descriptor const& ed)
  {
    this->container().delete_edge(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::clear()
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->container().clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::find_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator find_vertex(vertex_descriptor const& vd) const
  {
    return view_impl::build_iterator<const_vertex_iterator>::apply(
      this, this->container().find_vertex(vd));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::num_vertices() const
  //////////////////////////////////////////////////////////////////////
  size_t num_vertices(void) const
  {
    return this->domain().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::num_edges() const
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return this->container().num_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::num_edges_collective()
  //////////////////////////////////////////////////////////////////////
  size_t num_edges_collective(void) const
  {
    return this->container().num_edges_collective();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::num_local_edges()
  //////////////////////////////////////////////////////////////////////
  size_t num_local_edges(void) const
  {
    return this->container().num_local_edges();
  }

  bool is_directed(void) const
  { return this->container().is_directed(); }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::sort_edges()
  //////////////////////////////////////////////////////////////////////
  void sort_edges(void)
  { this->container().sort_edges(); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::sort_edges(Comp const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp const& comp)
  { this->container().sort_edges(comp); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::apply_set(vertex_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void apply_set(vertex_descriptor const& gid, F const& f)
  {
    this->container().apply_set(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::vp_apply_async(vertex_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  void vp_apply_async(vertex_descriptor const& vd , Functor const& f)
  {
    this->container().vp_apply_async(vd, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::vp_apply(vertex_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template <typename Functor>
  typename Functor::result_type vp_apply(vertex_descriptor const& vd,
                                         Functor const& f) const
  {
    return this->container().vp_apply(vd, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::ep_apply_async(edge_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  void ep_apply_async(edge_descriptor const& ed , Functor const& f)
  {
    this->container().ep_apply_async(ed, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::ep_apply(edge_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  typename Functor::result_type ep_apply(edge_descriptor const& ed,
                                         Functor const& f)
  {
    return this->container().ep_apply(ed, f);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an @ref array_view over the vertices of the graph.
  //////////////////////////////////////////////////////////////////////
  vertices_view_type vertices(void) const
  {
    return vertices_view_type(this->container(),this->domain());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines whether there exists an edge between two vertices
  /// @return A future representing the answer to this query
  //////////////////////////////////////////////////////////////////////
  future<bool> has_edge(vertex_descriptor const& source,
                        vertex_descriptor const& target)
  {
    return this->container().has_edge(source, target);
  }
};

} // stapl namespace

#endif /* STAPL_GRAPH_VIEW_HPP */
