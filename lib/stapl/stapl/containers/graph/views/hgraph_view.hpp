/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_VIEW_HGRAPH_VIEW_HPP
#define STAPL_CONTAINERS_GRAPH_VIEW_HGRAPH_VIEW_HPP

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/graph_accessor.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/proxy/index_accessor.hpp>
#include <stapl/views/proxy/const_index_accessor.hpp>
#include <stapl/utility/use_default.hpp>
#include <limits>

// Maximum number of hierarchy levels allowed
#define STAPL_MAX_HIERARCHY_LEVEL 1000

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief pView providing specific methods for hierarchical graphs.
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam MapFunc Mapping function type. (default: identity mapping function).
/// @tparam Derived Type of the most derived class (default: itself).
//////////////////////////////////////////////////////////////////////
template <typename PG,
          typename Dom     = typename container_traits<PG>::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>,
          typename Derived = use_default>
class hgraph_view
  : public graph_view<PG, Dom, MapFunc, hgraph_view<PG, Dom, MapFunc, Derived> >
{
protected:
  /// level of the hierarchy represented by this pView.
  size_t m_level;

public:
  STAPL_VIEW_REFLECT_TRAITS(hgraph_view)
  typedef  hgraph_view<PG, Dom, MapFunc, Derived>  this_type;
  typedef  graph_view<PG, Dom, MapFunc, this_type> base_type;
  typedef typename base_type::vertex_descriptor    vertex_descriptor;
  typedef typename base_type::vertex_property      vertex_property;
  typedef typename Dom::sparse_domain              sparse_domain;

  hgraph_view(void)
    : base_type(), m_level(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  /// @param lvl level of hierarchy
  //////////////////////////////////////////////////////////////////////
  hgraph_view(view_container_type* vcont, domain_type const& dom,
              map_func_type mfunc=MapFunc(), size_t lvl=0)
    : base_type(vcont, dom, mfunc), m_level(lvl)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  /// @param lvl level of hierarchy
  //////////////////////////////////////////////////////////////////////
  hgraph_view(view_container_type const& vcont, domain_type const& dom,
              map_func_type mfunc=MapFunc(), size_t lvl=0)
    : base_type(vcont, dom, mfunc), m_level(lvl)
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
  hgraph_view(view_container_type const& vcont, domain_type const& dom,
              map_func_type mfunc, hgraph_view const& other)
    : base_type(vcont, dom, mfunc), m_level(other.m_level)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param use_container_domain build the view sparse domain
  ///        using the container domain
  //////////////////////////////////////////////////////////////////////
  hgraph_view(view_container_type* vcont, bool use_container_domain = false)
    : base_type(vcont), m_level(0)
  {
    if (!use_container_domain)
      this->set_domain(build_domain_level0(*vcont));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param use_container_domain build the view sparse domain
  ///        using the container domain
  //////////////////////////////////////////////////////////////////////
  hgraph_view(view_container_type const& vcont,
              bool use_container_domain = false)
    : base_type(vcont), m_level(0)
  {
    if (!use_container_domain)
      this->set_domain(build_domain_level0(vcont));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the passed view is not the most
  ///        derived view.
  //////////////////////////////////////////////////////////////////////
  template<typename Derived1>
  hgraph_view(hgraph_view<PG, Dom, MapFunc, Derived1> const& other)
    : base_type(other.get_container(), other.domain(), other.mapfunc()),
      m_level(other.level())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a view over the children of a super-vertex.
  /// @param vd vertex descriptor of the super-vertex.
  /// @return view over the children of a super-vertex.
  //////////////////////////////////////////////////////////////////////
  this_type children(vertex_descriptor const& vd) const
  {
    return this_type(this->container(), this->operator[](vd).children());
  }

  size_t level(void) const
  {
    return m_level;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a vertex to the underlying container.
  /// @return vertex descriptor of the vertex added.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(void)
  {
    vertex_descriptor vid = this->container().add_vertex();
    this->domain().operator+=(vid);
    return vid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a vertex to the underlying container.
  /// @param vp vertex property of the new vertex.
  /// @return vertex descriptor of the vertex added.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    vertex_descriptor vid = this->container().add_vertex(vp);
    this->domain().operator+=(vid);
    return vid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a vertex to the underlying container.
  /// @param vd vertex descriptor of the new vertex.
  /// @param vp vertex property of the new vertex.
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_descriptor const& vd, vertex_property const& vp)
  {
    this->domain().operator+=(vd);

    this->container().add_vertex(vd, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Delete a vertex from the underlying container.
  /// @param vd vertex descriptor of the vertex to delete.
  //////////////////////////////////////////////////////////////////////
  void delete_vertex(vertex_descriptor const& vd)
  {
    this->domain().operator-=(vd);
    this->container().delete_vertex(vd);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_level);
  }

private:
  typedef graph_view <PG, Dom, MapFunc, Derived> graph_view_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Extract the sparse domain from a base container.
  /// @param bc base container.
  /// @return explicit domain of gid required for representing
  ///         a level of hierarchy.
  //////////////////////////////////////////////////////////////////////
  struct get_sparse_domain
  {
    typedef sparse_domain result_type;

    template<typename Bc>
    result_type operator()(Bc bc)
    {
      return bc.domain().get_sparse_domain();
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Build sparse domain of the level 0 of a hierarchical graph.
  /// @param vcont hierarchical graph container.
  /// @return explicit domain of gid required for representing
  ///         a level of hierarchy.
  //////////////////////////////////////////////////////////////////////
  sparse_domain build_domain_level0(view_container_type const& vcont)
  {
    stapl_assert(this->size() != 0,
     "Error: hgraph_view construction with level 0 empty");

    return map_reduce(get_sparse_domain(), plus<sparse_domain>(),
                      stapl::native_view(graph_view_type(vcont)));
  }
}; // class hgraph_view


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief  Specialization of @ref index_accessor for @ref hgraph_view.
/// @tparam PG container type.
/// @tparam Dom domain type.
/// @tparam MapFunc functor to map from view indices to indices of
///         the underlying container.
/// @tparam Derived the most derived type.
//////////////////////////////////////////////////////////////////////
template<typename PG, typename Dom, typename MapFunc, typename Derived>
class index_accessor<hgraph_view<PG, Dom, MapFunc, Derived> >
  : public index_accessor<graph_view<PG, Dom, MapFunc, Derived> >
{
public:
  typedef index_accessor <graph_view<PG, Dom, MapFunc, Derived> > base_type;
  typedef hgraph_view<PG, Dom, MapFunc, Derived>                  View;
  typedef typename view_traits<View>::index_type index_type;

  index_accessor(void)
    : base_type()
  { }

  index_accessor(null_reference const&)
    : base_type()
  { }

  index_accessor(index_accessor const& other)
    : base_type(other)
  { }

  index_accessor( View const* view, index_type const& gid)
  {
    this->m_container = view->get_container();
    this->m_gid       = view->mapfunc()(gid);
  }

  index_accessor(View const& view, index_type const& gid)
  {
    this->m_container = view.get_container();
    this->m_gid       = view.mapfunc()(gid);
  }
}; //class index_accessor, specialized for graph.

//////////////////////////////////////////////////////////////////////
/// @brief  Specialization of @ref const_index_accessor for @ref hgraph_view.
/// @tparam PG container type.
/// @tparam Dom domain type.
/// @tparam MapFunc functor to map from view indices to indices of
///         the underlying container.
/// @tparam Derived the most derived type.
//////////////////////////////////////////////////////////////////////
template<typename PG, typename Dom, typename MapFunc, typename Derived>
class const_index_accessor<hgraph_view<PG, Dom, MapFunc, Derived> >
  : public const_index_accessor<graph_view<PG, Dom, MapFunc, Derived> >
{
public:
  typedef const_index_accessor<graph_view<PG, Dom, MapFunc, Derived> >
    base_type;
  typedef hgraph_view<PG, Dom, MapFunc, Derived> View;
  typedef typename view_traits<View>::index_type  index_type;

  const_index_accessor(void)
    : base_type()
  { }

  const_index_accessor(null_reference const&)
    : base_type()
  { }

  const_index_accessor(const_index_accessor const& other)
    : base_type(other)
  { }

  const_index_accessor(View const* view, index_type const& gid)
  {
    this->m_container = view->get_container();
    this->m_gid       = view->mapfunc()(gid);
  }

  const_index_accessor(View const& view, index_type const& gid)
  {
    this->m_container = view.get_container();
    this->m_gid       = view.mapfunc()(gid);
  }
}; //class index_accessor, specialized for graph.

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Functor computing the vertex descriptor offset for the level
///   above a given one.
///
/// The constructor takes either an offset that will be used between
/// each level or a boolean. In the latter case, the offset will be computed
/// as the max vertex descriptor of the current level.
//////////////////////////////////////////////////////////////////////
struct compute_level_offset
{
private:
  size_t m_offset;
  bool   m_max_computation;

  struct get_vid
  {
    typedef size_t result_type;

    template<typename Vertex>
    size_t operator()(Vertex const& vertex)
    {
      return vertex.descriptor();
    }
  };

public:
  const static size_t default_offset;

  compute_level_offset(size_t offset = default_offset)
    : m_offset(offset), m_max_computation(false)
  { }
  compute_level_offset(bool max_computation)
    : m_offset(0), m_max_computation(true)
  { }

  template<typename GraphView>
  size_t operator()(GraphView& gview) const
  {
    if (m_max_computation)
    {
      return map_reduce(get_vid(), max<size_t>(), gview);
    }
    else
    {
      return (gview.level()+1)*m_offset;
    }
  }
};

const size_t compute_level_offset::default_offset =
    std::numeric_limits<size_t>::max()/STAPL_MAX_HIERARCHY_LEVEL;


//////////////////////////////////////////////////////////////////////
/// @brief Add a super vertex to a hierarchical graph.
//////////////////////////////////////////////////////////////////////
class add_svertex_wf
{
private:
  size_t m_offset;

public:
  typedef domset1D<size_t> result_type;

  add_svertex_wf(size_t const& offset)
    : m_offset(offset)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a super vertex to a hierarchical graph.
  /// @param dom Domain of super-vertex's children ids
  /// @param vd Vertex descriptor of the new super-vertex
  /// @param gv View over the hierarchical graph
  /// @return singleton domain with the super-vertex id
  //////////////////////////////////////////////////////////////////////
  template<typename Domain, typename Descriptor, typename GraphView>
  result_type operator()(Domain const dom, Descriptor vd, GraphView gv) const
  {
    typename GraphView::vertex_descriptor vid = m_offset + vd;

    gv.add_vertex(vid, typename GraphView::vertex_property());

    gv[vid].set_child(dom);

    // return range of 1 vertex descriptor for appending
    // to final domain of view for this level
    return result_type(vid, vid);
  }

  void define_type(typer& t)
  {
    t.member(m_offset);
  }
}; // class add_svertex_wf


//////////////////////////////////////////////////////////////////////
/// @brief Create a new hierarchy level over a given view.
/// @param graph the hierarchy level will be created on top of this view.
/// @param partitioner functor partitioning the view graph.
/// @param ef functor creating the edges in the new hierarchy level.
/// @param calculate_offset functor determining the vertex descriptor
///   offset between 2 level of hierarchy.
/// @return new level of hierarchy view.
///
/// Using the name "create_hierarchy" to avoid a conflict with the
/// "create_level" functions of the other hierarchical view implementation.
/////////////////////////////////////////////////////////////////////
template <typename GraphVw, typename VertPart, typename EdgeFunc>
GraphVw
create_hierarchy(const GraphVw& graph,
                 const VertPart& partitioner,
                 const EdgeFunc& ef,
                 compute_level_offset const& calculate_offset
                  = compute_level_offset())
{
  typedef GraphVw                                          view_type;
  typedef domset1D<typename view_type::vertex_descriptor>  domain_type;
  typedef typename VertPart::view_type                     SubdomainView;
  typedef typename VertPart::descriptor_view_type          DescriptorView;

  const size_t new_level = graph.level() + 1;

  //compute the offset for the vertex descriptors of this level
  const size_t offset = calculate_offset(graph);

  // call user's vertex-partitioner on graph of last-level:
  std::pair<SubdomainView, DescriptorView> pv = partitioner(graph, new_level);

  //Create graph view over the all container
  view_type fullview(graph.container(), true);

  // populate it with vertices
  domain_type dom = map_reduce(
    add_svertex_wf(offset), plus<domain_type>(),
    pv.first, pv.second, make_repeat_view(fullview)
  );

  // create a view for that level
  view_type new_level_view(graph.container(), dom, graph.mapfunc(), new_level);

  // add edges
  ef(new_level_view, new_level);

  return new_level_view;
}

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_VIEW_HGRAPH_VIEW_HPP
