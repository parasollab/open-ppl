/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_LAZY_GRAPH_VIEW_HPP
#define STAPL_VIEWS_LAZY_GRAPH_VIEW_HPP

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/domains/infinite.hpp>


namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Container for an ordered log of commits, which can be flushed later.
/// Used by the @ref lazy_graph_view_container to store log of deferred commits,
/// as well as apply them to the appropriate pGraph at a suitable time.
/// @tparam PG The type of pGraph to which the deferred commits will be applied.
///
/// Stores pending add/delete/migrate requests to the pGraph in separate
/// buffers, as well as a buffer that stores ordering information for each
/// pending request. On each location, the pending requests are flushed in the
/// order in which they were added.
//////////////////////////////////////////////////////////////////////
template <typename PG>
class graph_commit_log
{
public:
  typedef typename PG::vertex_property              vertex_property;
  typedef typename PG::vertex_descriptor            vertex_descriptor;

private:
  std::vector<vertex_property>                                m_add_prop_buffer;
  std::vector<vertex_descriptor>                              m_delete_buffer;
  std::vector<std::pair<vertex_descriptor, size_t> >          m_migrate_buffer;
  std::vector<std::pair<vertex_descriptor, vertex_property> > m_add_buffer;
  std::vector<size_t>                                         m_commit_order;

  PG* m_container;

public:

  graph_commit_log(graph_commit_log const& other)
    : m_add_prop_buffer(other.m_add_prop_buffer),
      m_delete_buffer(other.m_delete_buffer),
      m_migrate_buffer(other.m_migrate_buffer),
      m_add_buffer(other.m_add_buffer),
      m_container(other.m_container)
  { }

  graph_commit_log(PG* cont)
    : m_container(cont)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destructor also flushes the pending commits to the pGraph.
  //////////////////////////////////////////////////////////////////////
  ~graph_commit_log(void)
  {
    this->flush();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_property const&)
  /// Stores the request for future commit.
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_property const& vp)
  {
    m_add_prop_buffer.push_back(vp);
    m_commit_order.push_back(0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_descriptor const&, vertex_property const&)
  /// Stores the request for future commit.
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_descriptor const& vd, vertex_property const& vp)
  {
    m_add_buffer.push_back(std::make_pair(vd, vp));
    m_commit_order.push_back(1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::delete_vertex(vertex_descriptor const&)
  /// Stores the request for future commit.
  //////////////////////////////////////////////////////////////////////
  void delete_vertex(vertex_descriptor const& vd)
  {
    m_delete_buffer.push_back(vd);
    m_commit_order.push_back(2);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stores a request for migration of given vertex for future commit.
  /// @param vd The descriptor of the vertex to be migrated.
  /// @param location The location where the vertex needs to be migrated.
  //////////////////////////////////////////////////////////////////////
  void migrate(vertex_descriptor const& vd, size_t location)
  {
    m_migrate_buffer.push_back(std::make_pair(vd, location));
    m_commit_order.push_back(3);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes out all pending requests to the pGraph.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    size_t j=0, k=0, l=0, m=0;
    for (size_t i=0; i<m_commit_order.size(); ++i) {
      switch(m_commit_order[i])
      {
        case 0:
          this->m_container->add_vertex(m_add_prop_buffer[j]);
          ++j;
          break;
        case 1:
          this->m_container->add_vertex(m_add_buffer[k].first,
                                        m_add_buffer[k].second);
          ++k;
          break;
        case 2:
          this->m_container->delete_vertex(m_delete_buffer[l]);
          ++l;
          break;
        case 3:
          this->m_container->migrate(m_migrate_buffer[m].first,
                                     m_migrate_buffer[m].second);
          ++m;
          break;
      }
    }

    // clear out the buffers:
    m_add_prop_buffer.clear();
    m_add_buffer.clear();
    m_delete_buffer.clear();
    m_migrate_buffer.clear();
    // and the log:
    m_commit_order.clear();
  }

  void define_type(typer& t)
  {
    t.member(m_add_prop_buffer);
    t.member(m_delete_buffer);
    t.member(m_migrate_buffer);
    t.member(m_add_buffer);
    t.member(m_container);
    t.member(m_commit_order);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Container for deferring pGraph operations which can be flushed later.
/// Used by the @ref lazy_graph_view to defer commits,
/// as well as apply them to the appropriate pGraph at a suitable time.
/// @tparam PG The type of pGraph to which the deferred commits will be applied.
//////////////////////////////////////////////////////////////////////
template <typename PG>
class lazy_graph_view_container
{
public:
  typedef PG                                        container_type;
  typedef typename PG::value_type                   value_type;
  typedef infinite                                  domain_type;
  typedef domain_type::index_type                   index_type;
  typedef index_type                                gid_type;
  typedef value_type                                reference;
  typedef const value_type                          const_reference;

  typedef typename PG::vertex_property              vertex_property;
  typedef typename PG::vertex_descriptor            vertex_descriptor;

private:
  graph_commit_log<PG> m_commit_log;

public:

  void define_type(typer& t)
  {
    t.member(m_commit_log);
  }

  lazy_graph_view_container(lazy_graph_view_container* other)
    : m_commit_log(other->m_commit_log)
  { }

  lazy_graph_view_container(lazy_graph_view_container const& other)
    : m_commit_log(other.m_commit_log)
  { }

  lazy_graph_view_container(container_type* cont)
    : m_commit_log(cont)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::flush()
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    this->m_commit_log.flush();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::add_vertex(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_vertex(void)
  {
    m_commit_log.add_vertex(vertex_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::add_vertex(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_property const& vp)
  {
    m_commit_log.add_vertex(vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::add_vertex(vertex_descriptor const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_descriptor const& vd, vertex_property const& vp)
  {
    m_commit_log.add_vertex(vd, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::delete_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void delete_vertex(vertex_descriptor const& vd)
  {
    m_commit_log.delete_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::migrate(vertex_descriptor const&, size_t)
  //////////////////////////////////////////////////////////////////////
  void migrate(vertex_descriptor const& vd, size_t location)
  {
    m_commit_log.migrate(vd, location);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the version of the container.
  /// @return Version number of 0
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the container.
  ///
  /// The size of this container is infinity.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return domain_type().size();
  }

  domain_type domain(void) const
  {
    return domain_type();
  }

}; //class lazy_graph_view_container



//////////////////////////////////////////////////////////////////////
/// @brief pGraph View for deferring pGraph operations which can be flushed
/// later.
///
/// Used when modifications to the graph need to be made while an algorithm
/// is actively executing on said graph. Deferring the modifications keeps
/// the graph in a consistent state for all operations being applied.
/// Provides a subset of the interface of a @ref graph_view.
/// @tparam PG The type of pGraph to which the deferred commits will be applied.
//////////////////////////////////////////////////////////////////////
template <typename PG>
class lazy_graph_view
  : public core_view<PG, indexed_domain<size_t>, f_ident<size_t> >
{
  typedef core_view<PG, indexed_domain<size_t>, f_ident<size_t> > base_type;

  bool m_commit_flag;

 public:
  STAPL_VIEW_REFLECT_TRAITS(lazy_graph_view)

  typedef typename PG::vertex_property         vertex_property;
  typedef typename PG::vertex_descriptor       vertex_descriptor;

  lazy_graph_view(void)
    : m_commit_flag(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view takes ownership of the container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  lazy_graph_view(view_container_type* vcont)
    : base_type(vcont, domain_type(vcont->size()), f_ident<size_t>()),
      m_commit_flag(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view does not take ownership of the
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  lazy_graph_view(view_container_type const& vcont)
    : base_type(vcont, domain_type(vcont->size()), f_ident<size_t>()),
      m_commit_flag(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view takes ownership of the container.
  ///
  /// This constructor is required by the view_packing implementation of
  /// repeat_view over the lazy_graph_view.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom   domain of the view, should be equal to the domain of the
  ///              container.
  /// @param mf    mapping function of the view.
  //////////////////////////////////////////////////////////////////////
  lazy_graph_view(view_container_type const& vcont,
                  indexed_domain<size_t> const& dom,
                  f_ident<size_t> const& mf)
    : base_type(vcont, dom, mf), m_commit_flag(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view does not take ownership of the
  ///        container.
  ///
  /// This constructor is required by the view_packing implementation of
  /// repeat_view over the lazy_graph_view.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom   domain of the view, should be equal to the domain of the
  ///              container.
  /// @param mf    mapping function of the view.
  //////////////////////////////////////////////////////////////////////
  lazy_graph_view(view_container_type* vcont, indexed_domain<size_t> const& dom,
                  f_ident<size_t> const& mf)
    : base_type(vcont, dom, mf), m_commit_flag(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a copy of the passed lazy_graph_view.
  //////////////////////////////////////////////////////////////////////
  lazy_graph_view(lazy_graph_view const& other)
    : base_type(other.container(), other.domain(), f_ident<size_t>()),
      m_commit_flag(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::add_vertex(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_vertex(void)
  {
    this->container().add_vertex();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::add_vertex(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_property const& vp)
  {
    this->container().add_vertex(vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::add_vertex(vertex_descriptor const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  void add_vertex(vertex_descriptor const& vd, vertex_property const& vp)
  {
    this->container().add_vertex(vd, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::delete_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void delete_vertex(vertex_descriptor const& vd)
  {
    this->container().delete_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::migrate(vertex_descriptor const&, size_t)
  //////////////////////////////////////////////////////////////////////
  void migrate(vertex_descriptor const& vd, size_t location)
  {
    this->container().migrate_vertex(vd, location);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_commit_log::flush()
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    if (m_commit_flag)
      this->container().flush();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called after an algorithm is done, to flush all pending requests.
  /// @todo Finalize the behavior of this method when gForge todo 807 is
  /// completed.
  //////////////////////////////////////////////////////////////////////
  void post_execute(void)
  {
    this->flush();
  }

  size_t size(void) const
  {
    return get_num_locations();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_commit_flag);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @view_traits for the @ref lazy_graph_view.
/// @tparam C The type of pGraph to which the deferred commits will be applied.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct view_traits<lazy_graph_view<C> >
{
  typedef lazy_graph_view<C>                 value_type;
  typedef value_type                         reference;
  typedef const value_type                   const_reference;
  typedef C                                  container;
  typedef size_t                             index_type;
  typedef f_ident<index_type>                map_function;
  typedef indexed_domain<index_type>         domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to create a @ref lazy_graph_view over the
/// given pGraph pContainer.
/// @tparam C The type of pGraph to which the deferred commits will be applied.
/// @param c The pGraph to which the deferred commits will be applied.
/// @return A @ref lazy_graph_view over the given pGraph pContainer.
//////////////////////////////////////////////////////////////////////
template<typename C>
lazy_graph_view<lazy_graph_view_container<C> >
lazy_graph(C& c)
{
  typedef lazy_graph_view_container<C> container_t;
  return lazy_graph_view<container_t>(new container_t(&c));
}


} // stapl namespace

#endif /* STAPL_VIEWS_LAZY_GRAPH_VIEW_HPP */
