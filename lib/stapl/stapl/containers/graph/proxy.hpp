/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PROXY_HPP
#define STAPL_CONTAINERS_GRAPH_PROXY_HPP

#include <stapl/containers/graph/graph_fwd.hpp>
#include <stapl/utility/loc_qual.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for static @ref stapl::graph.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M,
          typename ...OptionalParams, typename Accessor>
class proxy<graph<D, M, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef graph<D, M, OptionalParams...>               target_t;
  typedef typename target_t::iterator                  iter_t;

public:
  typedef typename target_t::value_type                value_type;
  typedef typename target_t::partition_type            partition_type;
  typedef typename target_t::mapper_type               mapper_type;
  typedef typename target_t::domain_type               domain_type;
  typedef typename target_t::distribution_type         distribution_type;
  typedef typename target_t::map_dom_t                 map_dom_t;
  typedef typename target_t::index_type                index_type;
  typedef typename target_t::gid_type                  gid_type;
  typedef typename target_t::size_type                 size_type;

  typedef typename target_t::reference                 reference;
  typedef typename target_t::const_reference           const_reference;
  typedef typename target_t::accessor_type             accessor_type;
  typedef typename target_t::iterator                  iterator;
  typedef typename target_t::loc_dist_metadata         loc_dist_metadata;

  typedef typename target_t::vertex_property           vertex_property;
  typedef typename target_t::edge_property             edge_property;
  typedef typename target_t::vertex_descriptor         vertex_descriptor;
  typedef typename target_t::simple_vertex_descriptor  simple_vertex_descriptor;
  typedef typename target_t::edge_descriptor           edge_descriptor;
  typedef typename target_t::directness_type           directness_type;
  typedef typename target_t::multiplicity_type         multiplicity_type;

  typedef typename target_t::vertex_iterator           vertex_iterator;
  typedef typename target_t::const_vertex_iterator     const_vertex_iterator;
  typedef typename target_t::adj_edge_iterator         adj_edge_iterator;
  typedef typename target_t::const_adj_edge_iterator   const_adj_edge_iterator;
  typedef typename target_t::edge_iterator             edge_iterator;
  typedef typename target_t::vertex_reference          vertex_reference;


public:

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  {
    return Accessor::read();
  }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  size_t version(void) const
  {
    return Accessor::const_invoke(&target_t::version);
  }

  size_type size() const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  reference operator[](size_type gid)
  {
    return Accessor::invoke(&target_t::operator[], gid);
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  distribution_type* get_distribution(void)
  {
    return Accessor::invoke(&target_t::get_distribution);
  }

  distribution_type const* get_distribution(void) const
  {
    return Accessor::const_invoke(&target_t::get_distribution);
  }

  distribution_type& distribution(void)
  {
    typedef iterator (target_t::* fn_t) (gid_type const&);
    return *Accessor::invoke((fn_t) &target_t::get_distribution);
  }

  iterator make_iterator(domain_type const& domain, gid_type const& gid)
  {
    typedef iterator (target_t::* fn_t) (domain_type const&, gid_type const&);
    constexpr fn_t fn = &target_t::template make_iterator<domain_type>;
    return Accessor::invoke(fn, domain, gid);
  }

  iterator make_iterator(gid_type const& gid)
  {
    typedef iterator (target_t::* fn_t) (gid_type const&);
    constexpr fn_t fn = &target_t::template make_iterator;
    return Accessor::invoke(fn, gid);
  }

  void set_element(gid_type gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  bool is_directed(void) const
  {
    return Accessor::const_invoke(&target_t::is_directed);
  }

  vertex_iterator begin(void)
  {
    return Accessor::invoke(&target_t::begin);
  }

  const_vertex_iterator begin(void) const
  {
    return Accessor::const_invoke(&target_t::begin);
  }

  vertex_iterator end(void)
  {
    return Accessor::invoke(&target_t::end);
  }

  const_vertex_iterator end(void) const
  {
    return Accessor::const_invoke(&target_t::end);
  }

  edge_iterator edges_begin(void)
  {
    return Accessor::invoke(&target_t::edges_begin);
  }

  edge_iterator edges_end(void)
  {
    return Accessor::invoke(&target_t::edges_end);
  }

  vertex_descriptor add_vertex(void)
  {
    typedef vertex_descriptor (target_t::* fn_t) (void);
    return Accessor::invoke( (fn_t) &target_t::add_vertex);
  }

  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    typedef vertex_descriptor (target_t::* fn_t) (vertex_property const&);
    return Accessor::invoke( (fn_t) &target_t::add_vertex, vp);
  }

  vertex_descriptor add_vertex(vertex_descriptor const& gid,
                               vertex_property const& vp)
  {
    typedef vertex_descriptor (target_t::* fn_t)
      (vertex_descriptor const&, vertex_property const&);
    return Accessor::invoke( (fn_t) &target_t::add_vertex, gid, vp);
  }

  bool delete_vertex(vertex_descriptor const& gid)
  {
    return Accessor::invoke(&target_t::delete_vertex, gid);
  }

  vertex_iterator find_vertex(vertex_descriptor const& gid)
  {
    return Accessor::invoke(&target_t::find_vertex, gid);
  }

  edge_descriptor add_edge(edge_descriptor const& ed,
                           edge_property const& ep = edge_property())
  {
    return Accessor::invoke(&target_t::add_edge, ed, ep);
  }

  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& ep = edge_property())
  {
    return Accessor::invoke(&target_t::add_edge, source, target, ep);
  }

  edge_descriptor insert_edge(edge_descriptor const& ed,
                           edge_property const& ep = edge_property())
  {
    return Accessor::invoke(&target_t::insert_edge, ed, ep);
  }

  edge_descriptor insert_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& ep = edge_property())
  {
    return Accessor::invoke(&target_t::insert_edge, source, target, ep);
  }

  void add_edge_async(edge_descriptor const& ed,
                      edge_property const& ep = edge_property())
  {
    Accessor::invoke(&target_t::add_edge_async, ed, ep);
  }

  void add_edge_async(vertex_descriptor const& source,
                      vertex_descriptor const& target,
                      edge_property const& ep = edge_property())
  {
    Accessor::invoke(&target_t::add_edge_async, source, target, ep);
  }

  void insert_edge_async(edge_descriptor const& ed,
                      edge_property const& ep = edge_property())
  {
    Accessor::invoke(&target_t::insert_edge_async, ed, ep);
  }

  void insert_edge_async(vertex_descriptor const& source,
                      vertex_descriptor const& target,
                      edge_property const& ep = edge_property())
  {
    Accessor::invoke(&target_t::insert_edge_async, source, target, ep);
  }

  void delete_edge(edge_descriptor const& ed)
  {
    Accessor::invoke(&target_t::delete_edge, ed);
  }

  void delete_edge(vertex_descriptor const& source,
                   vertex_descriptor const& target)
  {
    Accessor::invoke(&target_t::delete_edge, source, target);
  }

  void sort_edges(void) const
  {
    typedef void (target_t::* mem_fun_t)(void);
    mem_fun_t mem_fun = &target_t::sort_edges;
    Accessor::invoke(mem_fun);
  }

  size_t num_vertices(void) const
  {
    return Accessor::const_invoke(&target_t::num_vertices);
  }

  void clear(void)
  {
    Accessor::invoke(&target_t::clear);
  }

  size_t num_edges(void) const
  {
    return Accessor::const_invoke(&target_t::num_edges);
  }

  size_t num_local_edges(void) const
  {
    return Accessor::const_invoke(&target_t::num_local_edges);
  }

  locality_info locality(gid_type gid)
  {
    return Accessor::invoke(&target_t::locality, gid);
  }

  template <typename F>
  void apply_set(vertex_descriptor const& gid, F const& f)
  {
    typedef void (target_t::* fn_t) (vertex_descriptor const&, F const&);
    Accessor::invoke( (fn_t)&target_t::apply_set, gid, f);
  }

  template <typename F>
  void vp_apply_async(vertex_descriptor const& gid, F const& f)
  {
    Accessor::invoke(&target_t::vp_apply_async, gid, f);
  }

  template <typename F>
  typename F::result_type vp_apply(vertex_descriptor const& gid, F const& f)
  {
    return Accessor::invoke(&target_t::vp_apply, gid, f);
  }

  template <typename F>
  void ep_apply_async(edge_descriptor const& ed, F const& f)
  {
    Accessor::invoke(&target_t::ep_apply_async, ed, f);
  }

  template <typename F>
  void aggregate_vp_apply_async(size_t const& loc, F const& f)
  {
    Accessor::invoke(&target_t::aggregate_vp_apply_async, loc, f);
  }

  template <typename Cont, typename F>
  void aggregate_apply_async(size_t const& loc, Cont const& v, F const& f)
  {
    typedef void (target_t::* fn_t) (size_t const&, Cont const&, F const&);
    Accessor::invoke((fn_t) &target_t::aggregate_apply_async, loc, v, f);
  }

  template <typename Cont, typename F>
  void aggregate_async(size_t const& loc, Cont const& v, F const& f)
  {
    typedef void (target_t::* fn_t) (size_t const&, Cont const&, F const&);
    Accessor::invoke((fn_t) &target_t::aggregate_async, loc, v, f);
  }

  reference make_reference(vertex_descriptor const& gid)
  {
    return Accessor::invoke(&target_t::make_reference, gid);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class is a (specialization of) @ref proxy for
/// @ref dynamic_graph.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M,
          typename ...OptionalParams, typename Accessor>
class proxy<dynamic_graph<D, M, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef dynamic_graph<D, M, OptionalParams...>       target_t;
  typedef typename target_t::iterator                  iter_t;

public:
  typedef typename target_t::value_type                value_type;
  typedef typename target_t::partition_type            partition_type;
  typedef typename target_t::mapper_type               mapper_type;
  typedef typename target_t::domain_type               domain_type;
  typedef typename target_t::distribution_type         distribution_type;
  typedef typename target_t::map_dom_t                 map_dom_t;
  typedef typename target_t::index_type                index_type;
  typedef typename target_t::gid_type                  gid_type;
  typedef typename target_t::size_type                 size_type;

  typedef typename target_t::reference                 reference;
  typedef typename target_t::const_reference           const_reference;
  typedef typename target_t::accessor_type             accessor_type;
  typedef typename target_t::iterator                  iterator;
  typedef typename target_t::loc_dist_metadata         loc_dist_metadata;

  typedef typename target_t::vertex_property           vertex_property;
  typedef typename target_t::edge_property             edge_property;
  typedef typename target_t::vertex_descriptor         vertex_descriptor;
  typedef typename target_t::simple_vertex_descriptor  simple_vertex_descriptor;
  typedef typename target_t::edge_descriptor           edge_descriptor;
  typedef typename target_t::directness_type           directness_type;
  typedef typename target_t::multiplicity_type         multiplicity_type;

  typedef typename target_t::vertex_iterator           vertex_iterator;
  typedef typename target_t::const_vertex_iterator     const_vertex_iterator;
  typedef typename target_t::adj_edge_iterator         adj_edge_iterator;
  typedef typename target_t::const_adj_edge_iterator   const_adj_edge_iterator;
  typedef typename target_t::edge_iterator             edge_iterator;
  typedef typename target_t::vertex_reference          vertex_reference;


public:
  //typedef proxy fast_view_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  {
    return Accessor::read();
  }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  size_type size(void) const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  reference operator[](size_type gid)
  {
    return Accessor::invoke(&target_t::operator[], gid);
  }

  size_t version(void) const
  {
    return Accessor::const_invoke(&target_t::version);
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  distribution_type* get_distribution(void)
  {
    return Accessor::invoke(&target_t::get_distribution);
  }

  distribution_type const* get_distribution(void) const
  {
    return Accessor::const_invoke(&target_t::get_distribution);
  }

  distribution_type& distribution(void)
  {
    return *Accessor::invoke(&target_t::get_distribution);
  }

  iterator make_iterator(gid_type const& gid)
  {
    return Accessor::invoke(&target_t::make_iterator, gid);
  }

  iterator make_iterator(domain_type const& domain, gid_type const& gid)
  {
    typedef iterator (target_t::* fn_t) (domain_type const&, gid_type const&);
    constexpr fn_t fn = &target_t::template make_iterator<domain_type>;
    return Accessor::invoke(fn, domain, gid);
  }

  void set_element(gid_type gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  bool is_directed(void) const
  {
    return Accessor::const_invoke(&target_t::is_directed);
  }

  vertex_iterator begin(void)
  {
    return Accessor::invoke(&target_t::begin);
  }

  const_vertex_iterator begin(void) const
  {
    return Accessor::const_invoke(&target_t::begin);
  }

  vertex_iterator end(void)
  {
    return Accessor::invoke(&target_t::end);
  }

  const_vertex_iterator end(void) const
  {
    return Accessor::const_invoke(&target_t::end);
  }

  edge_iterator edges_begin(void)
  {
    return Accessor::invoke(&target_t::edges_begin);
  }

  edge_iterator edges_end(void)
  {
    return Accessor::invoke(&target_t::edges_end);
  }

  vertex_descriptor add_vertex(void)
  {
    typedef vertex_descriptor (target_t::* fn_t)(void);
    return Accessor::invoke((fn_t)&target_t::add_vertex);
  }

  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    typedef vertex_descriptor (target_t::* fn_t)(vertex_property const&);
    return Accessor::invoke((fn_t)&target_t::add_vertex, vp);
  }

  vertex_descriptor add_vertex(vertex_descriptor const& gid,
                               vertex_property const& vp)
  {
    typedef vertex_descriptor (target_t::* fn_t)(vertex_descriptor const&,
                                                 vertex_property const&);
    return Accessor::invoke((fn_t)&target_t::add_vertex, gid, vp);
  }

  template<typename Functor>
  void add_vertex(vertex_descriptor const& gid,
                  vertex_property const& vp,
                  Functor const& func)
  {
    Accessor::invoke(&target_t::template add_vertex<Functor>, gid, vp, func);
  }

  bool delete_vertex(vertex_descriptor const& gid)
  {
    return Accessor::invoke(&target_t::delete_vertex, gid);
  }

  vertex_iterator find_vertex(vertex_descriptor const& gid)
  {
    return Accessor::invoke(&target_t::find_vertex, gid);
  }

  edge_descriptor add_edge(edge_descriptor const& ed,
                           edge_property const& ep = edge_property())
  {
    return Accessor::invoke(&target_t::add_edge, ed, ep);
  }

  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& ep = edge_property())
  {
    return Accessor::invoke(&target_t::add_edge, source, target, ep);
  }

  void add_edge_async(edge_descriptor const& ed,
                      edge_property const& ep = edge_property())
  {
    Accessor::invoke(&target_t::add_edge_async, ed, ep);
  }

  void add_edge_async(vertex_descriptor const& source,
                      vertex_descriptor const& target,
                      edge_property const& ep = edge_property())
  {
    void (target_t::*pmf)(vertex_descriptor const&,
                          vertex_descriptor const&,
                          edge_property const&) = &target_t::add_edge_async;
    Accessor::invoke(pmf, source, target, ep);
  }

  void delete_edge(edge_descriptor const& ed)
  {
    Accessor::invoke(&target_t::delete_edge, ed);
  }

  void delete_edge(vertex_descriptor const& source,
                   vertex_descriptor const& target)
  {
    Accessor::invoke(&target_t::delete_edge, source, target);
  }

  void sort_edges(void) const
  {
    typedef void (target_t::* mem_fun_t)(void);
    mem_fun_t mem_fun = &target_t::sort_edges;
    Accessor::invoke(mem_fun);
  }

  size_t num_vertices(void) const
  {
    return Accessor::const_invoke(&target_t::num_vertices);
  }

  void clear(void)
  {
    Accessor::invoke(&target_t::clear);
  }

  size_t num_edges(void) const
  {
    return Accessor::const_invoke(&target_t::num_edges);
  }

  size_t num_local_edges(void) const
  {
    return Accessor::const_invoke(&target_t::num_local_edges);
  }

  locality_info locality(gid_type gid)
  {
    return Accessor::invoke(&target_t::locality, gid);
  }

  template <typename F>
  void apply_set(vertex_descriptor const& gid, F const& f)
  {
    typedef void (target_t::* fn_t) (vertex_descriptor const&, F const&);
    Accessor::invoke((fn_t) &target_t::apply_set, gid, f);
  }

  template <typename F>
  void vp_apply_async(vertex_descriptor const& gid, F const& f)
  {
    using fn_t = void (target_t::*)(vertex_descriptor const&, F const&);
    Accessor::invoke((fn_t) &target_t::vp_apply_async, gid, f);
  }

  template <typename F>
  typename F::result_type vp_apply(vertex_descriptor const& gid, F const& f)
  {
    using result_type = typename F::result_type;
    using fn_t = result_type (target_t::*)(vertex_descriptor const&, F const&);
    return Accessor::invoke((fn_t) &target_t::vp_apply, gid, f);
  }

  template <typename F>
  void ep_apply_async(edge_descriptor const& ed, F const& f)
  {
    Accessor::invoke(&target_t::ep_apply_async, ed, f);
  }

  template <typename F>
  void aggregate_vp_apply_async(size_t const& loc, F const& f)
  {
    Accessor::invoke(&target_t::aggregate_vp_apply_async, loc, f);
  }

  template <typename Cont, typename F>
  void aggregate_apply_async(size_t const& loc, Cont const& v, F const& f)
  {
    typedef void (target_t::* fn_t) (size_t const&, Cont const&, F const&);
    Accessor::invoke((fn_t) &target_t::aggregate_apply_async, loc, v, f);
  }

  template <typename Cont, typename F>
  void aggregate_async(size_t const& loc, Cont const& v, F const& f)
  {
    typedef void (target_t::* fn_t) (size_t const&, Cont const&, F const&);
    Accessor::invoke((fn_t) &target_t::aggregate_async, loc, v, f);
  }

  reference make_reference(vertex_descriptor const& gid)
  {
    return Accessor::invoke(&target_t::make_reference, gid);
  }

  value_type get_element(gid_type const& gid)
  {
    return Accessor::const_invoke(&target_t::get_element, gid);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
}; // class proxy<dynamic_graph<...>, Accessor>

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_PROXY_HPP
