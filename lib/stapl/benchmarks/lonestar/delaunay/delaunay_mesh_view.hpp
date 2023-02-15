/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_MESH_VIEW_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_MESH_VIEW_HPP


#include <stapl/runtime.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/algorithms/sorting.hpp>

#include <boost/serialization/deque.hpp>

#include <benchmarks/lonestar/delaunay/delaunay_point.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_utilities.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_printer.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_debug_info.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_edge.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_vertex.hpp>

#include <benchmarks/lonestar/delaunay/delaunay_output.hpp>


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief A graph_view wrapper for simplifying delaunay's edge insertion logic
/// and making asynchronous remote function calls.
///////////////////////////////////////////////////////////////////////////////
template <typename PG,
          typename Dom     = typename container_traits<PG>::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>,
          typename Derived = use_default>
class delaunay_mesh_view
  : public graph_view<PG, Dom, MapFunc, Derived>
{
public:
  typedef core_view<PG,Dom,MapFunc>                         base_base_type;
  typedef graph_view<PG, Dom, MapFunc, Derived>             base_type;
  typedef delaunay_mesh_view<PG, Dom, MapFunc, Derived>     this_type;
  typedef typename select_derived<Derived, this_type>::type derived_type;
  typedef view_operations::sequence<derived_type>           sequence_op_type;

  STAPL_VIEW_REFLECT_TRAITS(delaunay_mesh_view)

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
  typedef std::pair<delaunay_edge, delaunay_edge> edge_pair;

  delaunay_mesh_view(void)                                 = default;
  delaunay_mesh_view(delaunay_mesh_view const&)            = default;
  delaunay_mesh_view(delaunay_mesh_view&&)                 = default;
  delaunay_mesh_view& operator=(delaunay_mesh_view const&) = default;

  delaunay_mesh_view(view_container_type* vcont, domain_type const& dom,
             map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  template <typename OV>
  delaunay_mesh_view(view_container_type* vcont, domain_type const& dom,
             map_func_type mfunc, OV const&)
    : base_type(vcont, dom, mfunc)
  { }

  delaunay_mesh_view(view_container_type const& vcont, domain_type const& dom,
             map_func_type mfunc=MapFunc(),
             delaunay_mesh_view const& other=delaunay_mesh_view())
    : base_type(vcont, dom, mfunc)
  { }

  template <typename OV>
  delaunay_mesh_view(view_container_type const& vcont, domain_type const& dom,
             map_func_type mfunc, OV const&)
    : base_type(vcont, dom, mfunc)
  { }

  delaunay_mesh_view(view_container_type* vcont)
    : base_type(vcont)
  { }

  delaunay_mesh_view(view_container_type const* vcont)
    : base_type(vcont)
  { }

  delaunay_mesh_view(view_container_type& vcont)
    : base_type(vcont)
  { }

  delaunay_mesh_view(view_container_type const& vcont)
    : base_type(vcont, vcont.domain())
  { }

  template<typename Derive>
  delaunay_mesh_view(delaunay_mesh_view<PG, Dom, MapFunc, Derive> const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  template <typename T1, typename T2>
  delaunay_mesh_view(delaunay_mesh_view<PG, iterator_domain<T1, T2>, MapFunc,
                                        Derived> const& other)
    : base_type(other.container(), domain_type(), other.mapfunc())
  { }

  delaunay_mesh_view(size_t num_vertices)
    : delaunay_mesh_view(new view_container_type(num_vertices))
  { }

  void sort_vertex_edges(size_t vd)
  {
    sort_vertex_edges_wf wf;
    this->apply_set(vd, wf);
  }

  template <typename Vertex>
  void sort_vertex_edges(Vertex&& v)
  {
    compare_edge_wf wf;
    v.sort_edges(wf);
  }

  void add_delaunay_edge(edge_descriptor const& ed,
                         edge_property const& ep)
  {
    if (vertex_location(ed.source()) == this->get_location_id()) {
      this->insert_edge(ed, ep, compare_edge_wf());
    } else {
      this->insert_edge_async(ed, ep, compare_edge_wf());
    }
  }

  void add_delaunay_edge(vertex_descriptor const& source,
                         vertex_descriptor const& target,
                         edge_property const& ep)
  {
    if (vertex_location(source) == this->get_location_id()) {
      this->insert_edge(source, target, ep, compare_edge_wf());
    } else {
      this->insert_edge_async(source, target, ep, compare_edge_wf());
    }
  }

  edge_pair add_delaunay_edge(vertex_descriptor const& source,
                              vertex_descriptor const& target,
                              point2d const& p1, point2d const& p2)
  {
    delaunay_edge ep1(source, target, p1, p2);
    delaunay_edge ep2(target, source, p2, p1);

    auto loc = this->get_location_id();
    compare_edge_wf wf;

    if (vertex_location(source) == loc) {
      if (vertex_location(target) == loc) {
        this->insert_edge(source, target, ep1, wf);
        this->insert_edge(target, source, ep2, wf);
      } else {
        this->insert_edge_async(target, source, ep2, wf);
        this->insert_edge(source, target, ep1, wf);
      }
    } else if (vertex_location(target) == loc) {
      this->insert_edge_async(source, target, ep1, wf);
      this->insert_edge(target, source, ep2, wf);
    } else {
      this->insert_edge_async(source, target, ep1, wf);
      this->insert_edge_async(target, source, ep2, wf);
    }

    return edge_pair(ep1, ep2);
  }

  edge_pair add_delaunay_edge(vertex_descriptor const& source,
                              vertex_descriptor const& target)
  {
    return add_delaunay_edge(source, target,
              (*this->find_vertex(source)).property().get_point(),
              (*this->find_vertex(target)).property().get_point());
  }


  edge_pair add_delaunay_edge(vertex_descriptor const& s1,
                              vertex_descriptor const& s2,
                              vertex_descriptor const& s3)
  {
    auto const& p1 = (*this->find_vertex(s1)).property().get_point();
    auto const& p2 = (*this->find_vertex(s2)).property().get_point();
    auto const& p3 = (*this->find_vertex(s3)).property().get_point();

    edge_pair epair_a =  add_delaunay_edge(s1, s2, p1, p2);
    edge_pair epair_b =  add_delaunay_edge(s2, s3, p2, p3);

    auto value = point_test(p1, p2, p3);

    if (value < 0) {
      return add_delaunay_edge(s1, s3, p1, p3);
    } else if (value > 0) {
      add_delaunay_edge(s1, s3, p1, p3);
      return edge_pair(epair_a.first, epair_b.second);
    } else {
      return edge_pair(epair_a.first, epair_b.second);
    }
  }


  delaunay_edge connect(delaunay_edge const& a, delaunay_edge const & b)
  {
    return add_delaunay_edge(a.target(),   b.source(),
                             a.get_Dest(), b.get_Org()).first;
  }

  void delete_delaunay_edge(delaunay_edge const& edge)
  {
    auto ed = edge.get_ed();
    this->delete_edge(ed);
    this->delete_edge(reverse(ed));
  }

  location_type vertex_location(vertex_descriptor vd) const
  {
    return vertex_location(*this, vd);
  }

  template <typename View2>
  auto vertex_location(View2& view, vertex_descriptor vd) const ->
        decltype(view.container().locality(vd).location(), vertex_descriptor())
  {
    return view.locality(vd).location();
  }

  template <typename View2>
  location_type vertex_location(View2& view, ...) const
  {
    return this->get_location_id();
  }

  template <typename WF>
  stapl::future<typename WF::result_type> async_command(location_type loc,
                                                        WF& wf)
  {
    if (loc == this->get_location_id()) {
      return make_ready_future(wf());
    } else {
      return stapl::opaque_rmi(loc, this->get_rmi_handle(),
        &this_type::template exec_async_command<WF>, wf);
    }
  }

  template <typename WF>
  typename WF::result_type exec_async_command(WF& wf)
  {
    return WF(this, wf)();
  }

  template <typename WF>
  typename WF::result_type sync_command(location_type loc, WF& wf)
  {
    return async_command(loc, wf).get();
  }

  template <template <typename Subview> class Object,
            typename Func, typename... Args>
  auto async_command(location_type loc, Object<this_type>& obj, Func func,
          Args const&... args) -> stapl::future<decltype((obj.*func)(args...))>
  {
    typedef decltype((obj.*func)(args...)) Result;
    typedef Object<this_type> NewObject;
    if (loc == this->get_location_id()) {
      return make_ready_future((obj.*func)(args...));
    } else {
      return stapl::opaque_rmi(loc, this->get_rmi_handle(),
        &this_type::template exec_async_command<Result, Object, Func, Args...>,
        obj, func, args...);
    }
  }

  template <typename Result, template <typename Subview> class Object,
            typename Func, typename... Args>
  Result exec_async_command(Object<this_type>& obj, Func func,
                            Args const&... args)
  {
    Object<this_type> new_obj(this, obj);
    return (new_obj.*func)(args...);
  }


  template <template <typename Subview> class Object,
            typename Func, typename... Args>
  auto sync_command(location_type loc, Object<this_type>& obj, Func func,
          Args const&... args) -> decltype((obj.*func)(args...))
  {
    return async_command(loc, obj, func, args...).get();
  }

  void write_dot(std::string filename)
  {
    delaunay_output_wf<this_type> wf(this, filename);
    wf.write_to_file();
  }
};


} // namespace delaunay


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_MESH_VIEW_HPP */
