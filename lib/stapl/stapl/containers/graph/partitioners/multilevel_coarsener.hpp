/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_COARSENER_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_COARSENER_HPP

#include <stapl/runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <boost/unordered_map.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/containers/graph/algorithms/create_level.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>

#define NMATCH_PASS 4

namespace stapl {

namespace partitioner_details {

//////////////////////////////////////////////////////////////////////
/// @brief Local information about the matching status of non-local vertices.
//////////////////////////////////////////////////////////////////////
class match_info
{
private:
  boost::unordered_map<size_t, size_t> m_received_edges;

public:
  /// Matching status
  /// MAYBE_MATCHED is used when a vertex sends a matching request to a
  /// neighbor but has not received an answer yet
  enum {UNMATCHED, MAYBE_MATCHED, MATCHED};

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if a vertex is unmatched.
  /// @param id vertex id.
  /// @return true if vertex is unmatched.
  //////////////////////////////////////////////////////////////////////
  bool is_unmatched(size_t const& id) const
  {
    boost::unordered_map<size_t, size_t>::const_iterator
      it = m_received_edges.find(id);
    return ( it == m_received_edges.end() || it->second == UNMATCHED);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add the matching status of a vertex to the local information.
  /// @param id vertex id.
  /// @param st matching status.
  //////////////////////////////////////////////////////////////////////
  void add_matched(size_t const& id, char const& st)
  { m_received_edges[id] = st; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Change status of MAYBE_MATCHED vertices to UNMATCHED.
  //////////////////////////////////////////////////////////////////////
  void clear_maybe_matched()
  {
    boost::unordered_map<size_t, size_t>::iterator
      it = m_received_edges.begin(),
      end_it = m_received_edges.end();
    for (; it != end_it; ++it)
    {
      if (it->second == MAYBE_MATCHED)
        it->second = UNMATCHED;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_received_edges);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to apply a functor on a vertex of a graph.
//////////////////////////////////////////////////////////////////////
struct target_aggr_wf
{
  //////////////////////////////////////////////////////////////////////
  /// @param g the graph pointer.
  /// @param uf the functor.
  //////////////////////////////////////////////////////////////////////
  template<typename Graph, typename UF>
  void operator() (Graph* g, UF const& uf) const
  {
    g->container_manager().apply(uf.target(), uf);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to clear MAYBE_MATCHED status of a vertex.
//////////////////////////////////////////////////////////////////////
struct clear_maybe_matched_wf
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param v the vertex proxy.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex v) const
  {
    v.property().property.clear_maybe_matched();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to clear MAYBE_MATCHED status of the local information
///        about non-local vertices.
//////////////////////////////////////////////////////////////////////
struct clear_maybe_matched2_wf
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param info the local information.
  //////////////////////////////////////////////////////////////////////
  template <typename Info>
  void operator()(Info info) const
  {
    info.clear_maybe_matched();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to validate a matching request.
/// @tparam Graph graph type.
/// @tparam Rem local information type.
//////////////////////////////////////////////////////////////////////
template <typename Graph, typename Rem, typename VP>
struct validate_request
{
  typedef VP property_type;

  typedef void result_type;
private:
  /// @todo mutable qualifier should be removed when is_local and locality
  /// of the pGraph container are made const.
  mutable Graph*  m_hview;
  mutable Rem*  m_rem;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param hview pointer to graph.
  /// @param rem pointer to local information.
  //////////////////////////////////////////////////////////////////////
  validate_request(Graph* hview, Rem* rem)
    : m_hview(hview), m_rem(rem)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex reference that initiated the request.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex& v) const
  {
    size_t matched_id = v.property().property.get_matched_vertex();
    v.property().property.set_matched(property_type::MATCHED);

    //Add info to boundary info
    (*m_rem)[get_location_id()].add_matched(matched_id, property_type::MATCHED);
    size_t myid = v.descriptor();

    //tell my remote neighbors that I am matched
    for (auto const& e : v)
    {
      size_t target = e.target();
      if (!m_hview->is_local(target) && target != matched_id)
      {
        (*m_rem)[m_hview->locality(target).location()].add_matched(
          myid, property_type::MATCHED);
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_hview);
    t.member(m_rem);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor requesting the matching of a vertex.
/// @tparam Graph graph type.
/// @tparam Rem local information type.
//////////////////////////////////////////////////////////////////////
template <typename Graph, typename Rem, typename VP>
struct match_vertex
{
  typedef VP property_type;

  typedef void result_type;
private:
  size_t m_vertex_id;
  mutable Graph*  m_hview;
  mutable Rem*  m_rem;
  size_t m_target;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param vertex_id id of the vertex requesting a matching.
  /// @param hview pointer to graph.
  /// @param rem pointer to local information.
  /// @param target id of the target vertex of the matching request.
  //////////////////////////////////////////////////////////////////////
  match_vertex(size_t const& vertex_id, Graph* hview, Rem* rem,
               size_t const& target)
    : m_vertex_id(vertex_id), m_hview(hview), m_rem(rem), m_target(target)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex reference, target of the matching request.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex& v) const
  {
    bool unmatched = v.property().property.is_unmatched();
    if (unmatched)
    {
      v.property().property.set_matched(property_type::MATCHED, m_vertex_id);
      size_t myid = v.descriptor();

      //Add info to boundary info
      (*m_rem)[get_location_id()].add_matched(m_vertex_id,
                                              property_type::MATCHED);

      //confirm to my matched vertex
      m_hview->apply_set(m_vertex_id,
                         validate_request<Graph, Rem, property_type>(m_hview,
                                                                     m_rem));

      //tell my remote neighbors that I am matched
      for (auto const& e : v)
      {
        size_t target = e.target();
        if (!m_hview->is_local(target) && target != m_vertex_id)
        {
          (*m_rem)[m_hview->locality(target).location()].add_matched(
            myid, property_type::MATCHED);
        }
      }

    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns target id of the matching request.
  //////////////////////////////////////////////////////////////////////
  size_t target() const
  {
    return m_target;
  }

  void define_type(typer& t)
  {
    t.member(m_vertex_id);
    t.member(m_hview);
    t.member(m_rem);
    t.member(m_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to share matching status of a vertex with
///        neighboring vertices.
/// @tparam HView graph view type.
/// @tparam RemoteInfo local information view type.
//////////////////////////////////////////////////////////////////////
template <typename HView, typename RemoteInfo, typename VP>
struct share_matching_status_wf
{
  HView    m_hview;
  RemoteInfo  m_remote_info;

  typedef VP property_type;
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param hview graph view type.
  /// @param remote_info local information view containing matching status of
  ///        non-local vertices.
  //////////////////////////////////////////////////////////////////////
  share_matching_status_wf(HView const& hview, RemoteInfo const& remote_info)
    : m_hview(hview), m_remote_info(remote_info)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex sharing its matching status.
  //////////////////////////////////////////////////////////////////////
  template<typename V>
  void operator()(V& v) const
  {
    size_t target;
    for (auto const& e : v) {
      target = e.target();
      if (!m_hview.container().is_local(target)) {
        m_remote_info[m_hview.container().locality(target).location()].
          add_matched(v.descriptor(), property_type::MATCHED);
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_hview);
    t.member(m_remote_info);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor trying to match a vertex with one of its neighbor.
/// @tparam HView graph view type.
/// @tparam RemoteInfo local information view type.
//////////////////////////////////////////////////////////////////////
template <typename HView, typename RemoteInfo, typename VP>
struct vertex_match_wf
{
  typedef VP property_type;

private:
  mutable aggregator_apply<
    match_vertex<typename HView::view_container_type,
                 typename RemoteInfo::view_container_type,
                 property_type>,
    typename HView::view_container_type, target_aggr_wf>
  m_aggr;
  mutable HView m_hview;
  mutable RemoteInfo m_remote_info;
  mutable unsigned int m_seed;
  size_t m_loc;
  size_t m_wside;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param wside value alternating between 0 and 1 used to decide which
  ///        vertex is in charge of a request for remote matching.
  /// @param hview graph view type.
  /// @param remote_info local information view containing matching status of
  ///        non-local vertices.
  //////////////////////////////////////////////////////////////////////
  vertex_match_wf(size_t const& wside, HView const& hview,
                  RemoteInfo const& remote_info)
    : m_aggr(hview.get_container(), MULTILEVEL_AGGREGATION), m_hview(hview),
      m_remote_info(remote_info), m_seed(1), m_loc(get_location_id()),
      m_wside(wside)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex reference.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex& v) const
  {
    if (v.property().property.is_matched())
    {
      return;
    }
    size_t myid = v.descriptor();
    bool matched = false;
    bool matched_local = false;
    size_t possible_match = 0;
    size_t highest_weight = 0;

    //Find a heavy-edge matching
    size_t curr_weight, target, local;
    for (auto const& e : v) {
      target = e.target();
      local = m_hview.container().is_local(target);
      curr_weight = e.property().property.adjusted_weight;
      if ( (!matched || curr_weight > highest_weight ||
            (curr_weight == highest_weight &&
             ( (!matched_local && local) || rand_r(&m_seed)%2 == 0) ) ) &&
           ( (local && m_hview[target].property().property.is_unmatched()) ||
             (!local && m_remote_info[m_loc].is_unmatched(target)) ) ) {
        matched_local = local;
        matched = true;
        highest_weight = curr_weight;
        possible_match = target;
      }
    }

    if (matched)
    {
      //local matched
      if (m_hview.container().is_local(possible_match))
      {
        v.property().property.set_matched(property_type::MATCHED,
                                          possible_match);
        m_hview[possible_match].property().property.set_matched(
          property_type::MATCHED, myid);

        //tell my remote neighbors that I am matched
        for (auto const& e : v)
        {
          target = e.target();
          if (!m_hview.container().is_local(target))
          {
            m_remote_info[m_hview.container().locality(target).location()].
              add_matched(myid, property_type::MATCHED);
          }
        }
        //tell my remote neighbors that my sibling is matched
        m_hview.apply_set(possible_match,
                          share_matching_status_wf<HView, RemoteInfo,
                                                   property_type>(m_hview,
                                                                  m_remote_info)
                          );

      }
      else
      {  // remote matched
        bool responsible = m_wside ? myid < possible_match
                                   : myid > possible_match;
        m_remote_info[m_loc].add_matched(possible_match,
                                         property_type::MAYBE_MATCHED);

        if (responsible)
        {
          v.property().property.set_matched(property_type::MAYBE_MATCHED,
                                            possible_match);
          m_aggr.add(match_vertex<typename HView::view_container_type,
                                  typename RemoteInfo::view_container_type,
                                  property_type>(
                       myid, m_hview.get_container(),
                       m_remote_info.get_container(), possible_match));
        }
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_aggr);
    t.member(m_hview);
    t.member(m_remote_info);
    t.member(m_loc);
    t.member(m_wside);
    t.member(m_seed);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to extract group id from the vertex property for
///        the creation of a super-vertex in a new level of hierarchy.
//////////////////////////////////////////////////////////////////////
struct get_group_id
{
  typedef size_t value_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p vertex property.
  //////////////////////////////////////////////////////////////////////
  template<typename Property>
  value_type get(Property p) const
  {
     return p.property.get_matched_vertex();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor adjusting the matched value of a vertex to be the same
///        as the one of its matched vertex, this value is used for the
///        creation of a level of hierarchy.
//////////////////////////////////////////////////////////////////////
struct adjust_matched_value
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex proxy.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex v)
  {
    size_t vid = v.descriptor();
    size_t matched_id = v.property().property.get_matched_vertex();

    bool responsible = true;
    if (v.property().property.is_matched())
    {
      unsigned int myseed = vid+matched_id;
      responsible = rand_r(&myseed)%2 ? vid < matched_id
                                      : vid > matched_id;
    }
    v.property().property.set_matched_vertex(responsible ? vid : matched_id);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to shuffle elements of a base container.
//////////////////////////////////////////////////////////////////////
struct shuffle_wf
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param cont base container.
  //////////////////////////////////////////////////////////////////////
  template <typename Bc>
  void operator()(Bc cont)
  {
    std::random_shuffle(cont.begin(), cont.end());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor dividing a property with a specific weight.
//////////////////////////////////////////////////////////////////////
struct property_divide_wf
{
private:
  size_t m_weight;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param weight the weight used in the division.
  //////////////////////////////////////////////////////////////////////
  property_divide_wf(size_t const& weight)
    : m_weight(weight)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param p a property reference.
  //////////////////////////////////////////////////////////////////////
  template <typename Property>
  void operator()(Property& p) const
  {
    p.property.adjusted_weight = p.property.weight/m_weight;
  }

  void define_type(typer& t)
  {
    t.member(m_weight);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor adding a specific weight to a property.
//////////////////////////////////////////////////////////////////////
struct property_add_wf
{
private:
  size_t m_weight;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param weight the weight used in the addition.
  //////////////////////////////////////////////////////////////////////
  property_add_wf(size_t const& weight)
    : m_weight(weight)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param p a property reference.
  //////////////////////////////////////////////////////////////////////
  template <typename Property>
  void operator()(Property& p) const
  {
    p.property.adjusted_weight = p.property.adjusted_weight + m_weight;
  }
  void define_type(typer& t)
  {
    t.member(m_weight);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor adjusting an edge weight with the target vertex weight
///        for improving the matching phase.
/// @tparam G graph view type.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct adjust_edge_weight_wf
{
  typedef typename G::edge_descriptor edge_desc_t;
  typedef create_level_detail::
          ep_apply_aggr_wf<edge_desc_t,property_divide_wf> ep_apply_aggr_wf_t;
  aggregator_apply<ep_apply_aggr_wf_t, typename G::view_container_type,
                   aggr_algo_detail::aggregator_helper_wf> m_aggr;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param g graph view.
  //////////////////////////////////////////////////////////////////////
  adjust_edge_weight_wf(G const& g)
    : m_aggr(g.get_container(), MULTILEVEL_AGGREGATION)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex reference.
  /// @param hview graph view.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename HView>
  void operator()(Vertex v, HView hview)
  {
    size_t vid = v.descriptor();
    size_t vertex_weight = v.property().property.get_weight();
    typename Vertex::adj_edge_iterator edge_it = v.begin(),
                                       edge_end_it = v.end();
    //Divide sister edges
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      m_aggr.add(ep_apply_aggr_wf_t(edge_desc_t((*edge_it).target(), vid),
                 property_divide_wf(vertex_weight)));
    }

  }
  void define_type(typer& t)
  { t.member(m_aggr); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor adjusting an edge weight with the target vertex outdegree
///        for improving the matching phase.
/// @tparam G graph view type.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct adjust_edge_weight_wf2
{
  typedef typename G::edge_descriptor edge_desc_t;
  typedef create_level_detail::
      ep_apply_aggr_wf<edge_desc_t, property_add_wf> ep_apply_aggr_wf_t;
  aggregator_apply<ep_apply_aggr_wf_t, typename G::view_container_type,
                   aggr_algo_detail::aggregator_helper_wf> m_aggr;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param g graph view.
  //////////////////////////////////////////////////////////////////////
  adjust_edge_weight_wf2(G const& g)
    : m_aggr(g.get_container(), MULTILEVEL_AGGREGATION)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex reference.
  /// @param hview graph view.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename HView>
  void operator()(Vertex v, HView hview)
  {
    size_t vid = v.descriptor();

    //Calculate outgoing edge weight
    size_t out_weight = 0;
    typename Vertex::adj_edge_iterator edge_it = v.begin(),
                                       edge_end_it = v.end();
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      out_weight += (*edge_it).property().property.weight;
    }
    //add (edge_weight)/(out_degree_weight-edge_weight) to sibling edge
    for (edge_it=v.begin(); edge_it!=edge_end_it; ++edge_it)
    {
      m_aggr.add(ep_apply_aggr_wf_t(edge_desc_t((*edge_it).target(), vid),
                 property_add_wf((*edge_it).property().property.weight/
                 (out_weight-(*edge_it).property().property.weight+1))));
    }

  }
  void define_type(typer& t)
  { t.member(m_aggr); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Adjust edge weights with target vertex weights and outdegrees.
/// @param hview graph view.
//////////////////////////////////////////////////////////////////////
template <typename HView>
void adjust_edge_weight(HView& hview)
{
  map_func(adjust_edge_weight_wf<HView>(hview), hview, make_repeat_view(hview));
  map_func(adjust_edge_weight_wf2<HView>(hview),hview, make_repeat_view(hview));
}


//////////////////////////////////////////////////////////////////////
/// @brief Functor to apply a functor on an element of a view.
/// @tparam WF functor type.
//////////////////////////////////////////////////////////////////////
template <typename WF>
struct apply_vertex_wf
{
private:
  WF m_wf;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param wf functor to be applied on a view element.
  //////////////////////////////////////////////////////////////////////
  apply_vertex_wf(WF const& wf)
    : m_wf(wf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param elt view element id.
  /// @param hview view.
  //////////////////////////////////////////////////////////////////////
  template <typename Elt, typename HView>
  void operator()(Elt elt, HView& hview)
  {
    hview.apply_set(elt, m_wf);
  }

  void define_type(typer& t)
  { t.member(m_wf); }
};

} //namespace partitioner_details


template <typename Accessor>
class proxy<partitioner_details::match_info, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef partitioner_details::match_info target_t;

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

  bool is_unmatched(size_t const& id) const
  { return Accessor::const_invoke(&target_t::is_unmatched, id); }

  void add_matched(size_t const& id, char const& st)
  { Accessor::invoke(&target_t::add_matched, id, st); }

  void clear_maybe_matched()
  { Accessor::invoke(&target_t::clear_maybe_matched); }
}; //struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief Match pair of vertices connected with a heavy edge weight
///        and create a new graph with a super-vertex representing a pair
///        of matched vertices or an unmatched vertex.
/// @param hview graph view whose vertices are matched.
/// @param level level of hierarchy of @p hview.
/// @return new level of hierarchy containing the super-vertices.
//////////////////////////////////////////////////////////////////////
template <typename HView>
HView heavy_edge_matching_collapser(HView const& hview, size_t const& level)
{
  using namespace partitioner_details;

  //Adjust edge weights
  adjust_edge_weight(hview);

  //pArray for remote matched
  typedef static_array<match_info> remote_cont;
  typedef array_view<remote_cont> remote_view;
  remote_cont remote_matched(get_num_locations());
  remote_view remote_matched_view(remote_matched);
  size_t wside;


  //pArray for permutations
  static_array<size_t> permutations(hview.size());
  array_view<static_array<size_t> > perm_vw(permutations);
  copy(counting_view<size_t>(hview.size()), perm_vw);
  map_func(shuffle_wf(), native_view(perm_vw));

  for (size_t i = 0; i < NMATCH_PASS; ++i)
  {
    wside = (level+i)%2;
    //Try to match vertices
    typedef vertex_match_wf<HView, remote_view,
                            typename HView::vertex_property::property_type> wf_t;
    map_func(apply_vertex_wf<wf_t>(wf_t(wside, hview, remote_matched_view)),
             perm_vw, make_repeat_view(hview));

    rmi_fence(); //fence because 2 levels of messages:
                 //1-matched 2-confirm request
    //Clear 'maybe_matched' vertices
    if (i != NMATCH_PASS-1)
    {
      map_func(clear_maybe_matched_wf(), hview);
      map_func(clear_maybe_matched2_wf(), remote_matched_view);
    }
  }

  map_func(adjust_matched_value(), hview);

  typedef graph_internal_property_map<typename HView::view_container_type,
                                      get_group_id> p_property_map_view_type;
    p_property_map_view_type group_map(hview.container(), get_group_id());

  HView next_level =
    create_level(hview, group_map,
                 std::plus<typename HView::vertex_property::property_type>(),
                 std::plus<typename HView::edge_property::property_type>(),
                 MULTILEVEL_AGGREGATION, false, true);

  return next_level;
}

} //namespace stapl

#endif
