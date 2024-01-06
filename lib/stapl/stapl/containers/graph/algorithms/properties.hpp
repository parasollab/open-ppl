/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PROPERTIES_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PROPERTIES_HPP

#include <stapl/containers/graph/graph.hpp>
#include <boost/serialization/deque.hpp>
#include <stapl/views/proxy_macros.hpp>

#include <stapl/containers/graph/algorithms/properties/page_rank_kla.hpp>

namespace stapl {

namespace properties {

///////////////////////////////////////////////////////////////////////////
/// @brief Message class used to send flow and vertex information between
/// neighboring vertices in the preflow push algorithm.
/// @ingroup pgraphAlgoDetails
///////////////////////////////////////////////////////////////////////////
class message
{
public:
  typedef graph<DIRECTED, NONMULTIEDGES>::vertex_descriptor  vd_type;

  bool      m_response;
  double    m_flow;
  vd_type   m_sender;
  vd_type   m_receiver;
  int       m_height;

  message(void)
    : m_response(false), m_flow(0), m_sender(), m_receiver(), m_height(0)
  { }

  message(bool answer, double flow, vd_type vd_send, vd_type vd_receive,
    int height)
    : m_response(answer), m_flow(flow), m_sender(vd_send),
      m_receiver(vd_receive), m_height(height)
  { }

  void response(bool response)
  { m_response = response; }

  void flow(double units)
  { m_flow = units; }

  void sender_vd(vd_type ed_value)
  { m_sender = ed_value; }

  void receiver_vd(vd_type ed_value)
  { m_receiver = ed_value; }

  void height(int height)
  { m_height = height; }

  bool response(void)  const
  { return m_response;}

  double flow(void) const
  { return m_flow; }

  vd_type sender_vd(void) const
  { return m_sender; }

  vd_type receiver_vd(void) const
  { return m_receiver; }

  int height(void) const
  { return m_height; }

  void define_type (typer& t)
  {
    t.member(m_response);
    t.member(m_flow);
    t.member(m_sender);
    t.member(m_receiver);
    t.member(m_height);
  }
};

///////////////////////////////////////////////////////////////////////////
/// @brief Class with accessor and mutator methods to represent the
/// properties of vertices in preflow push algorithm.
/// @ingroup pgraphAlgoDetails
///////////////////////////////////////////////////////////////////////////
class preflowpush_vertex_property
{
public:
  typedef message                                            message_type;
  typedef graph<DIRECTED, NONMULTIEDGES>::vertex_descriptor  vd_type;

  double                      m_excess;
  int                         m_height;
  std::deque<message_type>    m_msg_deq;
  int                         m_stop;

  preflowpush_vertex_property(void)
    : m_excess(0), m_height(0), m_stop(0)
  { }

  preflowpush_vertex_property(int excess, int height, int stop=0)
    : m_excess(excess), m_height(height), m_stop(stop)
  { }

  void excess(double value)
  { m_excess = value; }

  double excess(void) const
  { return m_excess;  }

  void dec_ex(double value)
  { m_excess -= value; }

  void inc_ex(double value)
  { m_excess += value; }

  int height(void) const
  { return m_height;  }

  void height(int value)
  { m_height = value; }

  void inc_height(void)
  { m_height++; }

  int stop(void) const
  { return m_stop; }

  void stop(int edge_it)
  { m_stop = edge_it; }

  void add_msg(message_type msg)
  { m_msg_deq.push_back(msg); }

  void pop_msg(void)
  { m_msg_deq.pop_front(); }

  std::size_t msgs(void)
  { return m_msg_deq.size(); }

  message_type get_msg(void)
  { return m_msg_deq.front(); }

  void define_type (typer& t)
  {
    t.member(m_excess);
    t.member(m_height);
    t.member(m_msg_deq);
    t.member(m_stop);
  }
};



///////////////////////////////////////////////////////////////////////////
/// @brief Class with accessor and mutator methods to represent the
/// properties of edges in preflow push algorithm.
/// @ingroup pgraphAlgoDetails
///////////////////////////////////////////////////////////////////////////
class preflowpush_edge_property
{
protected:
  double      m_rc;
  bool        m_backward;
  int         m_height;

public:
  preflowpush_edge_property(void)
    : m_rc(1), m_backward(false), m_height(0)
  { }

  preflowpush_edge_property(double rc, bool backward, int height)
    :  m_rc(rc), m_backward(backward), m_height(height)
  { }

  double rc(void) const
  { return m_rc; }

  void rc(double value)
  { m_rc = value; }

  void inc_rc(double amt)
  { m_rc += amt; }

  void dec_rc(double amt)
  { m_rc -= amt; }

  void height(int height)
  { m_height = height;}

  int height(void) const
  { return m_height; }

  bool backward(void) const
  { return m_backward; }

  void define_type (typer& t)
  {
    t.member(m_rc);
    t.member(m_backward);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref bad_rank().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class bad_rank_property
{
public:
  typedef double value_type;

private:
  value_type m_color;
  value_type m_color2;
  size_t     m_active;
  size_t     m_blacklisted;

public:
  bad_rank_property(void)
    : m_color(0.0), m_color2(0.0), m_active(true), m_blacklisted(false)
  { }

  value_type rank(void) const
  { return m_color; }

  void rank(value_type val)
  { m_color = val;  }

  value_type new_rank(void) const
  { return m_color2; }

  void new_rank(value_type val)
  { m_color2 = val; }

  bool is_active(void) const
  { return m_active; }

  void set_active(bool t)
  { m_active = t; }

  bool is_blacklisted(void) const
  { return m_blacklisted; }

  void set_blacklisted(bool t)
  { m_blacklisted = t; }

  void define_type(stapl::typer& t)
  {
    t.member(m_color);
    t.member(m_color2);
    t.member(m_active);
    t.member(m_blacklisted);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref betweenness_centrality().
/// Set of properties for each vertex, distance, delta, sigma,
/// list of parent vertices, and a marker for BFS visitation.
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
struct bc_property
{
  typedef size_t                    vd_type;
  typedef size_t                    dist_type;
  typedef double                    sigma_type;
  typedef double                    delta_type;
  typedef std::vector<vd_type>      bfs_dag_type;
  typedef std::vector<vd_type>      level_type;

  typedef std::vector<dist_type>    distances_t;
  typedef std::vector<sigma_type>   sigmas_t;
  typedef std::vector<delta_type>   deltas_t;
  typedef std::vector<bfs_dag_type> bfs_dags_t;
  typedef std::deque<vd_type>       active_t;
  typedef std::vector<level_type>   levels_t;

  delta_type  m_BC;
  distances_t m_distances;
  sigmas_t    m_sigmas;
  deltas_t    m_deltas;
  bfs_dags_t  m_bfs_dags;
  active_t    m_active1;
  active_t    m_active2;
  bool        m_active_flip;
  levels_t    m_levels;

  bc_property(void)
    : m_BC(0.0), m_active_flip(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes member properties and allocates space for
  /// a given number of sources.
  //////////////////////////////////////////////////////////////////////
  void initialize(size_t num_sources)
  {
    m_BC = 0.0;
    m_active_flip = true;

    m_distances.clear();
    m_sigmas.clear();
    m_deltas.clear();
    m_bfs_dags.clear();
    m_active1.clear();
    m_active2.clear();
    m_levels.clear();

    m_distances.resize(num_sources, 0);
    m_sigmas.resize(num_sources, 0);
    m_deltas.resize(num_sources, 0);
    m_bfs_dags.resize(num_sources);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resets member properties before calculating next set of
  /// sources.
  //////////////////////////////////////////////////////////////////////
  void reset_sources(void)
  {
    m_active_flip = true;
    m_active1.clear();
    m_active2.clear();
    m_levels.clear();
    std::fill(m_distances.begin(), m_distances.end(), 0);
    std::fill(m_sigmas.begin(), m_sigmas.end(), 0.0);
    std::fill(m_deltas.begin(), m_deltas.end(), 0.0);
    for (size_t i=0; i<m_bfs_dags.size(); ++i)
      m_bfs_dags[i].clear();
  }

  inline delta_type BC(void) const
  { return m_BC; }

  inline void BC(delta_type bc)
  { m_BC = bc; }

  inline dist_type distance(vd_type source_id) const
  { return m_distances[source_id]; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the distance of vertex from source to the value dist.
  /// Also adds the source id to list of sources at level=dist.
  //////////////////////////////////////////////////////////////////////
  inline void distance(vd_type source_id, dist_type dist)
  {
    m_distances[source_id] = dist;
    if (dist >= m_levels.size())
      m_levels.resize(dist+1);
    m_levels[dist].push_back(source_id);
  }

  inline delta_type delta(vd_type source_id) const
  { return m_deltas[source_id]; }

  inline void delta(vd_type source_id, delta_type delta)
  { m_deltas[source_id] = delta; }

  inline sigma_type sigma(vd_type source_id) const
  { return m_sigmas[source_id]; }

  inline void sigma(vd_type source_id, sigma_type sigma)
  { m_sigmas[source_id] = sigma; }

  inline const bfs_dag_type bfs_dag(vd_type source_id) const
  { return m_bfs_dags[source_id]; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Appends vertex to bfs dag if it is not already in the bfs dag
  /// and increments sigma value.
  /// @param source_id Source id of currently active source
  /// @param p_id Numeric id of parent vertex
  /// @param sig Sigma value of parent vertex
  //////////////////////////////////////////////////////////////////////
  inline void bfs_dag(vd_type source_id, vd_type p_id, sigma_type sig)
  {
    bfs_dag_type::const_iterator it =
        std::find(m_bfs_dags[source_id].begin(),
                  m_bfs_dags[source_id].end(), p_id);
    if (it == m_bfs_dags[source_id].end()) {
      m_bfs_dags[source_id].push_back(p_id);
      m_sigmas[source_id] += sig;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a list of sources that are on the same level (i.e.
  /// distance away from this vertex). Used in calculating delta, where
  /// delta is recursively defined starting from last level.
  /// @param level Level is equal to distance from source to this
  /// vertex.
  /// @see dependency_wf
  //////////////////////////////////////////////////////////////////////
  inline const level_type level_sources(dist_type level) const
  {
    if (level < m_levels.size())
      return m_levels[level];
    else
      return level_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Gets the next active source id and removes it from the list.
  //////////////////////////////////////////////////////////////////////
  inline vd_type next_active_traversal(void)
  {
    if (m_active_flip) {
      vd_type vd = m_active1.front();
      m_active1.pop_front();
      return vd;
    } else {
      vd_type vd = m_active2.front();
      m_active2.pop_front();
      return vd;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if there are any active sources left to process.
  //////////////////////////////////////////////////////////////////////
  inline bool is_active(void) const
  {
    if (m_active_flip)
      return !m_active1.empty();
    else
      return !m_active2.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a source id to the list of active sources.
  /// @see mssp_wf
  //////////////////////////////////////////////////////////////////////
  inline void set_active(vd_type source_id)
  {
    if (m_active_flip)
      m_active1.push_back(source_id);
    else
      m_active2.push_back(source_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a source id to the list of pending sources to be
  /// worked on the next level of mssp.
  /// @see mssp_wf
  //////////////////////////////////////////////////////////////////////
  inline void set_pending(vd_type source_id)
  {
    if (m_active_flip)
      m_active2.push_back(source_id);
    else
      m_active1.push_back(source_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the finished list of active sources worked on and
  /// sets the pending sources to be worked on next level.
  /// @see mssp_lsync
  /// @see post_execute
  //////////////////////////////////////////////////////////////////////
  inline void push_pending(void)
  { m_active_flip = !m_active_flip; }

  void define_type(typer& t)
  {
    t.member(m_BC);
    t.member(m_distances);
    t.member(m_sigmas);
    t.member(m_deltas);
    t.member(m_bfs_dags);
    t.member(m_active1);
    t.member(m_active2);
    t.member(m_active_flip);
    t.member(m_levels);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref breadth_first_search().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class bfs_property
{
private:
  typedef size_t VD;
  typedef size_t Level;

  VD    m_vd;
  Level m_level;

public:
  typedef VD    parent_type;
  typedef Level level_type;

  bfs_property(void)
    : m_vd(std::numeric_limits<VD>::max()),
      m_level(std::numeric_limits<Level>::max())
  { }

  inline VD parent(void) const
  { return m_vd; }

  inline void parent(VD const& vd)
  { m_vd = vd; }

  inline Level level(void) const
  { return m_level; }

  inline void level(Level const& c)
  { m_level = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_vd);
    t.member(m_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref approximate_breadth_first_search().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class approximate_bfs_property
{
private:
  typedef size_t VD;
  typedef size_t Level;

  VD    m_vd;
  Level m_level;
  Level m_propagated_level;
  bool  m_active;

public:
  typedef VD    parent_type;
  typedef Level level_type;

  approximate_bfs_property(void)
    : m_vd(std::numeric_limits<VD>::max()),
      m_level(std::numeric_limits<Level>::max()),
      m_propagated_level(std::numeric_limits<Level>::max()),
      m_active(false)
  { }

  VD parent(void) const
  { return m_vd; }

  void parent(VD const& vd)
  { m_vd = vd; }

  Level level(void) const
  { return m_level; }

  void level(Level const& c)
  { m_level = c; }

  Level propagated_level(void) const
  { return m_propagated_level; }

  void propagated_level(Level const& c)
  { m_propagated_level = c; }

  bool active(void) const
  { return m_active; }

  void active(bool v)
  { m_active = v; }

  void define_type(stapl::typer& t)
  {
    t.member(m_vd);
    t.member(m_level);
    t.member(m_propagated_level);
    t.member(m_active);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref closeness_centrality().
///
/// Derives from @ref bfs_property to reuse its components.
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class closeness_property
  : public bfs_property
{
  double m_closeness;

public:
  closeness_property(void)
    : m_closeness(0.0)
  { }

  inline double closeness(void) const
  { return m_closeness; }

  inline void closeness(double const& d)
  { m_closeness = d; }

  void define_type(stapl::typer& t)
  {
    t.base<bfs_property>(*this);
    t.member(m_closeness);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref coloring().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class coloring_property
{
 public:
  typedef size_t color_value_type;

 private:
  color_value_type   m_color;

 public:
  coloring_property(size_t i = 0)
    : m_color(i)
  { }

  coloring_property(coloring_property const& other) = default;

  color_value_type get_color() const
  { return m_color; }

  void set_color(color_value_type const& c)
  { m_color = c; }

  void define_type(typer& t)
  { t.member(m_color); }
};



//////////////////////////////////////////////////////////////////////
/// @brief Edge property for use with @ref boruvka() minimum-spanning
/// tree algorithm.
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
template<typename VertDesc, typename Weight>
class boruvka_edge_property
{
private:
  VertDesc m_source;
  VertDesc m_target;
  Weight   m_weight;

public:
  typedef Weight weight_type;

  boruvka_edge_property(VertDesc s, int t, Weight p)
    : m_source(s), m_target(t), m_weight(p)
  { }

  boruvka_edge_property() = default;

  boruvka_edge_property(boruvka_edge_property const& ep)
    : m_source(ep.source()), m_target(ep.target()), m_weight(ep.weight())
  { }

  VertDesc source() const
  { return m_source; }

  void source(VertDesc const& s)
  { m_source = s; }

  VertDesc target() const
  { return m_target; }

  void target(VertDesc const& t)
  { m_target = t; }

  Weight weight() const
  { return m_weight; }

  void weight(Weight const& p)
  { m_weight = p; }

  void define_type(typer& t)
  {
    t.member(m_source);
    t.member(m_target);
    t.member(m_weight);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref connected_components().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class cc_property
{
private:
  size_t m_cc;
  bool m_active;

public:
  cc_property(void)
    : m_cc(std::numeric_limits<size_t>::max()), m_active(true)
  { }

  cc_property(size_t c)
    : m_cc(c), m_active(false)
  { }

  void cc(size_t c)
  { m_cc = c; }

  size_t cc(void) const
  { return m_cc; }

  void active(bool a)
  { m_active = a; }

  bool active(void) const
  { return m_active; }

  void define_type(stapl::typer& t)
  {
    t.member(m_cc);
    t.member(m_active);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref community_detection().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class cd_property
{
public:
  typedef size_t label_type;
  typedef std::vector<label_type> labels_type;

private:
  label_type  m_label;
  labels_type m_neighboring_labels;

public:
  cd_property(void)
    : m_label(std::numeric_limits<size_t>::max()), m_neighboring_labels(0)
  { }

  cd_property(label_type c)
    : m_label(c), m_neighboring_labels(0)
  { }

  void label(label_type const& c)
  { m_label = c; }

  label_type label(void) const
  { return m_label; }

  labels_type label_vector(void) const
  { return m_neighboring_labels; }

  void label_vector_add(label_type const& c)
  { m_neighboring_labels.push_back(c); }

  void label_vector_clear(void)
  { m_neighboring_labels.clear(); }

  void define_type(stapl::typer& t)
  {
    t.member(m_label);
    t.member(m_neighboring_labels);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref link_prediction().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
template<typename VD>
class lp_property
{
public:
  typedef VD desc_t;
  typedef std::vector<double> link_probabilities_t;
  typedef std::vector<desc_t> step_2_buffer_t;
  typedef std::vector<std::pair<desc_t, step_2_buffer_t> > step_3_buffer_t;

private:
  link_probabilities_t m_link_probabilities;
  step_2_buffer_t      m_step_2_buffer;
  step_3_buffer_t      m_step_3_buffer;

public:
  lp_property(void) = default;

  void link_probability_size(size_t const& sz)
  { m_link_probabilities.resize(sz); }

  size_t link_probability_size(void) const
  { return m_link_probabilities.size(); }

  void link_probability(size_t const& idx, double const& p)
  { m_link_probabilities[idx] = p; }

  void add_link_probability(size_t const& idx, double const& p)
  { m_link_probabilities[idx] += p; }

  link_probabilities_t link_probabilities(void) const
  { return m_link_probabilities; }

  double link_probability(size_t const& idx) const
  { return m_link_probabilities[idx]; }

  step_2_buffer_t step_2_buffer(void) const
  { return m_step_2_buffer; }

  void step_2_buffer_add(desc_t const& src)
  { m_step_2_buffer.push_back(src); }

  void step_2_buffer_clear(void)
  { m_step_2_buffer.clear(); }

  step_3_buffer_t step_3_buffer(void) const
  { return m_step_3_buffer; }

  void step_3_buffer_add(desc_t const& tgt, step_2_buffer_t const& srcs)
  { m_step_3_buffer.push_back(std::make_pair(tgt, srcs)); }

  void step_3_buffer_clear(void)
  { m_step_3_buffer.clear(); }

  void define_type(stapl::typer& t)
  {
    t.member(m_link_probabilities);
    t.member(m_step_2_buffer);
    t.member(m_step_3_buffer);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Indicates the partition of a vertex in the bipartite graph.
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
enum bipartite_partition
{ LEFT, RIGHT };

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref maximal_bipartite_matching().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class mbm_property
{
public:
  typedef size_t vd_type;

private:
  bipartite_partition m_partition;
  bool    m_matched;
  size_t  m_level;
  vd_type m_match;

public:
  mbm_property(void)
    : m_matched(false), m_level(0),
      m_match(std::numeric_limits<vd_type>::max())
  { }

  bipartite_partition partition(void) const
  { return m_partition; }

  void partition(bipartite_partition const& p)
  { m_partition = p; }

  bool matched(void) const
  { return m_matched; }

  void matched(bool b)
  { m_matched = b; }

  size_t level(void) const
  { return m_level; }

  void level(size_t b)
  { m_level = b; }

  vd_type match(void) const
  { return m_match; }

  void match(vd_type const& v)
  { m_match = v; }

  void define_type(typer& t)
  {
    t.member(m_partition);
    t.member(m_matched);
    t.member(m_level);
    t.member(m_match);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref mssp().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class mssp_property
{
public:
  typedef size_t VD;
  typedef boost::unordered_map<VD, double> distances_t;
  typedef std::vector<VD> active_t;

private:
  boost::unordered_map<VD, VD>     m_vd;
  distances_t m_distance;
  active_t    m_active;

public:
  typedef VD     parent_type;
  typedef double dist_type;

  inline VD parent(VD const& traversal_id)
  { return m_vd[traversal_id]; }

  inline dist_type distance(VD const& traversal_id)
  { return m_distance[traversal_id]; }

  inline distances_t distances(void) const
  { return m_distance; }

  inline void parent(VD const& traversal_id, VD const& vd)
  { m_vd[traversal_id] = vd; }

  inline void distance(VD const& traversal_id, dist_type const& c)
  { m_distance[traversal_id] = c; }

  inline bool is_active(void) const
  { return !m_active.empty(); }

  inline VD next_active_traversal(void) const
  { return m_active[0]; }

  inline void set_active(VD traversal_id)
  { m_active.push_back(traversal_id); }

  inline void set_inactive(VD traversal_id)
  {
    active_t::iterator it =
      std::find(m_active.begin(), m_active.end(), traversal_id);
    if (it != m_active.end())
      m_active.erase(it);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_vd);
    t.member(m_distance);
    t.member(m_active);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref page_rank().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
template<typename Rank>
class page_rank_property
{
public:
  using value_type = Rank;

private:
  value_type m_color;
  value_type m_color2;

public:
  page_rank_property(void)
    : m_color(0.0), m_color2(0.0)
  { }

  value_type rank(void) const
  { return m_color;  }

  void rank(value_type val)
  { m_color = val;  }

  value_type new_rank(void) const
  { return m_color2; }

  void new_rank(value_type val)
  { m_color2 = val; }

  void define_type(stapl::typer& t)
  {
    t.member(m_color);
    t.member(m_color2);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref random_walk().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class rw_property
{
private:
  typedef size_t Level;

  size_t m_visits;
  Level  m_level;

public:
  typedef Level level_type;

  rw_property(void)
    : m_visits(0),
      m_level(std::numeric_limits<Level>::max())
  { }

  inline size_t visits(void) const
  { return m_visits; }

  inline void visits(size_t const& c)
  { m_visits = c; }

  inline Level level(void) const
  { return m_level; }

  inline void level(Level const& c)
  { m_level = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_visits);
    t.member(m_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref pscc.
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
template <typename VertexGIDType>
class scc_property
{
 public:
  typedef VertexGIDType                         color_type;

 private:
  color_type      m_cc;

 public:
  scc_property()
    : m_cc(stapl::index_bounds<color_type>::invalid()) {}

  color_type get_cc() const { return m_cc; }
  void set_cc(color_type c) { m_cc = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_cc);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref sssp().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class sssp_property
{
private:
  typedef size_t VD;
  VD     m_vd;
  double m_distance;
  bool   m_active;
public:
  typedef VD     parent_type;
  typedef double dist_type;

  inline VD parent(void) const
  { return m_vd; }

  inline dist_type distance(void) const
  { return m_distance; }

  inline void parent(VD const& vd)
  { m_vd = vd; }

  inline void distance(dist_type const& c)
  { m_distance = c; }

  inline bool is_active(void) const
  { return m_active; }

  inline void set_active(bool c)
  { m_active = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_vd);
    t.member(m_distance);
    t.member(m_active);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref topological_sort().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class topological_sort_property
{
public:
  typedef int vertex_value_type;

private:
  vertex_value_type m_rank;
  vertex_value_type m_preds;

public:
  topological_sort_property(int i = 0)
    : m_rank(i), m_preds(i)
  { }

  topological_sort_property(topological_sort_property const& other) = default;

  vertex_value_type rank(void) const
  { return m_rank; }

  vertex_value_type preds(void) const
  { return m_preds; }

  void rank(vertex_value_type const& c)
  { m_rank = c; }

  void preds(vertex_value_type const& c)
  { m_preds = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_rank);
    t.member(m_preds);
  }

  friend std::ostream& operator<<(std::ostream& stream,
                                  topological_sort_property const& v)
  {
    stream << "Rank (" << v.rank() << "), preds (" << v.preds() << ")";
    return stream;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref triangle_count().
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
class triangle_count_property
{
public:
  typedef std::vector<size_t> queue_t;

private:
  size_t m_num_triangles;
  size_t m_num_connected_triplets;
  queue_t m_waiting_queue;

public:
  triangle_count_property()
    : m_num_triangles(0),
      m_num_connected_triplets(0)
  { }

  size_t num_triangles(void) const
  { return m_num_triangles; }

  void num_triangles(size_t const& t)
  { m_num_triangles = t; }

  size_t num_connected_triplets(void) const
  { return m_num_connected_triplets; }

  void num_connected_triplets(size_t const& t)
  { m_num_connected_triplets = t; }

  queue_t waiting_queue(void) const
  { return m_waiting_queue; }

  void waiting_queue_add(size_t const& t)
  { return m_waiting_queue.push_back(t); }

  bool waiting_queue_empty(void) const
  { return m_waiting_queue.empty(); }

  void waiting_queue_clear(void)
  { return m_waiting_queue.clear(); }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_triangles);
    t.member(m_num_connected_triplets);
    t.member(m_waiting_queue);
  }
};

std::ostream& operator<<(std::ostream& os, triangle_count_property const& p);

} //namespace properties


// Proxies.
template <typename Accessor>
class proxy<properties::preflowpush_vertex_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef properties::preflowpush_vertex_property   target_t;
  typedef target_t::vd_type                         vd_type;
  typedef target_t::message_type                    message_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {return Accessor::read();}
  proxy const& operator=(proxy const& rhs)
  {Accessor::write(rhs); return *this;}
  proxy const& operator=(target_t const& rhs)
  {Accessor::write(rhs); return *this;}

  void height(int value)
  {Accessor::invoke(&target_t::height, value);}
  void excess(double value)
  {Accessor::invoke(&target_t::excess, value);}

  int height() const
  { return Accessor::const_invoke(&target_t::height); }

  void inc_height()
  { Accessor::invoke(&target_t::inc_height); }

  double excess() const
  { return Accessor::const_invoke(&target_t::excess); }

  void inc_ex(double value)
  { Accessor::invoke(&target_t::inc_ex, value) ; }

  void dec_ex(double value)
  { Accessor::invoke(&target_t::dec_ex, value) ; }

  void add_msg(message_type msg)
  { Accessor::invoke(&target_t::add_msg, msg); }

  void pop_msg()
  { Accessor::invoke(&target_t::pop_msg); }

  message_type get_msg() const
  { return Accessor::invoke(&target_t::get_msg); }

  std::size_t msgs()
  { return Accessor::invoke(&target_t::msgs); }

  int stop() const
  { return Accessor::const_invoke(&target_t::stop); }

  void stop(int edge)
  { Accessor::invoke(&target_t::stop, edge); }

}; // class preflow_push_vertex_property proxy

template <typename Accessor>
class proxy<properties::message, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::message                                         target_t;
  typedef typename target_t::vd_type                                   vd_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {return Accessor::read();}
  proxy const& operator=(proxy const& rhs)
  {Accessor::write(rhs); return *this;}
  proxy const& operator=(target_t const& rhs)
  {Accessor::write(rhs); return *this;}

  bool response() const
  { return Accessor::const_invoke(&target_t::response); }
  double flow() const
  { return Accessor::const_invoke(&target_t::flow); }

  vd_type sender_vd() const
  { return  Accessor::const_invoke(&target_t::sender_vd); }

  vd_type receiver_vd() const
  { return  Accessor::const_invoke(&target_t::receiver_vd); }

  int height() const
  { return Accessor::const_invoke(&target_t::height); }

  void response(bool response)
  { Accessor::invoke(&target_t::response, response); }
  void flow(double units)
  { Accessor::invoke(&target_t::flow, units); }

  void sender_vd(vd_type vd_value)
  { Accessor::invoke(&target_t::sender_vd, vd_value); }

  void receiver_vd(vd_type vd_value)
  { Accessor::invoke(&target_t::receiver_vd, vd_value); }

  void height(int height)
  { Accessor::invoke(&target_t::height, height);}


}; // class message proxy

template <class Accessor>
class proxy<properties::preflowpush_edge_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef properties::preflowpush_edge_property target_t;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {return Accessor::read();}
  proxy const& operator=(proxy const& rhs)
  {Accessor::write(rhs); return *this;}
  proxy const& operator=(target_t const& rhs)
  {Accessor::write(rhs); return *this;}

  bool backward() const
  { return Accessor::const_invoke(&target_t::backward); }

  double rc() const
  { return Accessor::const_invoke(&target_t::rc); }

  void rc(double value)
  { Accessor::invoke(&target_t::rc, value); }

  void inc_rc(double value)
  { Accessor::invoke(&target_t::inc_rc, value); }

  void dec_rc(double value)
  { Accessor::invoke(&target_t::dec_rc, value); }

  int height() const
  { return Accessor::const_invoke(&target_t::height); }

  void height(int height)
  { Accessor::invoke(&target_t::height, height); }

}; // class preflow_push_edge_property proxy


template <typename Accessor>
class proxy<properties::bad_rank_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::bad_rank_property target_t;
  typedef target_t::value_type value_type;

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

  value_type rank(void) const
  {
    return Accessor::const_invoke(&target_t::rank);
  }

  void rank(value_type val)
  {
    Accessor::invoke(&target_t::rank, val);
  }

  value_type new_rank(void) const
  {
    return Accessor::const_invoke(&target_t::new_rank);
  }

  void new_rank(value_type val)
  {
    Accessor::invoke(&target_t::new_rank, val);
  }

  size_t is_active(void) const
  {
    return Accessor::const_invoke(&target_t::is_active);
  }

  void set_active(bool val)
  {
    Accessor::invoke(&target_t::set_active, val);
  }

  size_t is_blacklisted(void) const
  {
    return Accessor::const_invoke(&target_t::is_blacklisted);
  }

  void set_blacklisted(bool val)
  {
    Accessor::invoke(&target_t::set_blacklisted, val);
  }
}; //class bad_rank_property proxy


template <typename Accessor>
class proxy<properties::bc_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::bc_property target_t;
public:
  typedef typename target_t::vd_type      vd_type;
  typedef typename target_t::dist_type    dist_type;
  typedef typename target_t::sigma_type   sigma_type;
  typedef typename target_t::delta_type   delta_type;
  typedef typename target_t::bfs_dag_type bfs_dag_type;
  typedef typename target_t::level_type   level_type;

  typedef typename target_t::distances_t  distances_t;
  typedef typename target_t::sigmas_t     sigmas_t;
  typedef typename target_t::distances_t  deltas_t;
  typedef typename target_t::bfs_dags_t   bfs_dags_t;
  typedef typename target_t::active_t     active_t;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this; }

  inline void initialize(size_t num_sources)
  { Accessor::invoke(&target_t::initialize, num_sources); }

  inline void reset_sources(void)
  { Accessor::invoke(&target_t::reset_sources); }

  inline delta_type BC(void) const
  { return Accessor::const_invoke(&target_t::BC); }

  inline void BC(delta_type bc)
  { Accessor::invoke(&target_t::BC, bc); }

  inline dist_type distance(vd_type source_id) const
  { return Accessor::const_invoke(&target_t::distance, source_id); }

  inline void distance(vd_type source_id, dist_type dist)
  { Accessor::invoke(&target_t::distance, source_id, dist); }

  inline delta_type delta(vd_type source_id) const
  { return Accessor::const_invoke(&target_t::delta, source_id); }

  inline void delta(vd_type source_id, delta_type delta)
  { Accessor::invoke(&target_t::delta, source_id, delta); }

  inline sigma_type sigma(vd_type source_id) const
  { return Accessor::const_invoke(&target_t::sigma, source_id); }

  inline void sigma(vd_type source_id, sigma_type value)
  { Accessor::invoke(&target_t::sigma, source_id, value); }

  inline const bfs_dag_type bfs_dag(vd_type source_id) const
  { return Accessor::const_invoke(&target_t::bfs_dag, source_id); }

  inline void bfs_dag(vd_type source_id, vd_type p_id, sigma_type sig)
  { Accessor::invoke(&target_t::bfs_dag, source_id, p_id, sig); }

  inline const level_type level_sources(dist_type level) const
  { return Accessor::const_invoke(&target_t::level_sources, level); }

  inline vd_type next_active_traversal(void)
  { return Accessor::invoke(&target_t::next_active_traversal); }

  inline bool is_active(void) const
  { return Accessor::const_invoke(&target_t::is_active); }

  inline void set_active(vd_type source_id)
  { Accessor::invoke(&target_t::set_pending, source_id); }

  inline void set_pending(vd_type source_id)
  { Accessor::invoke(&target_t::set_pending, source_id); }

  inline void push_pending(void)
  { Accessor::invoke(&target_t::push_pending); }
}; //class bc_property proxy


template <class Accessor>
class proxy<properties::bfs_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::bfs_property target_t;

public:
  typedef size_t vd_type;
  typedef size_t VD;
  typedef size_t Level;
  typedef size_t value_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  VD parent(void) const
  {
    VD (target_t::*pmf)(void) const = &target_t::parent;
    return Accessor::const_invoke(pmf);
  }

  void parent(VD const& vd)
  {
    void (target_t::*pmf)(VD const&) = &target_t::parent;
    Accessor::invoke(pmf, vd);
  }

  Level level(void) const
  {
    Level (target_t::*pmf)(void) const = &target_t::level;
    return Accessor::const_invoke(pmf);
  }

  void level(Level const& c)
  {
    void (target_t::*pmf)(Level const&) = &target_t::level;
    Accessor::invoke(pmf, c);
  }
}; //class bfs_property proxy


template <class Accessor>
class proxy<properties::approximate_bfs_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::approximate_bfs_property target_t;

public:
  typedef size_t vd_type;
  typedef size_t VD;
  typedef size_t Level;
  typedef size_t value_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  VD parent(void) const
  {
    VD (target_t::*pmf)(void) const = &target_t::parent;
    return Accessor::const_invoke(pmf);
  }

  void parent(VD const& vd)
  {
    void (target_t::*pmf)(VD const&) = &target_t::parent;
    Accessor::invoke(pmf, vd);
  }

  Level level(void) const
  {
    Level (target_t::*pmf)(void) const = &target_t::level;
    return Accessor::const_invoke(pmf);
  }

  void level(Level const& c)
  {
    void (target_t::*pmf)(Level const&) = &target_t::level;
    Accessor::invoke(pmf, c);
  }

  Level propagated_level(void) const
  {
    Level (target_t::*pmf)(void) const = &target_t::propagated_level;
    return Accessor::const_invoke(pmf);
  }

  void propagated_level(Level const& c)
  {
    void (target_t::*pmf)(Level const&) = &target_t::propagated_level;
    Accessor::invoke(pmf, c);
  }

  bool active(void) const
  {
    bool (target_t::*pmf)(void) const = &target_t::active;
    return Accessor::const_invoke(pmf);
  }

  void active(bool c)
  {
    void (target_t::*pmf)(bool) = &target_t::active;
    Accessor::invoke(pmf, c);
  }
}; //class approximate_bfs_property proxy


template <class Accessor>
class proxy<properties::closeness_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::closeness_property target_t;

public:
  typedef size_t vd_type;
  typedef size_t VD;
  typedef size_t Level;
  typedef size_t value_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  VD parent(void) const
  {
    VD (target_t::*pmf)(void) const = &target_t::parent;
    return Accessor::const_invoke(pmf);
  }

  void parent(VD const& vd)
  {
    void (target_t::*pmf)(VD const&) = &target_t::parent;
    Accessor::invoke(pmf, vd);
  }

  Level level(void) const
  {
    Level (target_t::*pmf)(void) const = &target_t::level;
    return Accessor::const_invoke(pmf);
  }

  void level(Level const& c)
  {
    void (target_t::*pmf)(Level const&) = &target_t::level;
    Accessor::invoke(pmf, c);
  }

  double closeness(void) const
  {
    double (target_t::*pmf)(void) const = &target_t::closeness;
    return Accessor::const_invoke(pmf);
  }

  void closeness(double const& d)
  {
    void (target_t::*pmf)(double const&) = &target_t::closeness;
    Accessor::invoke(pmf, d);
  }
}; //class closeness_property proxy


template <typename Accessor>
class proxy<properties::coloring_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::coloring_property target_t;

public:
  typedef size_t value_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
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

  size_t get_color() const
  {
    return Accessor::const_invoke(&target_t::get_color);
  }

  void set_color(size_t const& c)
  {
    Accessor::invoke(&target_t::set_color, c);
  }
}; //class coloring_property proxy


template <typename Accessor, typename VertDesc, typename Weight>
class proxy<properties::boruvka_edge_property<VertDesc, Weight> , Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::boruvka_edge_property<VertDesc,Weight> target_t;

public:
  typedef size_t value_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  VertDesc source() const
  { return Accessor::const_invoke(&target_t::source); }

  void source(VertDesc const& s)
  { Accessor::invoke(&target_t::source, s); }

  VertDesc target() const
  { return Accessor::const_invoke(&target_t::target); }

  void target(VertDesc const& t)
  { Accessor::invoke(&target_t::target, t); }

  Weight weight() const
  { return Accessor::const_invoke(&target_t::weight); }

  void weight(Weight const& p)
  { Accessor::invoke(&target_t::weight, p); }
}; //class boruvka_edge_property proxy


template <typename Accessor>
class proxy<properties::cc_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::cc_property target_t;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc) { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs) {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs) {
    Accessor::write(rhs);
    return *this;
  }

  void cc(size_t c)
  { Accessor::invoke(&target_t::cc, c); }

  size_t cc(void) const
  { return Accessor::const_invoke(&target_t::cc); }

  void active(bool a)
  { Accessor::invoke(&target_t::active, a); }

  bool active(void) const
  { return Accessor::const_invoke(&target_t::active); }
}; //class cc_property proxy


template <typename Accessor>
class proxy<properties::cd_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::cd_property target_t;

public:
  typedef target_t::label_type label_type;
  typedef target_t::labels_type labels_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc) { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs) {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs) {
    Accessor::write(rhs);
    return *this;
  }

  void label(label_type const& c)
  { Accessor::invoke(&target_t::label, c); }

  label_type label(void) const
  { return Accessor::const_invoke(&target_t::label); }

  labels_type label_vector(void) const
  { return Accessor::const_invoke(&target_t::label_vector); }

  void label_vector_add(label_type const& c)
  { Accessor::invoke(&target_t::label_vector_add, c); }

  void label_vector_clear(void)
  { Accessor::invoke(&target_t::label_vector_clear); }

}; //class cd_property proxy


template <typename VD, typename Accessor>
class proxy<properties::lp_property<VD>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::lp_property<VD> target_t;

public:
  typedef typename target_t::desc_t desc_t;
  typedef typename target_t::link_probabilities_t link_probabilities_t;
  typedef typename target_t::step_2_buffer_t step_2_buffer_t;
  typedef typename target_t::step_3_buffer_t step_3_buffer_t;

  explicit proxy(Accessor const& acc)
    : Accessor(acc) { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs) {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs) {
    Accessor::write(rhs);
    return *this;
  }

  void link_probability_size(size_t const& sz)
  { Accessor::invoke(&target_t::link_probability_size, sz); }

  size_t link_probability_size(void) const
  { return Accessor::const_invoke(&target_t::link_probability_size); }

  link_probabilities_t link_probabilities(void) const
  { return Accessor::const_invoke(&target_t::link_probabilities); }

  void link_probability(size_t const& idx, double const& p)
  { Accessor::invoke(&target_t::link_probability, idx, p); }

  void add_link_probability(size_t const& idx, double const& p)
  { Accessor::invoke(&target_t::add_link_probability, idx, p); }

  double link_probability(size_t const& idx) const
  { return Accessor::const_invoke(&target_t::link_probability, idx); }

  step_2_buffer_t step_2_buffer(void) const
  { return Accessor::const_invoke(&target_t::step_2_buffer); }

  void step_2_buffer_add(desc_t const& src)
  { Accessor::invoke(&target_t::step_2_buffer_add, src); }

  void step_2_buffer_clear(void)
  { Accessor::invoke(&target_t::step_2_buffer_clear); }

  step_3_buffer_t step_3_buffer(void) const
  { return Accessor::const_invoke(&target_t::step_3_buffer); }

  void step_3_buffer_add(desc_t const& tgt, step_2_buffer_t const& srcs)
  { Accessor::invoke(&target_t::step_3_buffer_add, tgt, srcs); }

  void step_3_buffer_clear(void)
  { Accessor::invoke(&target_t::step_3_buffer_clear); }

}; //class lp_property proxy


template <class Accessor>
class proxy<properties::mbm_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::mbm_property target_t;

public:
  typedef target_t::vd_type vd_type;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  properties::bipartite_partition partition(void) const
  { return Accessor::const_invoke(&target_t::partition); }

  void partition(properties::bipartite_partition const& p)
  { Accessor::invoke(&target_t::partition, p); }

  bool matched(void)
  { return Accessor::const_invoke(&target_t::matched); }

  void matched(bool b)
  { Accessor::invoke(&target_t::matched, b); }

  size_t level(void)
  { return Accessor::const_invoke(&target_t::level); }

  void level(size_t b)
  { Accessor::invoke(&target_t::level, b); }

  vd_type match(void)
  { return Accessor::const_invoke(&target_t::match); }

  void match(vd_type const& v)
  { Accessor::invoke(&target_t::match, v); }

}; //class mbm_property proxy


template <class Accessor>
class proxy<properties::mssp_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::mssp_property target_t;

public:
  typedef size_t   vd_type;
  typedef typename target_t::dist_type   dist_type;
  typedef typename target_t::distances_t distances_t;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline vd_type parent(vd_type const& traversal_id)
  { return Accessor::invoke(&target_t::parent, traversal_id); }

  inline dist_type distance(vd_type const& traversal_id)
  {
    dist_type (target_t::*pmf)(vd_type const&) = &target_t::distance;
    return Accessor::invoke(pmf, traversal_id);
  }

  inline distances_t distances(void) const
  { return Accessor::const_invoke(&target_t::distances); }

  inline void  parent(vd_type const& traversal_id, vd_type const& vd)
  { Accessor::invoke(&target_t::parent, traversal_id, vd); }

  inline void  distance(vd_type const& traversal_id, dist_type const& c)
  { Accessor::invoke(&target_t::distance, traversal_id, c); }

  inline bool is_active(void) const
  { return Accessor::const_invoke(&target_t::is_active); }

  inline vd_type next_active_traversal(void) const
  { return Accessor::const_invoke(&target_t::next_active_traversal); }

  inline void set_active(vd_type const& traversal_id)
  { Accessor::invoke(&target_t::set_active, traversal_id); }

  inline void set_inactive(vd_type const& traversal_id)
  { Accessor::invoke(&target_t::set_inactive, traversal_id); }

}; //class mssp_property proxy


template<typename Rank, typename Accessor>
class proxy<properties::page_rank_property<Rank>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::page_rank_property<Rank> target_t;
  typedef typename target_t::value_type value_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this; }

  value_type rank(void) const
  { return Accessor::const_invoke(&target_t::rank); }

  void rank(value_type val)
  { Accessor::invoke(&target_t::rank, val); }

  value_type new_rank(void) const
  { return Accessor::const_invoke(&target_t::new_rank); }

  void new_rank(value_type val)
  { Accessor::invoke(&target_t::new_rank, val); }

}; //class page_rank_property proxy


template <class Accessor>
class proxy<properties::rw_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::rw_property target_t;

public:
  typedef size_t Level;
  typedef size_t value_type;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline size_t visits(void) const
  {
    size_t (target_t::*pmf)(void) const = &target_t::visits;
    return Accessor::const_invoke(pmf);
  }

  inline void  visits(size_t const& c)
  { Accessor::invoke(&target_t::visits, c); }

  inline Level level(void) const
  {
    Level (target_t::*pmf)(void) const = &target_t::level;
    return Accessor::const_invoke(pmf);
  }

  inline void  level(Level const& c)
  { Accessor::invoke(&target_t::level, c); }
}; //class rw_property proxy


template <typename VertGID, typename Accessor>
class proxy<properties::scc_property<VertGID>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef properties::scc_property<VertGID>      target_t;
  typedef typename target_t::color_type           color_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc) { }

  operator target_t() const
  { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this; }

  void set_cc(color_type c)
  { Accessor::invoke(&target_t::set_cc, c); }
  color_type get_cc() const
  { return Accessor::const_invoke(&target_t::get_cc); }
}; //class scc_propertyproxy


template <class Accessor>
class proxy<properties::sssp_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::sssp_property target_t;

public:
  typedef target_t::parent_type   parent_type;
  typedef typename target_t::dist_type dist_type;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline parent_type parent(void) const
  { return Accessor::const_invoke(&target_t::parent); }

  inline dist_type distance(void) const
  { return Accessor::const_invoke(&target_t::distance); }

  inline void  parent(parent_type const& vd)
  { Accessor::invoke(&target_t::parent, vd); }

  inline void  distance(dist_type const& c)
  { Accessor::invoke(&target_t::distance, c); }

  inline bool is_active(void) const
  { return Accessor::const_invoke(&target_t::is_active); }

  inline void set_active(bool c)
  { Accessor::invoke(&target_t::set_active, c); }

}; //class sssp_property proxy


template <typename Accessor>
class proxy<properties::topological_sort_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef properties::topological_sort_property target_t;

  typedef target_t::vertex_value_type vertex_value_type;

public:
  typedef int value_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

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

  int rank(void) const
  { return Accessor::const_invoke(&target_t::rank); }

  int preds(void) const
  { return Accessor::const_invoke(&target_t::preds); }

  void rank(int const& c)
  { Accessor::invoke(&target_t::rank, c); }

  void preds(int const& c)
  { Accessor::invoke(&target_t::preds, c); }

  friend std::ostream& operator<<(std::ostream& stream,
                                  proxy<properties::topological_sort_property,
                                        Accessor> const& v)
  {
    stream << "Rank (" << v.rank() << "), preds (" << v.preds() << ")";
    return stream;
  }
}; //class topological_sort_property proxy


template <class Accessor>
class proxy<properties::triangle_count_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::triangle_count_property target_t;

public:
  typedef typename target_t::queue_t queue_t;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline size_t num_triangles(void) const
  { return Accessor::const_invoke(&target_t::num_triangles); }

  inline void num_triangles(size_t const& t)
  { Accessor::invoke(&target_t::num_triangles, t); }

  inline size_t num_connected_triplets(void) const
  { return Accessor::const_invoke(&target_t::num_connected_triplets); }

  inline void num_connected_triplets(size_t const& t)
  { Accessor::invoke(&target_t::num_connected_triplets, t); }

  inline queue_t waiting_queue(void) const
  { return Accessor::const_invoke(&target_t::waiting_queue); }

  inline bool waiting_queue_empty(void) const
  { return Accessor::const_invoke(&target_t::waiting_queue_empty); }

  inline void waiting_queue_add(size_t const& t)
  { return Accessor::invoke(&target_t::waiting_queue_add, t); }

  inline void waiting_queue_clear(void)
  { return Accessor::invoke(&target_t::waiting_queue_clear); }
}; //class triangle_count_property proxy

} //namespace stapl


#endif
