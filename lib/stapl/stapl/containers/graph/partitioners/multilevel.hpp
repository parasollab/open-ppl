/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/native_view.hpp>
#include <vector>
#include <cmath>
#include "multilevel_utility.hpp"
#include "multilevel_coarsener.hpp"
#include "multilevel_initial_partitioner.hpp"
#include "multilevel_refiner.hpp"

namespace stapl {

namespace partitioner_details {

//////////////////////////////////////////////////////////////////////
/// @brief vertex property for multilevel graph partitioner.
//////////////////////////////////////////////////////////////////////
class multilevel_vertex_property
{
 private:
  size_t m_weight;
  size_t m_partition_id;
  size_t m_matched_vertex;
  char   m_matched;
  std::vector<std::pair<size_t,size_t> > m_edges;
  std::pair<size_t, size_t> m_degree;
 public:
  enum {UNMATCHED, MAYBE_MATCHED, MATCHED};

  //////////////////////////////////////////////////////////////////////
  /// @param weight vertex weight.
  //////////////////////////////////////////////////////////////////////
  multilevel_vertex_property(size_t const& weight = 0)
    : m_weight(weight), m_partition_id(0), m_matched_vertex(0),
      m_matched(UNMATCHED),
       m_edges(), m_degree()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Addition operator with a vertex property. Sums vertex weights of
  ///        both vertex properties and returns a new vertex property
  ///        constructed with the vertex weight sum.
  /// @param rhs vertex property whose vertex weight is added to the
  ///        vertex weight of this property.
  /// @return new vertex property constructed with the vertex weight sum.
  //////////////////////////////////////////////////////////////////////
  multilevel_vertex_property
  operator+(multilevel_vertex_property const& rhs) const
  { return rhs.m_weight + this->m_weight; }

  size_t get_weight() const
  { return m_weight; }

  size_t get_partition_id() const
  { return m_partition_id; }

  size_t get_matched_vertex() const
  { return m_matched_vertex; }

  std::vector<std::pair<size_t, size_t> > get_partition_info() const
  {
    return m_edges;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compares the gain of two partitions. Returns true if the first
  ///        partition has a higher gain.
  //////////////////////////////////////////////////////////////////////
  struct gain_comparator
  {
    //////////////////////////////////////////////////////////////////////
    /// @param i first pair of partition id and gain.
    /// @param i second pair of partition id and gain.
    /// @return true if the first partition has a higher gain.
    //////////////////////////////////////////////////////////////////////
    bool operator()(std::pair<size_t, int> const& i,
                    std::pair<size_t, int> const& j) const
    {
      return (i.second > j.second);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Sort neighboring partitions by decreasing gain.
  //////////////////////////////////////////////////////////////////////
  void sort_partition_info()
  {
    std::sort(m_edges.begin(), m_edges.end(), gain_comparator());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if vertex is matched.
  //////////////////////////////////////////////////////////////////////
  bool is_matched() const
  {
    return (m_matched == MATCHED);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if vertex is maybe matched.
  //////////////////////////////////////////////////////////////////////
  bool is_maybe_matched() const
  {
    return (m_matched == MAYBE_MATCHED);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if vertex is unmatched.
  //////////////////////////////////////////////////////////////////////
  bool is_unmatched() const
  {
    return (m_matched == UNMATCHED);
  }

  size_t get_internal_degree() const
  {
    return m_degree.first;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the move of this vertex to another partition
  ///        will increase the edge-cut.
  //////////////////////////////////////////////////////////////////////
  bool cannot_move() const
  {
    return (m_degree.first > m_degree.second);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Change a MAYBE_MATCHED status to UNMATCHED.
  //////////////////////////////////////////////////////////////////////
  void clear_maybe_matched()
  {
    if (m_matched == MAYBE_MATCHED)
      m_matched = UNMATCHED;
  }

  void set_weight(size_t const& w)
  {
    m_weight = w;
  }

  void set_partition_id(size_t const& pid)
  {
    m_partition_id = pid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the partition id of the vertex.
  /// @param pid new partition id.
  //////////////////////////////////////////////////////////////////////
  void update_partition_id(size_t const& pid)
  {
    std::vector<std::pair<size_t, size_t> >::iterator
      it = std::find_if(m_edges.begin(), m_edges.end(), pid_comp(pid));
    //update outdegree
    m_degree.second -= (it->second - m_degree.first);
    //update internal degree
    m_degree.first = it->second;

    //update partion_id
    m_partition_id = pid;
  }

  void set_matched_vertex(size_t const& m)
  {
    m_matched_vertex = m;
  }

  void set_matched(char const& m)
  {
    m_matched = m;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the matching status and the matched vertex.
  /// @param m matching status.
  /// @param id matched vertex id.
  //////////////////////////////////////////////////////////////////////
  void set_matched(char const& m, size_t const& id)
  {
    m_matched = m;
    m_matched_vertex = id;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compares id of partition with a specific partition id.
  //////////////////////////////////////////////////////////////////////
  struct pid_comp
  {
  private:
    size_t m_pid;

  public:
    //////////////////////////////////////////////////////////////////////
    /// @param pid partition id used for equality check.
    //////////////////////////////////////////////////////////////////////
    pid_comp(size_t const& pid)
      : m_pid(pid)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @param p pair of partition id and partition weight.
    /// @return true if pair has the same partition id as @p m_pid.
    //////////////////////////////////////////////////////////////////////
    bool operator()(std::pair<size_t, size_t> const& p)
    {
      return (p.first == m_pid);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Add neighboring partition id and weight to the vertex.
  /// @param p pair of partition id and weight.
  //////////////////////////////////////////////////////////////////////
  void add_partition(std::pair<size_t, size_t> const& p)
  {
    std::vector<std::pair<size_t, size_t> >::iterator
      it = std::find_if(m_edges.begin(), m_edges.end(), pid_comp(p.first));
    if (it != m_edges.end())
      it->second += p.second;
    else
      m_edges.push_back(p);

    //update degree
    if (p.first == m_partition_id)
      m_degree.first += p.second;
    else
      m_degree.second += p.second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor updating neighboring partition information of a vertex.
  /// @param old_pid id of source partition whose weight is subtracted
  ///        with @p weight.
  /// @param new_pid id of target partition whose weight is incremented
  ///        with @p weight.
  /// @param weight weight to be moved.
  //////////////////////////////////////////////////////////////////////
  void update_partition(size_t const& old_pid, size_t const& new_pid,
                        size_t const& weight)
  {
    std::vector<std::pair<size_t, size_t> >::iterator
      it = std::find_if(m_edges.begin(), m_edges.end(), pid_comp(old_pid));
    stapl_assert(it->second >= weight,
      "ERROR in partition refinement: update_partition()");
    it->second -= weight;
    it = std::find_if(m_edges.begin(), m_edges.end(), pid_comp(new_pid));
    if (it != m_edges.end())
      it->second += weight;
    else
      m_edges.push_back(std::make_pair(new_pid, weight));

    //update degrees
    if (old_pid == m_partition_id)
      m_degree.first -= weight;
    else
      m_degree.second -= weight;
    if (new_pid == m_partition_id)
      m_degree.first += weight;
    else
      m_degree.second += weight;
  }

  void define_type(typer& t)
  {
    t.member(m_weight);
    t.member(m_partition_id);
    t.member(m_matched_vertex);
    t.member(m_matched);
    t.member(m_edges);
    t.member(m_degree);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Print property of this vertex to an output stream.
  /// @param stream output stream.
  /// @param v vertex property.
  /// @return the output stream @p stream.
  //////////////////////////////////////////////////////////////////////
  friend std::ostream& operator<< (std::ostream& stream,
      multilevel_vertex_property const& v)
  {
    stream << "weight=" << v.m_weight << ", "
           << "partition_id=" << v.m_partition_id << ", "
           << "matched_vertex=" << v.m_matched_vertex << ", "
           << "m_matched=" << v.m_matched;
    return stream;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief vertex property for multilevel graph partitioner.
//////////////////////////////////////////////////////////////////////
class multilevel_edge_property
{
public:
  typedef size_t weight_type;
  weight_type weight;
  weight_type adjusted_weight;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor initializing the edge weight and default constructing
  /// the adjusted weight.
  /// @param w edge weight.
  //////////////////////////////////////////////////////////////////////
  multilevel_edge_property(weight_type const& w = 0)
    : weight(w), adjusted_weight()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Addition of this multilevel_edge_property with another one.
  /// @param other right-hand side of the addition.
  /// @return multilevel_edge_property whose weight is the sum of both
  /// multilevel_edge_property objects.
  //////////////////////////////////////////////////////////////////////
  multilevel_edge_property
  operator+(multilevel_edge_property const& other) const
  {
    return weight + other.weight;
  }

  void define_type(typer& t)
  {
    t.member(weight);
    t.member(adjusted_weight);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to copy a vertex from a weighted graph to a graph with
///        the multilevel property.
/// @tparam VertexWeightMap vertex weight property map type.
//////////////////////////////////////////////////////////////////////
template <typename VertexWeightMap>
struct build_vertex_hierarchical_graph_wf
{
private:
  VertexWeightMap m_vertex_weight_map;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param vertex_weight_map vertex weight property map.
  //////////////////////////////////////////////////////////////////////
  build_vertex_hierarchical_graph_wf(VertexWeightMap const& vertex_weight_map)
    : m_vertex_weight_map(vertex_weight_map)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex copied.
  /// @param hgraph graph with multilevel property.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex,typename HGraph>
  void operator()(Vertex v, HGraph hgraph)
  {
    hgraph[v.descriptor()].property().property.set_weight(
      m_vertex_weight_map.get(v));
  }

  void define_type(typer& t)
  {
    t.member(m_vertex_weight_map);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to copy the edges of a weighted graph to a graph with
///        the multilevel property.
/// @tparam EdgeWeightMap edge weight property map type.
//////////////////////////////////////////////////////////////////////
template <typename EdgeWeightMap>
struct build_edge_hierarchical_graph_wf
{
private:
  EdgeWeightMap m_edge_weight_map;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param edge_weight_map edge weight property map.
  //////////////////////////////////////////////////////////////////////
  build_edge_hierarchical_graph_wf(EdgeWeightMap const& edge_weight_map)
    : m_edge_weight_map(edge_weight_map)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex whose edges are copied.
  /// @param hgraph graph with multilevel property.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex,typename HGraph>
  void operator()(Vertex v, HGraph hgraph)
  {
    typedef typename HGraph::edge_descriptor edge_descriptor;
    typedef typename HGraph::edge_property::property_type edge_property;
    typename Vertex::adj_edge_iterator edge_it = v.begin(),
                                       edge_end_it = v.end();
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      if ((*edge_it).source() != (*edge_it).target())
      {
        hgraph.add_edge_async(edge_descriptor((*edge_it).source(),
                                              (*edge_it).target()),
                              edge_property(m_edge_weight_map.get(*edge_it)));
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_edge_weight_map);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Extract partition domains from vertices.
//////////////////////////////////////////////////////////////////////
struct get_partition_doms
{
  typedef std::vector<domset1D<size_t> > result_type;
private:
  size_t m_num_partitions;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param num_partitions number of partitions.
  //////////////////////////////////////////////////////////////////////
  get_partition_doms(size_t const& num_partitions)
    : m_num_partitions(num_partitions)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param part graph base container.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition>
  result_type operator()(Partition part) const
  {
    result_type part_doms(m_num_partitions);
    typename Partition::vertex_iterator it = part.begin(),
                                        end_it = part.end();
    for (; it!=end_it; ++it)
    {
      part_doms[(*it).property().property.get_partition_id()]
        += (*it).descriptor();
    }
    return part_doms;
  }

  void define_type(typer& t)
  {
    t.member(m_num_partitions);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to set partition domains in the hierarchical view.
//////////////////////////////////////////////////////////////////////
struct set_partition_domain_wf
{
  typedef std::vector<domset1D<size_t> > dom_list_t;
private:
  dom_list_t m_dom_list;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param dom_list partition domains.
  //////////////////////////////////////////////////////////////////////
  set_partition_domain_wf(dom_list_t const& dom_list)
    : m_dom_list(dom_list)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param p super-vertex of hierarchical view.
  /// @param partition index for super-vertex @p p.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition>
  void operator()(Partition p, size_t const& index)
  {
    p.property().set_domain(m_dom_list[index]);
  }

  void define_type(typer& t)
  {
    t.member(m_dom_list);
  }
};

} // namespace partitioner_details


template <typename Accessor>
class proxy<partitioner_details::multilevel_vertex_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef partitioner_details::multilevel_vertex_property target_t;

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

  size_t get_weight() const
  { return Accessor::const_invoke(&target_t::get_weight); }

  size_t get_partition_id() const
  { return Accessor::const_invoke(&target_t::get_partition_id); }

  size_t get_matched_vertex() const
  { return Accessor::const_invoke(&target_t::get_matched_vertex); }

  std::vector<std::pair<size_t, size_t> > get_partition_info() const
  { return Accessor::const_invoke(&target_t::get_partition_info); }

  void sort_partition_info()
  { Accessor::invoke(&target_t::sort_partition_info); }

  bool is_matched() const
  { return Accessor::const_invoke(&target_t::is_matched); }

  bool is_maybe_matched() const
  { return Accessor::const_invoke(&target_t::is_maybe_matched); }

  bool is_unmatched() const
  { return Accessor::const_invoke(&target_t::is_unmatched); }

  bool cannot_move() const
  { return Accessor::const_invoke(&target_t::cannot_move); }

  size_t get_internal_degree() const
  { return Accessor::const_invoke(&target_t::get_internal_degree); }

  void set_weight(size_t const& w)
  { Accessor::invoke(&target_t::set_weight, w); }

  void set_partition_id(size_t const& pid)
  { Accessor::invoke(&target_t::set_partition_id, pid); }

  void update_partition_id(size_t const& pid)
  { Accessor::invoke(&target_t::update_partition_id, pid); }

  void set_matched_vertex(size_t const& m)
  { Accessor::invoke(&target_t::set_matched_vertex, m); }

  void set_matched(char const& m)
  { Accessor::invoke(&target_t::set_matched, m); }

  void set_matched(char const& m, size_t const& id)
  {
    typedef void (target_t::* fn_t)(char const&, size_t const&);
    Accessor::invoke( (fn_t) &target_t::set_matched, m, id);
  }

  void clear_maybe_matched()
  { Accessor::invoke(&target_t::clear_maybe_matched); }

  void add_partition(std::pair<size_t, size_t> const& p)
  { Accessor::invoke(&target_t::add_partition, p); }

  void update_partition(size_t const& old_pid, size_t const& new_pid,
                        size_t const& weight)
  { Accessor::invoke(&target_t::update_partition, old_pid, new_pid, weight); }

  friend std::ostream& operator<< (std::ostream& stream,
                                   proxy<partitioner_details::
                                           multilevel_vertex_property,
                                         Accessor> const& v)
  {
    stream << "weight=" << v.get_weight() << ", "
           << "partition_id=" << v.get_partition_id() << ", "
           << "matched_vertex=" << v.get_matched_vertex() << ", "
           << "m_matched=" << v.is_matched();
    return stream;
  }
}; //struct proxy


template <class Accessor>
class proxy<partitioner_details::multilevel_edge_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef partitioner_details::multilevel_edge_property target_t;
  typedef target_t::weight_type weight_type;

public:
  STAPL_PROXY_MEMBER(weight, weight_type);
  STAPL_PROXY_MEMBER(adjusted_weight, weight_type);

  explicit proxy(Accessor const& acc)
    : Accessor(acc), weight(acc), adjusted_weight(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}
};


//////////////////////////////////////////////////////////////////////
/// @brief Parallel k-way multilevel graph partitioner partitioning a
///        graph into k pieces using the multilevel scheme.
/// @tparam VertexWeightMap vertex weight property map type.
/// @tparam EdgeWeightMap edge weight property map type.
//////////////////////////////////////////////////////////////////////
template <typename VertexWeightMap, typename EdgeWeightMap>
struct k_way_multilevel
{
private:
  VertexWeightMap m_vertex_weight_map;
  EdgeWeightMap   m_edge_weight_map;
  size_t          m_num_partitions;
  size_t          m_imbalance;
  size_t          m_coarsening_factor;
  size_t          m_pass_fm;

public:

  //////////////////////////////////////////////////////////////////////
  /// @param vertex_weight_map vertex weight property map.
  /// @param edge_weight_map edge weight property map.
  /// @param num_parts number of partitions.
  /// @param imbalance imbalance allowed to minimize partition edge-cut.
  /// @param coarsening_factor average number of vertices per partition when
  ///        coarsening phase stops.
  //////////////////////////////////////////////////////////////////////
  k_way_multilevel(VertexWeightMap const& vertex_weight_map,
                   EdgeWeightMap const& edge_weight_map,
                   size_t const& num_parts, size_t const& imbalance,
                   size_t const& coarsening_factor, size_t const& pass_fm)
    : m_vertex_weight_map(vertex_weight_map),
      m_edge_weight_map(edge_weight_map),
      m_num_partitions(num_parts),
      m_imbalance(imbalance),
      m_coarsening_factor(coarsening_factor),
      m_pass_fm(pass_fm)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param graph_vw graph view to be partitioned.
  //////////////////////////////////////////////////////////////////////
  template<class GView>
  gpartition<GView>
  operator()(GView const& graph_vw) const
  {
    using namespace partitioner_details;

    typedef graph<DIRECTED, NONMULTIEDGES,
      super_vertex_property<multilevel_vertex_property>,
      super_edge_property<multilevel_edge_property> > hgraph_type;
    typedef graph_view<hgraph_type> hgraph_view_type;

    std::vector<hgraph_view_type> graph_view_levels;

    size_t original_size = graph_vw.size();
    hgraph_type* original_graph = new hgraph_type(original_size);
    hgraph_view_type original_graph_view = hgraph_view_type(original_graph);


    //Build level 0 of hierarchical graph
    //Build vertices
    map_func(build_vertex_hierarchical_graph_wf<VertexWeightMap>
               (m_vertex_weight_map), graph_vw,
             make_repeat_view(original_graph_view));
    //Build edges
    map_func(build_edge_hierarchical_graph_wf<EdgeWeightMap>(m_edge_weight_map),
             graph_vw, make_repeat_view(original_graph_view));


    /********************
     * Coarsening phase *
     ********************/
    hgraph_view_type next_level = original_graph_view;
    size_t num_levels = 1;
    size_t current_size = next_level.size(), previous_size = 0;

    graph_view_levels.push_back(next_level);

    while ( num_levels == 1 ||
            ((current_size >= m_coarsening_factor*m_num_partitions) &&
             (current_size <= previous_size*0.85)) )
    {
      //Match, collapse vertices and create next level of hierarchy
      next_level = heavy_edge_matching_collapser(next_level, num_levels);
      previous_size = current_size;
      current_size = next_level.size();
      //Push back new level
      graph_view_levels.push_back(next_level);
      ++num_levels;
    }

    /**********************
     * Partitioning phase *
     **********************/
    hgraph_view_type last_level = graph_view_levels.back();

    //Recursive bisection and return total weight
    size_t total_weight =
      recursive_bisection_partition(last_level, m_num_partitions, m_imbalance);

    /**********************
     * Refinement phase   *
     **********************/
    //Calculate optimal partition weight
    size_t opt_part_weight = ceil(total_weight/(double)m_num_partitions);

    //refine last level
    for (int level=num_levels-2; level>=0; --level)
    {
      //propagate_info
      map_func(set_children_pid(), graph_view_levels[level+1],
               make_repeat_view(graph_view_levels[level]));

      //Refine graph level
      fm_refinement(graph_view_levels[level], m_num_partitions, opt_part_weight,
                    m_imbalance, m_pass_fm);
    }

    //Print partition quality
    /* std::pair<double, size_t> part_quality =
      print_partition_quality(original_graph_view, m_num_partitions);
    if (get_location_id() == 0)
      std::cout << "Cut=" <<  part_quality.second
                << " Imbalance=" << part_quality.first << std::endl; */

    //Create hierarchical view on top of the original graph
    typedef typename partition_view_type<GView>::type partition_type;
    partition_type partition_view = create_partition_view(graph_vw,
                                                          m_num_partitions);
    std::vector<domset1D<size_t> >
      partition_doms = map_reduce(get_partition_doms(m_num_partitions),
                                  combine_vector_wf<domset1D<size_t> >(),
                                  native_view(original_graph_view));
    map_func(set_partition_domain_wf(partition_doms), partition_view,
             counting_view<size_t>(m_num_partitions));

    return gpartition<GView>(partition_view);
 }
}; // struct k_way_multilevel

} // namespace stapl

#endif
