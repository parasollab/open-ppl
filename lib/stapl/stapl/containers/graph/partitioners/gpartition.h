/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_GPARTITION_H
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_GPARTITION_H

#include <stapl/runtime.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class representing a graph partition. The class provides
///        queries to retrieve the partitions and measure its quality.
/// @ingroup pgraphPartitioner
/// @tparam GView graph view type of the partitioned graph.
//////////////////////////////////////////////////////////////////////
template<class GView>
class gpartition
{
public:
  typedef GView base_graph_view_t;
  typedef typename partition_view_type<GView>::type partition_view_t;
  typedef typename partition_view_t::vertex_property::domain_type
                                                 vertex_partition_domain;
  typedef typename partition_view_t::edge_property edge_domain;
  typedef typename partition_view_t::vertex_reference vertex_reference;
  typedef typename partition_view_t::vertex_iterator vertex_iterator;
  typedef typename partition_view_t::adj_edge_iterator adj_edge_iterator;
  typedef size_t partition_id;
private:
  /// graph of domains storing the partition
  partition_view_t partition_graph;

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor extracting the domain of a partition
  //////////////////////////////////////////////////////////////////////
  struct get_partition_domain
  {
    typedef vertex_partition_domain result_type;

  public:

    template<typename SuperVertex>
    vertex_partition_domain operator()(SuperVertex& sv) const
    {
      return sv.domain();
    }

  };

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that takes a hierarchical graph view
  ///        representing the partition
  //////////////////////////////////////////////////////////////////////
  gpartition(partition_view_t const& hview)
    : partition_graph(hview)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the hierarchical view representing the partitions
  /// @return hierarchical view
  //////////////////////////////////////////////////////////////////////
  partition_view_t partition() const
  {
    return partition_graph;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the vertex domain of a specific partition
  /// @param pid partition id
  /// @return vertex domain
  //////////////////////////////////////////////////////////////////////
  vertex_partition_domain vertices(partition_id const& pid) const
  {
    return partition_graph.vp_apply(pid, get_partition_domain());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the outgoing cut-edges of a specific partition
  /// @param pid partition id
  /// @return edge domain
  //////////////////////////////////////////////////////////////////////
  edge_domain out_cut_edges(partition_id const& pid) const
  {
    vertex_reference vref=partition_graph[pid];
    adj_edge_iterator it;
    edge_domain dom;
    for (it=vref.begin();it!=vref.end();++it) {
      dom=dom+(*it).property();
    }
    return dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the outgoing cut-edges from one partition to another one
  /// @param pid1 partition id of the source partition
  /// @param pid2 partition id of the target partition
  /// @return edge domain
  //////////////////////////////////////////////////////////////////////
  edge_domain out_cut_edges(partition_id const& pid1,
                            partition_id const& pid2) const
  {
    vertex_reference vref=partition_graph[pid1];
    adj_edge_iterator it;
    for (it=vref.begin();it!=vref.end();++it) {
      if ((*it).target()==pid2)
        return (*it).property();
    }
    return edge_domain();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the cut-edges of a specific partition
  /// @param pid partition id
  /// @return edge domain
  //////////////////////////////////////////////////////////////////////
  edge_domain cut_edges(partition_id const& pid) const
  {
    edge_domain dom=out_cut_edges(pid);
    vertex_iterator it;
    for (partition_id i=0;i<size();++i) {
      dom+=out_cut_edges(i,pid);
    }
    return dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the cut-edges from one partition to another one
  /// @param pid1 partition id of the source partition
  /// @param pid2 partition id of the target partition
  /// @return edge domain
  //////////////////////////////////////////////////////////////////////
  edge_domain cut_edges(partition_id const& pid1,
                        partition_id const& pid2) const
  {
    return out_cut_edges(pid1,pid2)+out_cut_edges(pid2,pid1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of a partition
  /// @param pid partition id
  //////////////////////////////////////////////////////////////////////
  size_t vertex_size(partition_id const& pid) const
  {
    return vertices(pid).size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the cut-edges of a specific partition
  /// @param pid partition id
  //////////////////////////////////////////////////////////////////////
  size_t cut_edge_size(partition_id const& pid) const
  {
    return cut_edges(pid).size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of cut-edges from one partition to another one
  /// @param pid1 partition id of the source partition
  /// @param pid2 partition id of the target partition
  //////////////////////////////////////////////////////////////////////
  size_t cut_edge_size(partition_id const& pid1,
                       partition_id const& pid2) const
  {
    return cut_edges(pid1,pid2).size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of partitions
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return partition_graph.size();
  }

  void define_type(typer &t)
  {
    stapl_assert(false,"Define_type of the gpartition class called\n");
  }
}; // end gpartition class

} //end namespace stapl

#endif /* STAPL_CONTAINERS_GRAPH_PARTITIONERS_GPARTITION_H */
