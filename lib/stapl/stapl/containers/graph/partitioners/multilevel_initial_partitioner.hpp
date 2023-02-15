/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_INITIAL_PARTITIONER_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_INITIAL_PARTITIONER_HPP

#include <stapl/runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/native_view.hpp>
#include <cmath>
#include <vector>
#include "metis_utils.h"

namespace stapl {

namespace partitioner_details {

//////////////////////////////////////////////////////////////////////
/// @brief Transform a graph base container into the CSR format used by
///        METIS.
//////////////////////////////////////////////////////////////////////
struct build_pair_weight_id
{
  metis_graph* m_metis_graph;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param metis_graph_obj METIS graph object storing the result
  ///        of the conversion.
  //////////////////////////////////////////////////////////////////////
  build_pair_weight_id(metis_graph* const& metis_graph_obj)
    : m_metis_graph(metis_graph_obj)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param part graph base container.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition>
  result_type operator()(Partition part)
  {
    metis_graph metis_graph_temp(part.size(), part.num_edges());
    metis_graph_temp.xadj[0] = 0;
    //size_t weight = 0;

    typename Partition::vertex_iterator it = part.begin(),
                                    end_it = part.end();
    size_t i=0;
    for (; it!=end_it; ++it, ++i) {
      typename Partition::reference v = *it;
      typename Partition::reference::adj_edge_iterator edge_it = v.begin(),
                                                   end_edge_it = v.end();
      metis_graph_temp.vwgt[i] = v.property().property.get_weight();
      metis_graph_temp.xadj[i+1] = v.size() + metis_graph_temp.xadj[i];
      //weight += metis_graph_temp.vwgt[i];

      for (size_t j=metis_graph_temp.xadj[i]; edge_it != end_edge_it;
           ++edge_it, ++j) {
        metis_graph_temp.adjncy[j] = (*edge_it).target();
        metis_graph_temp.adjwgt[j] = (*edge_it).property().property.weight;
      }
    }

    *m_metis_graph = metis_graph_temp;

    //return weight;
  }

  void define_type(typer& t)
  { t.member(m_metis_graph); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor for parallel nested bisection partitioning of a graph.
///        This implementation uses the METIS library for the bisection.
//////////////////////////////////////////////////////////////////////
struct metis_bisection_wf
{
private:
  metis_graph*     m_mgraph;
  size_t           m_num_parts;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param mgraph graph in CSR format used by METIS.
  /// @param num_parts number of partitions created.
  //////////////////////////////////////////////////////////////////////
  metis_bisection_wf(metis_graph* mgraph, size_t const& num_parts)
    : m_mgraph(mgraph), m_num_parts(num_parts)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param index index of partition to build.
  /// @param hview graph view.
  //////////////////////////////////////////////////////////////////////
  template <typename HView>
  void operator()(size_t const& index, HView hview)
  {
    size_t num_recursion_levels = std::log2(m_num_parts);
    int nparts = 2;
    int moptions[METIS_NOPTIONS];
    int edgecut;
    std::vector<int> part(m_mgraph->nvtxs);

    METIS_SetDefaultOptions(moptions);
    moptions[METIS_OPTION_NUMBERING]=0;

    //Iterate over levels of recursion
    for (size_t i=0; i < num_recursion_levels; ++i)
    {
      char partition_id = (index & (1 << (num_recursion_levels-i-1)))
                            >> (num_recursion_levels-i-1);

      METIS_PartGraphRecursive(&(m_mgraph->nvtxs), &(m_mgraph->ncon),
                               &(m_mgraph->xadj[0]), &(m_mgraph->adjncy[0]),
                               &(m_mgraph->vwgt[0]), NULL,
                               &(m_mgraph->adjwgt[0]), &nparts, NULL, NULL,
                               moptions, &edgecut,&(part[0]));

      keep_part(m_mgraph, &(part[0]), partition_id);
    }

    //Copy result to partition domain
    size_t num_vertices = m_mgraph->nvtxs;
    for (size_t j=0; j < num_vertices; ++j)
    {
      //Mark vertices with partition ids
      hview[m_mgraph->label[j]].property().property.set_partition_id(index);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_num_parts);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to set the partition id of a vertex with the result
///        of the METIS k-way partitioner.
//////////////////////////////////////////////////////////////////////
struct metis_kway_wf
{
private:
  std::vector<int>* m_part;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param part array containing the result of the METIS k-way partitioner.
  //////////////////////////////////////////////////////////////////////
  metis_kway_wf(std::vector<int>* part)
    : m_part(part)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex proxy.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex v)
  {
    //Mark vertices with partition ids
    v.property().property.set_partition_id((*m_part)[v.descriptor()]);
  }

  void define_type(typer& t)
  {
    t.member(m_part);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief METIS k-way partitioning of a graph.
/// @param mgraph graph in CSR format used by METIS.
/// @param num_partitions number of partitions created.
/// @param hview graph view.
//////////////////////////////////////////////////////////////////////
template <typename HView>
void metis_kway(metis_graph& mgraph, size_t const& num_partitions,
                HView& hview)
{
  int nparts = num_partitions;
  int moptions[METIS_NOPTIONS];
  int edgecut;
  std::vector<int> part(mgraph.nvtxs);

  METIS_SetDefaultOptions(moptions);
  moptions[METIS_OPTION_NUMBERING]=0;

  METIS_PartGraphKway(&mgraph.nvtxs, &mgraph.ncon, &mgraph.xadj[0],
                      &mgraph.adjncy[0], &mgraph.vwgt[0], NULL,
                      &mgraph.adjwgt[0], &nparts, NULL, NULL, moptions,
                      &edgecut, &part[0]);

  map_func(metis_kway_wf(&part), hview);
}


//////////////////////////////////////////////////////////////////////
/// @brief Object to broadcast a graph in CSR format used by METIS.
/// @todo Clean up the broadcast not to have a rmi at the algorithm level
//////////////////////////////////////////////////////////////////////
struct rmi_allgather_obj
  : public p_object
{
private:
  metis_graph const& m_local_graph;
  metis_graph& m_global_graph;
  size_t vtx_offset, edge_offset;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param local_graph local piece of the graph.
  /// @param global graph graph storing the result of the broadcast.
  ///
  /// Assuming global graph is already allocated with the required
  /// memory to hold all concatenations of the local graphs.
  //////////////////////////////////////////////////////////////////////
  rmi_allgather_obj(metis_graph const& local_graph, metis_graph& global_graph)
    : m_local_graph(local_graph), m_global_graph(global_graph),
      vtx_offset(0), edge_offset(0)
  {
    m_global_graph.xadj[0] = 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Execute the broadcast and store result in @p m_global_graph.
  //////////////////////////////////////////////////////////////////////
  void operator()()
  {
    stapl::futures<metis_graph> h1 =
      stapl::allgather_rmi(this->get_rmi_handle(),
                           &rmi_allgather_obj::get_local_graph);

    for (stapl::futures<metis_graph>::size_type i = 0; i<h1.size(); ++i) {
      append_metis_graph(h1.get(i));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Append a piece of graph to the broadcast result
  ///        @p m_global_graph.
  /// @param g piece of graph in CSR format used by METIS.
  //////////////////////////////////////////////////////////////////////
  void append_metis_graph(metis_graph const& g)
  {
    for (size_t i=vtx_offset; i< (size_t)g.nvtxs + vtx_offset; ++i) {
      m_global_graph.vwgt[i] = g.vwgt[i-vtx_offset];

      m_global_graph.xadj[i+1] =
        g.xadj[i-vtx_offset+1] + m_global_graph.xadj[vtx_offset];
    }

    for (size_t i=edge_offset; i< (size_t)g.nedges + edge_offset; ++i) {
      m_global_graph.adjncy[i] = g.adjncy[i-edge_offset];
      m_global_graph.adjwgt[i] = g.adjwgt[i-edge_offset];
    }

    vtx_offset += g.nvtxs;
    edge_offset += g.nedges;
  }

  metis_graph get_local_graph()
  { return m_local_graph; }

  void define_type(typer& t)
  {
    t.member(m_local_graph);
    t.member(m_global_graph);
  }

};

} //namespace partitioner_details


//////////////////////////////////////////////////////////////////////
/// @brief Parallel nested bisection partitioning of a graph.
///        This implementation uses the METIS library for the bisection.
/// @param hview graph view.
/// @param num_partitions number of partitions created.
/// @param imbalance partition imbalance allowed.
/// @return total vertex weight of graph @p hview.
//////////////////////////////////////////////////////////////////////
template <typename HView>
size_t recursive_bisection_partition(HView& hview,
                                   size_t const& num_partitions,
                                   size_t const& imbalance)
{
  using namespace partitioner_details;

  metis_graph local_metis_graph;

  size_t num_vertices = hview.size();
  size_t num_edges = hview.container().num_edges_collective();

  //Fill the local metis graph.
  map_func(build_pair_weight_id(&local_metis_graph),
           native_view(hview));

  //Broadcast the last level to all PEs
  metis_graph global_metis_graph(num_vertices, num_edges);
  rmi_allgather_obj obj(local_metis_graph, global_metis_graph);
  obj();

  size_t total_weight = 0;
  std::vector<int>::iterator it = global_metis_graph.vwgt.begin(),
                         end_it = global_metis_graph.vwgt.end();
  for (; it != end_it; ++it)
    total_weight += *it;

  //Local recursive bisection
  if ((get_num_locations() != num_partitions) || (num_partitions%2 != 0))
  {
    metis_kway(global_metis_graph, num_partitions, hview);
  }
  else
  {
    map_func(metis_bisection_wf(&global_metis_graph, num_partitions),
             counting_view<size_t>(num_partitions), make_repeat_view(hview));
  }

  return total_weight;
}

} //namespace stapl

#endif
