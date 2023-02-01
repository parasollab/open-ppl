/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_UTILITY_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_UTILITY_HPP

#include <stapl/runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <vector>

#define MULTILEVEL_AGGREGATION 512

namespace stapl {

namespace partitioner_details {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to combine two vectors by summing their elements pair-wise.
/// @tparam T element type of the vectors.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct combine_vector_wf
{
  typedef std::vector<T> result_type;
  //////////////////////////////////////////////////////////////////////
  /// @param part1 first vector.
  /// @param part2 second vector.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition1, typename Partition2>
  result_type operator()(Partition1 const& part1, Partition2 const& part2) const
  {
    result_type result(part1);
    typename result_type::iterator it = result.begin(),
                               end_it = result.end();
    typename Partition2::const_iterator it2 = part2.begin();
    for (; it!=end_it; ++it, ++it2)
      *it += *it2;
    return result;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor computing the cut-weight produced by a vertex in a graph.
//////////////////////////////////////////////////////////////////////
struct get_cut_weight
{
  typedef size_t result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param v a graph vertex.
  /// @param hview view of the graph containing vertex @p v.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename HView>
  result_type operator()(Vertex v, HView hview)
  {
    size_t mypid = v.property().property.get_partition_id();
    size_t cut_weight = 0;
    typename Vertex::adj_edge_iterator edge_it = v.begin(),
                                       edge_end_it = v.end();
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      if (mypid !=
          hview[(*edge_it).target()].property().property.get_partition_id())
        cut_weight += (*edge_it).property().property.weight;
    }
    return cut_weight;
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Functor computing the partition weights of a graph
//////////////////////////////////////////////////////////////////////
struct compute_partition_weights
{
  typedef std::vector<size_t> result_type;
private:
  size_t m_num_partitions;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param num_partitions number of partitions.
  //////////////////////////////////////////////////////////////////////
  compute_partition_weights(size_t const& num_partitions)
    : m_num_partitions(num_partitions)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param part graph whose partition weights are computed.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition>
  result_type operator()(Partition part) const
  {
    result_type part_weights(m_num_partitions, 0);
    typename Partition::vertex_iterator it = part.begin(),
                                        end_it = part.end();
    for (; it!=end_it; ++it)
    {
      part_weights[(*it).property().property.get_partition_id()]
        += (*it).property().property.get_weight();
    }
    return part_weights;
  }

  void define_type(typer& t)
  {
    t.member(m_num_partitions);
  }
};

} //namespace partitioner_details


//////////////////////////////////////////////////////////////////////
/// @brief Compute the imbalance and cut-weight of a graph partition.
/// @param hview partitioned graph view.
/// @param num_parts number of partitions.
/// @return pair of the imbalance and cut-weight of the partition of @p hview.
//////////////////////////////////////////////////////////////////////
template <typename HView>
std::pair<double, size_t> print_partition_quality(HView const& hview,
                                                  size_t const& num_parts)
{
  using namespace partitioner_details;

  //compute cut-weight
  size_t cut_weight = map_reduce(get_cut_weight(), plus<size_t>(), hview,
                                 make_repeat_view(hview));
  cut_weight /= 2; //because undirected graph

  //calculate partition weights
  std::vector<size_t>
    partition_weights = map_reduce(compute_partition_weights(num_parts),
                                   combine_vector_wf<size_t>(),
                                   native_view(hview));
  //find max partition weight
  size_t max_partition_weight = *std::max_element(partition_weights.begin(),
                                                  partition_weights.end());
  size_t total_weight = std::accumulate(partition_weights.begin(),
                                        partition_weights.end(), 0);
  size_t opt_part_weight = ceil(total_weight/(double)num_parts);
  //compute imbalance
  double imbalance = max_partition_weight/(double)opt_part_weight;

  return std::make_pair(imbalance, cut_weight);
}

} //namespace stapl

#endif
