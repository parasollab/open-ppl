/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_REFINER_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_MULTILEVEL_REFINER_HPP

#include <stapl/runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/native_view.hpp>
#include <vector>
#include <stapl/containers/graph/algorithms/aggregator.hpp>

namespace stapl {

namespace partitioner_details {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to setting the partition id of a vertex.
//////////////////////////////////////////////////////////////////////
struct set_children_partition_id
{
private:
  size_t m_pid;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param pid partition id assigned to vertex.
  //////////////////////////////////////////////////////////////////////
  set_children_partition_id(size_t const& pid)
    : m_pid(pid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param p property of the vertex.
  //////////////////////////////////////////////////////////////////////
  template <typename Property>
  result_type operator()(Property& p) const
  {
    p.property.set_partition_id(m_pid);
  }

  void define_type(typer& t)
  {
    t.member(m_pid);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor setting the partition ids of super-vertex's children.
//////////////////////////////////////////////////////////////////////
struct set_children_pid
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param v super-vertex proxy.
  /// @param hview graph view.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename HView>
  void operator()(Vertex v, HView hview) const
  {
    size_t mypid = v.property().property.get_partition_id();
    v.property().children_apply(hview, set_children_partition_id(mypid));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor getting the weight of each partition.
//////////////////////////////////////////////////////////////////////
struct get_partition_weights
{
  typedef std::vector<size_t> result_type;
private:
  size_t m_num_partitions;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param num_partitions number of partitions.
  //////////////////////////////////////////////////////////////////////
  get_partition_weights(size_t const& num_partitions)
    : m_num_partitions(num_partitions)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param part base container.
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


//////////////////////////////////////////////////////////////////////
/// @brief Functor to add the adjacent edge weight and associated
///        partition id to a vertex.
//////////////////////////////////////////////////////////////////////
struct add_partition
{
  typedef void result_type;
private:
  size_t m_pid;
  size_t m_weight;
  size_t m_target;
public:
  //////////////////////////////////////////////////////////////////////
  /// @param pid partition id.
  /// @param weight weight associated with partition @p pid.
  /// @param target target vertex id.
  //////////////////////////////////////////////////////////////////////
  add_partition(size_t const& pid,
                size_t const& weight, size_t const& target)
    : m_pid(pid), m_weight(weight), m_target(target)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param p vertex property reference.
  //////////////////////////////////////////////////////////////////////
  template <typename Property>
  result_type operator()(Property& p) const
  {
    p.property.add_partition(std::make_pair(m_pid, m_weight));
  }

  size_t target() const
  {
    return m_target;
  }

  void define_type(typer& t)
  {
    t.member(m_pid);
    t.member(m_weight);
    t.member(m_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to share vertex partition id with neighboring vertices.
/// @tparam HView graph view type.
//////////////////////////////////////////////////////////////////////
template <typename HView>
struct send_mypartition_id
{
private:
  aggregator<add_partition, typename HView::view_container_type> m_aggr;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param hview graph view.
  //////////////////////////////////////////////////////////////////////
  send_mypartition_id(HView const& hview)
    : m_aggr(hview.get_container(), MULTILEVEL_AGGREGATION)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex sharing its partition id.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex v)
  {
    size_t mypid = v.property().property.get_partition_id();
    typename Vertex::adj_edge_iterator edge_it = v.begin(),
                                       edge_end_it = v.end();
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      //send my partition id and edge weight to neighbor
      m_aggr.add(add_partition(mypid, (*edge_it).property().property.weight,
                               (*edge_it).target()));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_aggr);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor updating neighboring partition information of a vertex.
//////////////////////////////////////////////////////////////////////
struct update_partition
{
  typedef void result_type;
private:
  size_t m_old_pid;
  size_t m_new_pid;
  size_t m_weight;
  size_t m_target;
public:
  //////////////////////////////////////////////////////////////////////
  /// @param old_pid id of source partition whose weight is subtracted
  ///        with @p weight.
  /// @param new_pid id of target partition whose weight is incremented
  ///        with @p weight.
  /// @param weight weight to be moved.
  /// @param target vertex whose neighboring partition
  ///        information are updated.
  //////////////////////////////////////////////////////////////////////
  update_partition(size_t const& old_pid, size_t const& new_pid,
                size_t const& weight, size_t const& target)
    : m_old_pid(old_pid), m_new_pid(new_pid), m_weight(weight),
      m_target(target)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param vertex property reference.
  //////////////////////////////////////////////////////////////////////
  template <typename Property>
  result_type operator()(Property& p) const
  {
    p.property.update_partition(m_old_pid, m_new_pid, m_weight);
  }

  size_t target() const
  {
    return m_target;
  }

  void define_type(typer& t)
  {
    t.member(m_old_pid);
    t.member(m_new_pid);
    t.member(m_weight);
    t.member(m_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Fiduccia-Mattheyses heuristic functor to decrease the edge-cut
///        of a graph partition while maintaining a low partition imbalance.
/// @tparam Hview graph view type.
//////////////////////////////////////////////////////////////////////
template <typename HView>
struct fm_refinement_wf
{
private:
  aggregator<update_partition, typename HView::view_container_type> m_aggr;
  HView m_hview;
  std::vector<size_t>& m_part_weights;
  size_t m_opt_part_weight;
  size_t m_imbalance;

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if a vertex move respects the partition balance.
  /// @param old_pid partition id of the vertex before the move.
  /// @param new_pid partition id of the vertex after the move.
  /// @param weight weight of vertex considered for a move.
  /// @return true if vertex move respects the partition balance.
  //////////////////////////////////////////////////////////////////////
  bool respect_weight_balance(size_t const& old_pid, size_t const& new_pid,
                              size_t const& weight) const
  {
    return (m_part_weights[old_pid]-weight >=
             m_opt_part_weight*(1-m_imbalance/100.0)) &&
           (m_part_weights[new_pid]+weight <=
             m_opt_part_weight*(1+m_imbalance/100.0));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if a vertex move improves the partition balance.
  /// @param old_pid partition id of the vertex before the move.
  /// @param new_pid partition id of the vertex after the move.
  /// @param weight weight of vertex considered for a move.
  /// @return true if vertex move improves the partition balance.
  //////////////////////////////////////////////////////////////////////
  bool improve_weight_balance(size_t const& old_pid, size_t const& new_pid,
                              size_t const& weight) const
  {
    return respect_weight_balance(old_pid, new_pid, weight) &&
           (m_part_weights[old_pid] > m_opt_part_weight*(1+m_imbalance/100.0) ||
           m_part_weights[new_pid] < m_opt_part_weight*(1-m_imbalance/100.0));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update neighbor information with the new partition id of
  ///        the vertex @p v.
  /// @param v vertex moved.
  /// @param old_pid partition id of the vertex before the move.
  /// @param new_pid partition id of the vertex after the move.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void update_neighbors(Vertex v, size_t const& old_pid, size_t const& new_pid)
  {
    m_part_weights[old_pid] -= v.property().property.get_weight();
    m_part_weights[new_pid] += v.property().property.get_weight();
    typename Vertex::adj_edge_iterator edge_it = v.begin(),
                                       edge_end_it = v.end();
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      m_aggr.add(update_partition(old_pid, new_pid,
                                  (*edge_it).property().property.weight,
                                  (*edge_it).target()));
    }
  }

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param hview graph view.
  /// @param part_weights weight of each partition.
  /// @param opt_part_weight optimal partition weight.
  /// @param imbalance partition imbalance allowed for the refinement.
  //////////////////////////////////////////////////////////////////////
  fm_refinement_wf(HView const& hview,
                   std::vector<size_t>& part_weights,
                   size_t const& opt_part_weight,
                   size_t const& imbalance)
    : m_aggr(hview.get_container(), MULTILEVEL_AGGREGATION), m_hview(hview),
      m_part_weights(part_weights), m_opt_part_weight(opt_part_weight),
      m_imbalance(imbalance)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param part graph base container.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition>
  void operator()(Partition part)
  {
    //bool moved_a_vertex = false;
    typename Partition::iterator part_it = part.begin(),
                             part_end_it = part.end();
    for (; part_it != part_end_it; ++part_it)
    {
      typename Partition::reference v = *part_it;

      //if v is not a boundary vertex or v.id > v.ed
      if (v.property().property.cannot_move())
        continue;

      size_t mypid = v.property().property.get_partition_id();

      //sort neighboring partitions by decreasing cut-weight gain
      v.property().property.sort_partition_info();
      //Get partition_infos
      std::vector<std::pair<size_t, size_t> >
        part_info(v.property().property.get_partition_info());

      //Get my partition edge weight
      size_t partition_edge_weight =
        v.property().property.get_internal_degree();


      std::vector<std::pair<size_t, size_t> >::iterator it = part_info.begin(),
                                                    end_it = part_info.end();
      for (; it != end_it; ++it)
      {
        if (it->second > partition_edge_weight)
        {
          if (respect_weight_balance(mypid, it->first,
                                     v.property().property.get_weight()))
          {
            //std::cout << "Vertex moved " << v.descriptor() << std::endl;
            //moved_a_vertex = true;
            update_neighbors(v, mypid, it->first);
            v.property().property.update_partition_id(it->first);
            break;
          }
        }
        else if (it->second == partition_edge_weight)
        {
          if (it->first != mypid &&
              improve_weight_balance(mypid, it->first,
                                     v.property().property.get_weight()))
          {
            //std::cout << "Vertex moved " << v.descriptor() << std::endl;
            //moved_a_vertex = true;
            update_neighbors(v, mypid, it->first);
            v.property().property.update_partition_id(it->first);
            break;
          }
        }
        else
          break;
      }
    }
    //return moved_a_vertex;
  }

  void define_type( typer& t)
  {
    t.member(m_aggr);
    t.member(m_hview);
    t.member(m_part_weights);
    t.member(m_opt_part_weight);
    t.member(m_imbalance);
  }
};

} //namespace partitioner_details


//////////////////////////////////////////////////////////////////////
/// @brief Fiduccia-Mattheyses heuristic to decrease the edge-cut
///        of a graph partition while maintaining a low partition imbalance.
/// @param hview graph view.
/// @param num_partitions number of partitions.
/// @param opt_part_weight optimal partition weight.
/// @param imbalance partition imbalance allowed for the refinement.
/// @param pass_fm number of refinement passes.
//////////////////////////////////////////////////////////////////////
template <typename HView>
void fm_refinement(HView const& hview,
                   size_t const& num_partitions,
                   size_t const& optimal_part_weight,
                   size_t const& imbalance,
                   size_t const& pass_fm)
{
  using namespace partitioner_details;

  //Send partition id and edge weight of a vertex to neighbors
  map_func(send_mypartition_id<HView>(hview), hview);

  for (size_t j=0; j<pass_fm; ++j)
  {
    //calculate partition weights
    std::vector<size_t>
      partition_weights = map_reduce(get_partition_weights(num_partitions),
                                     combine_vector_wf<size_t>(),
                                     native_view(hview));

    map_func(fm_refinement_wf<HView>(hview, partition_weights,
                                     optimal_part_weight,imbalance),
             native_view(hview));

  }
}

} //namespace stapl

#endif
