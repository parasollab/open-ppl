/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_REFINERS_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_REFINERS_HPP

#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/algorithms/hierarchical_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/views/balance_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/array_view.hpp>
#include <vector>
#include <utility>
#include <cmath>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor extracting a vertex-weight in a property map.
/// @tparam Weight_property property map type.
/// @ingroup pgraphPartitioner
//////////////////////////////////////////////////////////////////////
template<typename Weight_property>
struct get_vertex_weight
{
  typedef size_t result_type;
private:
  Weight_property weight_property;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param prop property map.
  //////////////////////////////////////////////////////////////////////
  get_vertex_weight(Weight_property const& prop)
    : weight_property(prop)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param elt vertex.
  /// @return vertex weight
  //////////////////////////////////////////////////////////////////////
  template<typename Elt>
  result_type operator()(Elt elt)
  {
    return weight_property.get(elt);
  }

  void define_type(typer &t)
  {
    t.member(weight_property);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to get and store partition weights in a property
///        map.
/// @ingroup pgraphPartitioner
/// @tparam Weight_property vertex property map type.
/// @tparam Weight_partition_map partition property map type.
//////////////////////////////////////////////////////////////////////
//Work function to get and store partition weights in an external property map
template<typename Weight_property, typename Weight_partition_map>
struct get_partition_weights
{
  typedef void result_type;
private:
  Weight_property weight_property;
  Weight_partition_map partition_weight_map;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param prop vertex property map.
  /// @param prop2 partition property map.
  //////////////////////////////////////////////////////////////////////
  get_partition_weights(Weight_property const& prop,
                        Weight_partition_map const& prop2)
    : weight_property(prop), partition_weight_map(prop2)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param elt partition.
  //////////////////////////////////////////////////////////////////////
  template<typename Elt>
  void operator()(Elt elt)
  {
    typename Elt::property_type p = elt.property();
    //Collect vertex-weights of this partition
    size_t weight =
      map_reduce(get_vertex_weight<Weight_property>(weight_property),
                 plus<size_t>(), p);
    partition_weight_map.put(elt, weight);
  }

  void define_type(typer &t)
  {
    t.member(weight_property);
    t.member(partition_weight_map);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to balance 2 neighboring partitions in a
///        diffusive fashion according to the independent set.
/// @ingroup pgraphPartitioner
/// @tparam Weight_property vertex property map type.
/// @tparam Weight_partition_map partition property map type.
//////////////////////////////////////////////////////////////////////
template<typename Weight_property, typename Weight_partition_map>
struct balance_neighbor_partitions_wf
{
public:
  typedef std::vector<std::pair<size_t, size_t> > indep_edge_set_type;

private:
  Weight_property       weight_property;
  Weight_partition_map  partition_weight_map;
  indep_edge_set_type   indep_edge_set;

  int                   tolerance;

public:
  typedef void result_type;

//////////////////////////////////////////////////////////////////////
/// @param prop vertex property map.
/// @param prop2 partition property map.
/// @param indep independent sets of partitions.
/// @param tol vertex weight imbalance allowed.
//////////////////////////////////////////////////////////////////////
  balance_neighbor_partitions_wf(Weight_property const& prop,
                                 Weight_partition_map const& prop2,
                                 indep_edge_set_type const& indep,
                                 size_t const& tol)
    : weight_property(prop), partition_weight_map(prop2),
    indep_edge_set(indep), tolerance(tol)
 { }

  void define_type(typer &t)
  {
    t.member(weight_property);
    t.member(partition_weight_map);
    t.member(indep_edge_set);
    t.member(tolerance);
  }

  //////////////////////////////////////////////////////////////////////
  /// @param p1 partition considered as a source of vertices.
  /// @param hview partition view.
  //////////////////////////////////////////////////////////////////////
  template<typename Elt, typename HView>
  void operator()(Elt p1, HView hview)
  {
#ifdef DEBUG_INFO
    std::cout << "Location " << get_location_id()
              << " with partition " << p1.descriptor() << std::endl;
#endif

    bool second = true;

    indep_edge_set_type::iterator it = std::find_if(indep_edge_set.begin(),
                                                    indep_edge_set.end(),
                                                    is_source(p1.descriptor()));

    if (it==indep_edge_set.end())
    {
      it=std::find_if(indep_edge_set.begin(), indep_edge_set.end(),
                      is_target(p1.descriptor()));

      if (it==indep_edge_set.end())
        return;

      second=false;
    }

    typename HView::vertex_reference p2 =
      second ? hview[it->second] : hview[it->first];

    //Calculate weight difference and weight to move
    int weight_diff=partition_weight_map.get(p1)-partition_weight_map.get(p2);

#ifdef DEBUG_INFO
    std::cout << "read partition_weight_map: " << partition_weight_map.get(*p1)
              << " and " << partition_weight_map.get(*p2) << std::endl;
    std::cout << "weight_diff: " << weight_diff << std::endl;
#endif

    if (std::abs(weight_diff) > tolerance)
    {
      if (weight_diff > 0)
      {
        move_vertices(
          hview, p1, second ? it->second : it->first, std::abs(weight_diff)/2
        );
      }
    }

  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper functor which calls contains on a vertex in a graph
  ///   with passed descriptor as parameter.
  //////////////////////////////////////////////////////////////////////
  template<typename D>
  struct check_contains
  {
    D m_descriptor;

    typedef bool result_type;

    check_contains(D const& d)
      : m_descriptor(d)
    { }

    template<typename T>
    bool operator()(T& t) const
    {
      return t.domain().contains(m_descriptor);
    }

    void define_type(typer& t)
    {
      t.member(m_descriptor);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Balance 2 neighboring partitions in a diffusive fashion.
  /// @param hview partition view.
  /// @param p1 overweighted partition.
  /// @param d descriptor of adjacent vertex.
  /// @param weight total weight to move if possible to even partitions.
  /// @todo Explore making nested types (vertex/edge iterators)
  /// more accessible to user.
  //////////////////////////////////////////////////////////////////////
  template<typename HView, typename P1, typename D>
  void move_vertices(HView hview, P1 p1, D const& d, int weight)
  {

    typename P1::property_type p1_graph=p1.property();

    std::vector<std::pair<size_t, size_t> > vertex_edge_cuts;
    typename P1::property_type::vertex_iterator it;
    typename P1::property_type::vertex_iterator::reference::adj_edge_iterator
      edge_it, edge_it_e;
    for (it=p1_graph.begin(); it!=p1_graph.end(); ++it)
    {
      int num_edge_cuts=0;
      edge_it = (*it).begin();
      edge_it_e = (*it).end();
      for (; edge_it!=edge_it_e; ++edge_it)
      {
        if (hview.vp_apply(d, check_contains<D>((*edge_it).target())))
          num_edge_cuts++;
      }
      vertex_edge_cuts.push_back(std::make_pair((*it).descriptor(),
                                 num_edge_cuts));
    }

    //sort vertices of p1 by decreasing edge-cut
    std::sort(vertex_edge_cuts.begin(), vertex_edge_cuts.end(), edge_cut_cmp());

    int real_weight_to_move=0;
    std::vector<std::pair<size_t, size_t> >::iterator vect_it;

    //pick first vertices whose weight-sum is less than or equal to 'weight'
    for (vect_it=vertex_edge_cuts.begin(); vect_it!=vertex_edge_cuts.end();
        ++vect_it)
    {
      int vertex_weight = weight_property.get(vect_it->first);
      if (vertex_weight  < 2*(weight - real_weight_to_move))
      {
        move_vertex(hview, p1.descriptor(), d, vect_it->first);
        real_weight_to_move+=vertex_weight;
        if ( 2*(weight - real_weight_to_move) <= tolerance )
          break;
      }
    }

#ifdef DEBUG_INFO
    std::cout << "real_weight_to_move: " << real_weight_to_move << std::endl;
#endif

    //update weight of partition Pi and Pj
    if (real_weight_to_move!=0)
    {
      //remove weight from p1
      partition_weight_map.apply(p1,add_partition_weight(-real_weight_to_move));
      //add weight to p2
      partition_weight_map.apply(
        hview[d], add_partition_weight(real_weight_to_move)
      );
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Compare edge-cut of 2 vertices.
  //////////////////////////////////////////////////////////////////////
  struct edge_cut_cmp
  {
    typedef std::pair<size_t, size_t> pair_type;

    //////////////////////////////////////////////////////////////////////
    /// @param lhs pair1 of vertex descriptor and vertex edge-cut.
    /// @param rhs pair2 of vertex descriptor and vertex edge-cut.
    /// @return true if pair1 has a higher edge-cut than pair2.
    //////////////////////////////////////////////////////////////////////
    bool operator()(pair_type const& lhs, pair_type const& rhs)
    {
      return lhs.second > rhs.second;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the 'elt' edge has 'source'  as source.
  //////////////////////////////////////////////////////////////////////
  struct is_source
  {
    typedef std::pair<size_t, size_t> pair_type;
  private:
    size_t source;
  public:
    //////////////////////////////////////////////////////////////////////
    /// @param source source vertex descriptor.
    //////////////////////////////////////////////////////////////////////
    is_source(size_t const& s)
      : source(s)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @param elt graph edge.
    //////////////////////////////////////////////////////////////////////
    bool operator()(pair_type const& elt)
    {
        return elt.first == source;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the 'elt' edge has 'target'  as target.
  //////////////////////////////////////////////////////////////////////
  struct is_target
  {
    typedef std::pair<size_t, size_t> pair_type;
  private:
    size_t target;
  public:
    //////////////////////////////////////////////////////////////////////
    /// @param target target vertex descriptor.
    //////////////////////////////////////////////////////////////////////
    is_target(size_t const& t)
      : target(t)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @param elt graph edge.
    //////////////////////////////////////////////////////////////////////
    bool operator()(pair_type const& elt)
    {
        return elt.second == target;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Increment partition weight with a weight.
  //////////////////////////////////////////////////////////////////////
  struct add_partition_weight
  {
  private:
    int weight;

  public:
    //////////////////////////////////////////////////////////////////////
    /// @param w weight to add to partition weight.
    //////////////////////////////////////////////////////////////////////
    add_partition_weight(int const& w)
      : weight(w)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @param elt partition weight.
    //////////////////////////////////////////////////////////////////////
    void operator() (size_t &elt) const
    {
      elt+=weight;
    }

    void define_type(typer& t)
    {
      t.member(weight);
    }
  };
}; // struct balance_neighbor_partitions_wf

#ifdef DEBUG_INFO
//////////////////////////////////////////////////////////////////////
/// @brief Work function to print weight of partitions.
/// @ingroup pgraphPartitioner
//////////////////////////////////////////////////////////////////////
template<typename Weight_property>
struct print_prop_map
{
private:
  Weight_property weight_property;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param prop partition weight property map.
  //////////////////////////////////////////////////////////////////////
  print_prop_map(Weight_property const& prop)
    : weight_property(prop)
 { }

  //////////////////////////////////////////////////////////////////////
  /// @param elt partition.
  //////////////////////////////////////////////////////////////////////
  template<typename Elt>
  void operator()(Elt elt)
  {
    std::cout << "Partition " << elt.descriptor() << ": "
              << weight_property.get(elt) << std::endl;
  }

  void define_type(typer &t)
  {
    t.member(weight_property);
  }

};
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Refiner to balance partitions in a diffusive fashion.
///        The diffusion is done pair-wise and each diffusion phase
///        considers an independent set of partition pairs (2 pairs
///        which do not have a partition in common)
/// @ingroup pgraphPartitioner
/// @tparam Weight_property vertex property map type.
//////////////////////////////////////////////////////////////////////
template<typename Weight_property>
struct weight_balanced_refiner
{
public:
  typedef std::vector<std::vector<std::pair<size_t, size_t> > >
    indep_edge_set_type;

private:
  Weight_property weight_property;
  indep_edge_set_type indep_edge_sets;
  size_t tolerance;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param prop vertex property map.
  /// @param vec independent sets of partitions.
  /// @param tol vertex weight imbalance allowed.
  //////////////////////////////////////////////////////////////////////
  weight_balanced_refiner(Weight_property const& prop,
                          indep_edge_set_type const& vec, size_t const& tol)
    : weight_property(prop), indep_edge_sets(vec), tolerance(tol)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param gpart partition to be refined.
  //////////////////////////////////////////////////////////////////////
  template<class GView>
  gpartition<GView> operator()(gpartition<GView> const& gpart) const
  {

    typedef typename gpartition<GView>::partition_view_t hview_type;
    hview_type hview = gpart.partition();

    typedef array<size_t>                    partition_weight_storage_t;
    typedef array_view<partition_weight_storage_t> partition_weight_view_t;
    typedef graph_external_property_map<hview_type, size_t,
                                        partition_weight_view_t>
                                                 partition_weight_map_t;

    size_t num_partitions = hview.size();

    //Property map of weights of partitions
    partition_weight_storage_t prop_storage(num_partitions);
    partition_weight_view_t    current_prop(prop_storage);
    partition_weight_map_t     partition_weight_map(hview, current_prop);

    //Property map of weights of partitions for previous iteration
    partition_weight_storage_t prev_weight(num_partitions);
    partition_weight_view_t    prev_prop(prev_weight);


    //Compute initial weight of each partition
    map_func(get_partition_weights<Weight_property,
             partition_weight_map_t>(weight_property, partition_weight_map),
             hview);


#ifdef DEBUG_INFO
    if (get_location_id()==0)
     std::cout << "Original partition weights:" << std::endl;

    map_func(print_prop_map<partition_weight_map_t>(partition_weight_map),
             hview);
#endif

    do
    {
      //Save current weight of each partition
      copy(current_prop, prev_prop);

      indep_edge_set_type::const_iterator phase_it;
      //Iterate over the different independent edge sets
      for (phase_it=indep_edge_sets.begin(); phase_it!=indep_edge_sets.end();
          ++phase_it)
      {
        //Try to balance weights between neighboring partitions
        map_func(balance_neighbor_partitions_wf<Weight_property,
                                                partition_weight_map_t>(
                   weight_property, partition_weight_map, *phase_it, tolerance),
                 hview,
                 make_repeat_view(hview));
      }
    } while (!map_reduce(equal_to<size_t>(), logical_and<bool>(), current_prop,
                        prev_prop));
      //until it converges -- no vertex movement between 2 iterations

#ifdef DEBUG_INFO
    if (get_location_id()==0)
     std::cout << "New partition weights,"
               << "after vertex weight balanced refinement:" << std::endl;
    map_func(print_prop_map<partition_weight_map_t>(partition_weight_map),
             hview);
#endif

    return gpart;
  }

  void define_type(typer &t)
  {
    t.member(weight_property);
    t.member(tolerance);
    t.member(indep_edge_sets);
  }
};

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_PARTITIONERS_REFINERS_HPP
