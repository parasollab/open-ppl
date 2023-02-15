/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_TRAITS_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_TRAITS_HPP

#include "adjacency_list_core_graph.h"
#include "undirected_util.h"

#include "vdg_storage_list.h"
#include "vdg_storage_vector.h"
#include "vdg_intVD.h"

namespace stapl {
namespace sequential {

template <class Traits, graph_attributes type>
struct graph_type;

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the sequential graph.
/// @ingroup graphTraits
///
/// The adjacency list is used by default, with vector storage for
/// vertices and their adjacent edges, and integral vertex descriptors.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, class VertexP, class EdgeP>
class adj_graph_traits
{
  typedef adj_graph_traits<D, M, VertexP, EdgeP> this_type;
public:
  typedef VertexP vertex_property;
  typedef EdgeP edge_property;
  typedef size_t simple_vertex_descriptor;
  typedef size_t vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor> edge_descriptor;

  typedef vdg_storage_int<this_type> storage_type;

  //specify the core graph class; e.g., the graph storage
  //by default we implement an adjacency list;
  typedef typename select_edge<vertex_descriptor, edge_property, D>::type
    edge_type;

  typedef adjacency_list_graph<this_type> core_graph_type;
  typedef typename graph_type<this_type, D>::type directness_type;
  typedef typename graph_type<this_type, M>::type multiplicity_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits for the sequential graph with "smart" list storage.
/// @ingroup graphTraits
///
/// The adjacency list is used by default, with smart-list storage for
/// vertices and their adjacent edges, and smart vertex descriptors.
/// A "smart" list storage keeps versioning information for the graph,
/// using which "smart" vertex descriptors can return iterators to their
/// vertices in O(1) time. Smart vertex descriptors store iterators to
/// their vertices inside, and return that iterator if the version matches
/// the version of the graph.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, class VertexP, class EdgeP>
class adj_graph_traits_list_storage
{
  typedef adj_graph_traits_list_storage<D, M, VertexP, EdgeP> this_type;
public:
  typedef VertexP vertex_property;
  typedef EdgeP edge_property;
  typedef size_t simple_vertex_descriptor;
  typedef vertex_descriptor_list<D, simple_vertex_descriptor,
                                 VertexP, EdgeP> vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor> edge_descriptor;

  typedef vdg_storage_list<this_type> storage_type;

  //specify the core graph class; e.g., the graph storage
  //by default we implement an adjacency list;
  typedef typename select_edge<vertex_descriptor, edge_property, D>::type
    edge_type;

  typedef adjacency_list_graph<this_type> core_graph_type;
  typedef typename graph_type<this_type, D>::type directness_type;
  typedef typename graph_type<this_type, M>::type multiplicity_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits for the sequential graph with "smart" vector storage.
/// @ingroup graphTraits
///
/// The adjacency list is used by default, with smart-vector storage for
/// vertices and their adjacent edges, and smart vertex descriptors.
/// A "smart" vector storage keeps versioning information for the graph,
/// using which "smart" vertex descriptors can return iterators to their
/// vertices in O(1) time. Smart vertex descriptors store iterators to
/// their vertices inside, and return that iterator if the version matches
/// the version of the graph.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, class VertexP, class EdgeP>
class adj_graph_traits_vector_storage
{
  typedef adj_graph_traits_vector_storage<D, M, VertexP, EdgeP> this_type;
public:
  typedef VertexP vertex_property;
  typedef EdgeP edge_property;
  typedef size_t simple_vertex_descriptor;
  typedef vertex_descriptor_vector<D, simple_vertex_descriptor,
                                   VertexP, EdgeP> vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor> edge_descriptor;
  typedef vdg_storage_vector<this_type> storage_type;
  //specify the core graph class; e.g., the graph storage
  //by default we implement an adjacency list;
  typedef typename select_edge<vertex_descriptor, edge_property, D>::type
    edge_type;
  typedef adjacency_list_graph<this_type> core_graph_type;
  typedef typename graph_type<this_type, D>::type directness_type;
  typedef typename graph_type<this_type, M>::type multiplicity_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits for the sequential graph with vector storage.
/// @ingroup graphTraits
///
/// The adjacency list is used by default, with vector storage for
/// vertices and their adjacent edges, and integral vertex descriptors.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, class VertexP, class EdgeP>
class adj_graph_traits_vector_storage_intVD
{
  typedef adj_graph_traits_vector_storage_intVD<D, M, VertexP, EdgeP> this_type;
public:
  typedef VertexP vertex_property;
  typedef EdgeP edge_property;
  typedef size_t vertex_descriptor;
  typedef size_t simple_vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor> edge_descriptor;
  typedef vdg_storage_int<this_type> storage_type;
  //specify the core graph class; e.g., the graph storage
  //by default we implement an adjacency list;
  typedef typename select_edge<vertex_descriptor, edge_property, D>::type
    edge_type;
  typedef adjacency_list_graph<this_type> core_graph_type;
  typedef typename graph_type<this_type, D>::type directness_type;
  typedef typename graph_type<this_type, M>::type multiplicity_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits for the sequential graph with map storage.
/// @ingroup graphTraits
///
/// The adjacency list is used by default, with map storage for
/// vertices and vector storage for their adjacent edges, and integral
/// vertex descriptors.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, class VertexP, class EdgeP>
class adj_map_int
{
  typedef adj_map_int<D, M, VertexP, EdgeP> this_type;
public:
  typedef VertexP vertex_property;
  typedef EdgeP edge_property;
  typedef size_t vertex_descriptor;
  typedef size_t simple_vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor> edge_descriptor;
  typedef vdg_map_int<this_type> storage_type;
  typedef typename select_edge<vertex_descriptor, edge_property, D>::type
    edge_type;
  typedef adjacency_list_graph<this_type> core_graph_type;
  typedef typename graph_type<this_type, D>::type directness_type;
  typedef typename graph_type<this_type, M>::type multiplicity_type;
};

}  // namespace sequential
}  // namespace stapl.

#endif
