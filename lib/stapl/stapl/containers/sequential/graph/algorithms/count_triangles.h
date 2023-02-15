/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_COUNT_TRIANGLES_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_COUNT_TRIANGLES_HPP

#include "graph_algo_util.h"

namespace stapl {
namespace sequential{

////////////////////////////////////////////////////////////////////////
/// @brief Counts the number of 'triangles' in a graph that contain the given
/// vertex @p v.
/// @param g the UNDIRECTED graph
/// @param v the vertex whose triangles are to be counted
/// @return the number of triangles
/// @note This algorithm is guaranteed correct only for undirected graphs
/// that contain no self-loops or parallel edges.
/// @note The complexity for the algorithm is O(|E|).
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////
template<typename Graph>
size_t count_local_triangles(Graph& g, typename Graph::vertex_descriptor v)
{
  typedef typename Graph::vertex_iterator   vi_type;
  typedef typename Graph::adj_edge_iterator aei_type;

  map_property_map<Graph, bool> nmap;

  size_t triangle_count = 0;
  vi_type it = g.find_vertex(v);
  aei_type aei, aei_end = (*it).end();
  aei_type naei, naei_end; //neighbor's neighbors
  //we mark v's neighbors as we go along; this should turn out correct
  for (aei = (*it).begin(); aei != aei_end; ++aei) {
    nmap.put((*aei).target(), true); //target is a neighbor

    //now find the triangles
    //NOTE: reassigning it here; shouldn't need v anymore
    it = g.find_vertex((*aei).target());
    naei_end = (*it).end();
    for (naei = (*it).begin(); naei != naei_end; ++naei) {
      if (nmap.get((*naei).target()))
        ++triangle_count;
    }
  }

  //shouldn't need to divide by 2; I think each triangle is seen only once.
  //return triangle_count/2;
  return triangle_count;
}

}//end namespace sequential
}//end namespace stapl

#endif
