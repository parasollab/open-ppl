/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_PAGE_RANK_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_PAGE_RANK_HPP

#include "graph_algo_util.h"
#include <vector>

namespace stapl {
namespace sequential{

///////////////////////////////////////////////////////////////////////////
/// @brief Keeps track of the number of iterations that have
/// been run. Will return true after n iterations have passed.
/// @param n The number of iterations.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
struct n_iterations
{
  explicit n_iterations(std::size_t n)
   : n(n)
   {
   }

  template<typename RankMap, typename Graph>
  bool operator()(const RankMap&, const Graph&)
  {
    return n-- == 0;
  }

 private:
  std::size_t n;
};

namespace detail {

///////////////////////////////////////////////////////////////////////////
/// @brief Performs a step of the page rank algorithm on directed
/// directed graph.
/// @param g The directed input graph.
/// @param from_rank The previous rank map.
/// @param to_rank The object to store the next rank map in.
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template<typename Graph, typename RankMap, typename RankMap2>
void page_rank_step(Graph& g, RankMap& from_rank, RankMap2& to_rank,
        typename RankMap::property_value_type damping)
{
  typedef typename Graph::vertex_iterator VI;
  typedef typename Graph::adj_edge_iterator AEI;

  typedef typename RankMap::property_value_type rank_type;

  // Set new rank maps
  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    // put (1-damping) to all v's to_rank map
    to_rank.put(*vi, rank_type(1 - damping));
  }

  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    rank_type u_rank_out = damping * from_rank.get(*vi) / (*vi).size();
    /*FORALL_ADJ_T(u, v, g, Graph)*/
    for (AEI ei = (*vi).begin(); ei != (*vi).end(); ++ei) {
      to_rank.put(g.find_vertex((*ei).target()),
                  to_rank.get(*vi) + u_rank_out);
    }
  }
}

/*
 * for UnDirected graphs
 */
/*
template<typename Graph, typename RankMap, typename RankMap2>
void page_rank_step(Graph& g, RankMap& from_rank, RankMap2& to_rank,
        typename RankMap::value_type damping) {
  typedef typename Graph::vertex_iterator VI;
  typedef typename Graph::adj_edge_iterator AEI;

  typedef typename RankMap::value_type damping_type;

  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    typename RankMap::value_type rank(0);
    for (AEI ei = (*vi).begin(); ei != (*vi).end(); ++ei) {
      //FORALL_INEDGES_T(v, e, g, Graph)
      rank += from_rank.get(g.find_vertex((*ei).target())) /
                            g.find_vertex((*ei).target()).size();
      to_rank.put(vi, (damping_type(1) - damping) + damping * rank);
    }
  }
}
*/
} //namespace detail

///////////////////////////////////////////////////////////////////////////
/// @brief A function to run the PageRank algorithm on an input graph.
/// PageRank is a link analysis algorithm that attempts to assess the relative
/// importance of each element in a set.
/// @param g The input graph.
/// @param rank_map The rank map where the final rankings of each vertex will
/// be stored.
/// @param done A function that will return true when the algorithm is done.
/// @param damping The damping factor for the PageRank.
/// @param n The number of vertices in the graph.
/// @param rank_map2 An intermediate rank map used to calculate the rankings
/// during each iteration.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template<typename Graph, typename RankMap, typename Done, typename RankMap2>
void page_rank(Graph& g, RankMap& rank_map, Done done,
         typename RankMap::property_value_type damping,
         size_t n,
         RankMap2& rank_map2)
{
  typedef typename Graph::vertex_iterator VI;
  typedef typename Graph::adj_edge_iterator AEI;

  typedef typename RankMap::property_value_type rank_type;

  rank_type initial_rank = rank_type(rank_type(1) / n);  // initial rank = 1/n

  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    rank_map.put(*vi, initial_rank);  // initialize all vertex ranks to 1/n
  };

  bool to_map_2 = true;
  while ((to_map_2 && !done(rank_map, g)) ||
         (!to_map_2 && !done(rank_map2, g))) {

    // do for either UG or DG
    if (to_map_2) {
      detail::page_rank_step(g, rank_map, rank_map2, damping);
    } else {
      detail::page_rank_step(g, rank_map2, rank_map, damping);
    }
    to_map_2 = !to_map_2;
  }

  if (!to_map_2) {
    for (VI vi = g.begin(); vi != g.end(); ++vi) {
      rank_map.put(*vi, rank_map2.get(*vi));
    }
  }
}


///////////////////////////////////////////////////////////////////////////
/// @brief A function to run the PageRank algorithm on an input graph.
/// PageRank is a link analysis algorithm that attempts to assess the relative
/// importance of each element in a set. No second rank map is provided, so
/// a default one is used.
/// @param g The input graph.
/// @param rank_map The rank map where the final rankings of each vertex will
/// be stored.
/// @param done A function that will return true when the algorithm is done.
/// @param damping The damping factor for the PageRank.
/// @param n The number of vertices in the graph.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template<typename Graph, typename RankMap, typename Done>
void page_rank(Graph& g, RankMap& rank_map, Done done,
         typename RankMap::property_value_type damping,
         size_t n)
{
  typedef RankMap::property_value_type value_type;

  vector_property_map<Graph, value_type> ranks2; //(/*g.get_num_vertices()*/);
  //std::vector<rank_type> ranks2(g.get_num_vertices());
  // vector_rank_map<rank_type> ranks2(g.get_num_vertices());
  page_rank(g, rank_map, done, damping, n,
            ranks2);
}


///////////////////////////////////////////////////////////////////////////
/// @brief A function to run the PageRank algorithm on an input graph.
/// PageRank is a link analysis algorithm that attempts to assess the relative
/// importance of each element in a set. No second rank map is provided, so
/// a default one is used.
/// @param g The input graph.
/// @param rank_map The rank map where the final rankings of each vertex will
/// be stored.
/// @param done A function that will return true when the algorithm is done.
/// @param damping The damping factor for the PageRank.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template<typename Graph, typename RankMap, typename Done>
void page_rank(Graph& g, RankMap& rank_map, Done done,
          typename RankMap::property_value_type damping = 0.85)
{
  page_rank(g, rank_map, done, damping, g.get_num_vertices());
}

///////////////////////////////////////////////////////////////////////////
/// @brief A function to run the PageRank algorithm on an input graph.
/// PageRank is a link analysis algorithm that attempts to assess the relative
/// importance of each element in a set. No second rank map is provided, so
/// a default one is used. No "done" function is provided, so a done function
/// that returns true after 20 iterations is used. No damping factor is
/// provided, so a default value of 0.85 is used.
/// @param g The input graph.
/// @param rank_map The rank map where the final rankings of each vertex will
/// be stored.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template<typename Graph, typename RankMap>
void page_rank(Graph& g, RankMap& rank_map)
{
  page_rank(g, rank_map, n_iterations(20));
}

///////////////////////////////////////////////////////////////////////////
/// @brief Removes the vertices in a mutable graph that have a size of zero,
/// where the size is the degree of the vertex.
/// @param g The mutable, input graph.
/// @todo this could be _much_ more efficient, using a queue to store
/// the vertices that should be reprocessed and keeping track of which
/// vertices are in the queue with a property map. This only applies when we
/// have a bidirectional graph.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template<typename MutableGraph>
void remove_dangling_links(MutableGraph& g)
{
  size_t old_n;
  do {
    old_n = g.get_num_vertices();

    typename MutableGraph::vertex_iterator vi, vi_end;
    vi = g.begin(); vi_end = g.end();
    for (; vi != vi_end; ++vi) {
      if ((*vi).size() == 0) {
        g.delete_vertex((*vi).descriptor());
      }
    }
  } while (g.get_num_vertices() < old_n);
}

} //namespace sequential
} //namespace stapl

#endif
