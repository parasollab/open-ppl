/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_SCC_ALGO_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_SCC_ALGO_HPP

#include <deque>
#include "depth_first_search.h"
#include "graph_algo.h"

namespace stapl{

namespace sequential{

///////////////////////////////////////////////////////////////////////////
/// @brief Calculate the 'time' needed to finish computation on each vertex.
/// @tparam VGraph the graph view
/// @note Here we refer to 'time' as the number of iterations used during
/// execution, not the wall clock time.
/// @note @p m_finish is incremented during the execution of the algorithm and
/// the current value is matched to a vertex when the vertex is finished.
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
///////////////////////////////////////////////////////////////////////////
template <class VGraph>
class visitor_compute_finish_time
        : public visitor_base<VGraph>
{
  typedef std::vector<std::pair<typename VGraph::vertex_descriptor,size_t> > FT;
  /// a vector of pairs that list each vertex with its respective finish time
  FT& m_finishtimes;
  /// the amount of 'time' elapsed during execution
  size_t m_finish;
  /// the number of vertices in the graph
  size_t m_sz;

 public:
  visitor_compute_finish_time(VGraph& g, FT& v)
    : m_finishtimes(v)
  {
    m_sz = g.get_num_vertices();
    m_finishtimes.resize(m_sz);
    m_finish = 0;
  }

  visitor_return finish_vertex(typename VGraph::vertex_iterator vi, int =-1)
  {
    m_finishtimes[m_sz-m_finish-1]=
      std::pair<typename VGraph::vertex_descriptor,size_t>
        ((*vi).descriptor(),m_finish);
    ++m_finish;
    return CONTINUE;
  }
};


// visitor for when going backwards

////////////////////////////////////////////////////////////////////////////
/// @brief Add a discovered vertex to its strongly-connected component.
/// @tparam VGraph the graph view
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
////////////////////////////////////////////////////////////////////////////
template <class VGraph>
class visitor_compute_scc
        : public visitor_base<VGraph>
{
  typedef std::vector<typename VGraph::vertex_descriptor> vscc_type;
  /// a vector of vertex descriptors in a strongly-connected component
  vscc_type& m_vscc;
 public:
  visitor_compute_scc(vscc_type& vscc)
    : m_vscc(vscc)
  { }

  visitor_return discover_vertex(typename VGraph::vertex_iterator vi)
  {
    m_vscc.push_back((*vi).descriptor());
    return CONTINUE;
  }
};


// scc helper for views without predecessors info

//////////////////////////////////////////////////////////////////////////////
/// @brief Compute the strongly-connected components of the graph by building
/// the reverse graph and performing a breadth-first search on it.
/// @tparam VGraph the graph view
/// @tparam ColorMap the map of colors for each vertex
/// @tparam b Boolean value indicating whether the graph is bidirectional
/// @ingroup seqGraphUtil
/////////////////////////////////////////////////////////////////////////////
template <class VGraph, class ColorMap, bool b>
struct scc_helper
{
  static void compute(VGraph& g,
                      ColorMap& cmap,
                      std::vector<
                        std::vector<typename VGraph::vertex_descriptor>
                      >& sccs)
  {

    typedef std::pair<typename VGraph::vertex_descriptor,size_t > scc_info;

    std::vector<scc_info> ft;
    visitor_compute_finish_time<VGraph> vis(g,ft);
    depth_first_search(g, vis, cmap);

    //here build the reverse graph
    typedef typename ColorMap::property_value_type color_type;
    typedef graph_color<color_type>                color_set;
    typedef graph<DIRECTED,MULTIEDGES,color_type>  RG_type;
    RG_type rg;
    build_reverse_graph (g, rg);

    vertex_property_map<RG_type> rcmap(rg);
    rcmap.reset();
    ///perform bfs on reverse graph
    for (size_t i=0;i<ft.size();i++){
      typename RG_type::vertex_iterator vi =
        rg.find_vertex((size_t)(ft[i].first));
      if (rcmap.get(*vi) == color_set::white()){
        //dfs through predecessors
        typedef typename std::vector<typename RG_type::vertex_descriptor>
                           TempVecType;

        TempVecType temp_vec;

        visitor_compute_scc<RG_type> rvis(temp_vec);
        depth_first_search(rg, (size_t)(ft[i].first), rvis, rcmap);

        std::vector<typename VGraph::vertex_descriptor> scc;
        for (auto&& i : temp_vec) {
          scc.push_back(i);
        }
        sccs.push_back(scc);
      }
    }
  }
};

//if static bidirectional view

///////////////////////////////////////////////////////////////////////////////
/// @brief If the graph is bidirectional, do nothing.
/// @tparam VGraph the graph view
/// @tparam ColorMap the map of colors for each vertex
/// @ingroup seqGraphUtil
///////////////////////////////////////////////////////////////////////////////
template <class VGraph, class ColorMap>
struct scc_helper<VGraph,ColorMap,true>
{
  static void compute(VGraph&,
                      ColorMap&,
                      std::vector<
                        std::vector<typename VGraph::vertex_descriptor>
                      >&)
  { }
};

//////////////////////////////////////////////////////////////////////////////
/// @brief Call the work function that computes the strongly-connected
/// components of the graph.
/// @param g the graph
/// @param cmap the color map for the graph
/// @param scc a vector of vectors that lists all of the vertices in each
/// strongly-connected component
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////////
template <class VGraph, class ColorMap>
void scc(VGraph& g,
         ColorMap& cmap,
         std::vector<std::vector<typename VGraph::vertex_descriptor> >& scc)
{
  scc_helper<VGraph, ColorMap,is_bidirectional_view<VGraph>::value>
    ::compute(g, cmap, scc);
}

}//namespace sequential
}//namespace stapl

#endif
