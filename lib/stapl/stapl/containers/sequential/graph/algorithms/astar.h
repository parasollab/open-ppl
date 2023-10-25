/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ALGO_ASTAR_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ALGO_ASTAR_HPP

#include <vector>
#include <limits>
#ifndef __APPLE__
  #include "values.h"
#endif
#include "util/d_ary_heap.h"
#include "graph_algo_util.h"
#include "dijkstra.h"

namespace stapl{
namespace sequential{

//////////////////////////////////////////////////////////////////////////////
/// @brief Calculate an A* path between the source and target vertices.
/// @param g the graph
/// @param parentmap maps each vertex to the its shortest path vertex
/// @param fscore, gscore stores the distance from the source and projected
/// distance from the target for all vertices, respectively
/// @param weightmap map of the edge weights
/// @param source the source vertex of the path
/// @param goal the target vertex of the path
/// @param h the heuristic work function
/// @param comp the comparator work function
/// @param combine the combining work function
/// @param colormap the map for the color of each vertex
/// @param Q the queue that holds the calculated paths
/// @return Returns a SSSP tree where the edge weights are the cumulative
/// path lengths from the source.
/// @note The function call does not take an edge weight map. The
/// implementations access the graph's entire edge property directly, so they
/// must be internal. This means that the property on the edge must have the
/// appropriate compare and combine functors defined for it. In the default
/// case, this means '<' and '+' operators. So if the edge weights are some
/// class foo, foo+foo and foo<foo (returning bool) need to be defined, and the
/// result of foo+foo needs to be the property_value_type of the distance map
/// output parameter.
/// @note Additionally, given the nature of A*, a proper admissible heuristic
/// function needs to be provided. The heuristic must be monotonic towards the
/// goal node, and needs to be an underestimate of the actual path to the goal.
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////////
template <typename Graph, typename Comparator, typename CombineFunction,
         typename PMap, typename DMap, typename WMap,
         typename HeuristicEstimate, typename CMap, typename Queue>
void astar(Graph& g,
           PMap& parentmap,
           DMap& gscore,
           DMap& fscore,
           WMap& weightmap,
           typename Graph::vertex_descriptor source,
           typename Graph::vertex_descriptor goal,
           HeuristicEstimate& h,
           Comparator& comp,
           CombineFunction& combine,
           CMap& colormap,
           Queue& Q)
{
  typedef typename Graph::vertex_iterator   VI;
  typedef typename Graph::vertex_descriptor VD;
  typedef typename Graph::vertex_reference  VR;
  typedef typename Graph::adj_edge_iterator AEI;
  typedef typename WMap::property_value_type Weight;
  typedef typename CMap::property_value_type  color_type;
  typedef graph_color<color_type>             color_set;

  //enqueue start
  colormap.put(source, color_set::gray());
  Q.push(*g.find_vertex(source));

  while (!Q.empty()) {
    VR u = Q.top(); Q.pop();

    //goal is found, stop early
    if (u.descriptor() == goal)
      break;

    colormap.put(u, color_set::black());

    AEI ei_end = u.end();
    AEI ei = u.begin();

    //for each neighbor, update queue
    for (; ei!=ei_end; ei++) {
      VD v = (*ei).target();
      color_type color = colormap.get(v);

      //shortest path already found for this node
      if (color == color_set::black())
        continue;

      VI v_iter = g.find_vertex(v);

      //check to update queue
      //(not visited OR new gscore < gscore)
      Weight tentative = combine(gscore.get(u), weightmap.get(*ei));
      if (color == color_set::white() ||
          comp(tentative, gscore.get(*v_iter))){

        //new node or shorter path found. Update g and f score
        parentmap.put(*v_iter, u.descriptor());
        gscore.put(*v_iter, tentative);
        fscore.put(*v_iter,
            combine(tentative, h(v_iter->property())));

        //if new node, push. Otherwise update
        if (color == color_set::white()){
         colormap.put(v, color_set::gray());
         Q.push(*v_iter);
        }
        else {
          Q.update(*v_iter);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////
/// @brief Calculate an A* path between the source and goal vertices with a
/// default combine work function and no provided color map or queue.
/// @param g the graph
/// @param parentmap maps each vertex to the its shortest path vertex
/// @param fscore, gscore stores the distance from the source and projected
/// distance from the target for all vertices, respectively
/// @param weightmap map of the edge weights
/// @param source the source vertex of the path
/// @param goal the target vertex of the path
/// @param h the heuristic work function
/// @param comp the comparator work function
/// @return a SSSP tree where the edge weights are the cumulative
/// path lengths from the source
/// @note The combination work function is initialized to std::plus.
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap, typename DMap, typename WMap,
         typename HeuristicEstimate, typename Comparator>
void astar(Graph& g,
           PMap& parentmap,
           DMap& gscore,
           DMap& fscore,
           WMap& weightmap,
           typename Graph::vertex_descriptor source,
           typename Graph::vertex_descriptor goal,
           HeuristicEstimate& h,
           Comparator& comp)
{
  typedef typename Graph::vertex_iterator    VI;
  typedef typename Graph::vertex_reference   VR;
  typedef typename WMap::property_value_type Weight;
  typedef map_property_map<Graph, size_t>    index_map_t;

  map_property_map<Graph, size_t> colormap;
  index_map_t index_map;
  d_ary_heap_indirect<VR, 4, index_map_t, DMap, Comparator>
    queue(fscore, index_map, comp);

  std::plus<Weight> combine;
  VI s = g.find_vertex(source);
  gscore.put(*s, Weight());
  fscore.put(*s, combine(Weight(h(s->property())), Weight()));
  VI vi = g.begin(); VI vi_end = g.end();
  for (; vi != vi_end; ++vi) {
    VR v = *vi;
    parentmap.put(v, v.descriptor());
    colormap.put(v, graph_color<size_t>::white());
  }

  astar(g, parentmap, gscore, fscore, weightmap, source, goal, h,
      comp, combine, colormap, queue);
}

////////////////////////////////////////////////////////////////////////
/// @brief Calculate an A* path between the source and goal vertices with
/// default compare and combine work functions and no provided color map or
/// queue.
/// @param g the graph
/// @param parentmap maps each vertex to the its shortest path vertex
/// @param fscore, gscore stores the distance from the source and projected
/// distance from the target for all vertices, respectively
/// @param weightmap map of the edge weights
/// @param source the source vertex of the path
/// @param goal the target vertex of the path
/// @param h the heuristic work function
/// @return Returns a SSSP tree where the edge weights are the cumulative
/// path lengths from the source.
/// @note The compare work function is initialized to std::less and the
/// combination work function is initialized to std::plus.
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap, typename DMap, typename WMap,
         typename HeuristicEstimate>
void astar(Graph& g,
           PMap& parentmap,
           DMap& gscore,
           DMap& fscore,
           WMap& weightmap,
           typename Graph::vertex_descriptor source,
           typename Graph::vertex_descriptor goal,
           HeuristicEstimate& h)
{
  typedef typename WMap::property_value_type Weight;
  std::less<Weight> lsop;

  astar(g,parentmap,gscore, fscore,weightmap, source, goal, h, lsop);
}

//////////////////////////////////////////////////////////////////////
/// @brief Calculate an A* path between the source and goal vertices with
/// default compare and combine work functions and no provided weight map,
/// color map, or queue.
/// @param g the graph
/// @param parentmap maps each vertex to the its shortest path vertex
/// @param fscore, gscore stores the distance from the source and projected
/// distance from the target for all vertices, respectively
/// @param source the source vertex of the path
/// @param goal the target vertex of the path
/// @param h the heuristic work function
/// @return Returns a SSSP tree where the edge weights are the cumulative
/// path lengths from the source.
/// @note The compare work function is initialized to std::less and the
/// combination work function is initialized to std::plus.
/// @note Each value for the weight map is initialized to 1.
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap,
          typename DMap, typename HeuristicEstimate>
void astar(Graph& g,
           PMap& parentmap,
           DMap& gscore,
           DMap& fscore,
           typename Graph::vertex_descriptor source,
           typename Graph::vertex_descriptor goal,
           HeuristicEstimate& h)
{
  //no weightmap parameter; use map from VD to size_t, initialized to 1
  map_property_map<Graph, size_t> weightmap;

  typename Graph::vertex_iterator it, it_end = g.end();
  for (it = g.begin(); it != it_end; ++it)
    weightmap.put(*it, 1);

  std::less<size_t> lsop;
  astar(g, parentmap, gscore, fscore, weightmap, source, goal, lsop, h);
}

//////////////////////////////////////////////////////////////////////
/// @brief Calculate an A* path between the source and goal vertices with
/// default compare and combine work functions and no provided color map or
/// queue and store the vertices of the resulting path in a vector in order.
/// @param g the graph
/// @param wmap map of the edge weights
/// @param source the source vertex of the path
/// @param goal the target vertex of the path
/// @param v the vector in which to store the vertices of the resulting path
/// @param h the heuristic work function
/// @return Returns the size of the vector.
/// @note The compare work function is initialized to std::less and the
/// combination work function is initialized to std::plus.
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////////
template <typename Graph, typename EdgeMap, typename HeuristicEstimate>
size_t astar(Graph& g,
             EdgeMap& wmap,
             typename Graph::vertex_descriptor source,
             typename Graph::vertex_descriptor goal,
             std::vector<typename Graph::vertex_descriptor>& v,
             HeuristicEstimate& h)
{
  typedef typename Graph::vertex_descriptor VD;
  typedef map_property_map<Graph, VD> pred_map_t;
  typedef map_property_map<Graph, typename EdgeMap::property_value_type>
            dist_map_t;
  pred_map_t parentmap;
  dist_map_t gscore;
  dist_map_t fscore;
  astar(g, parentmap, gscore, fscore, wmap, source, goal, h);
  if (parentmap.get(goal) == goal) return 0;//there is no path
  //now prepare the return datastructure (the path)
  VD temp = goal;
  //push the last element
  v.push_back(goal);
  while (parentmap.get(temp) != source){
    v.push_back(parentmap.get(temp));
    temp = parentmap.get(temp);
  }
  v.push_back(parentmap.get(temp));
  std::reverse(v.begin(), v.end());
  return v.size();
}

////////////////////////////////////////////////////////////////////
/// @brief Calculate an A* path between the source and goal vertices with
/// default compare and combine work functions and no provided edge map,
/// color map or queue and store the vertices of the resulting path in a
/// vector in order.
/// @param g the graph
/// @param source the source vertex of the path
/// @param goal the target vertex of the path
/// @param v the vector in which to store the vertices of the resulting path
/// @param h the heuristic work function
/// @return Returns the size of the vector.
/// @note The compare work function is initialized to std::less and the
/// combination work function is initialized to std::plus.
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////////
template <typename Graph, typename HeuristicEstimate>
size_t astar(Graph& g,
             typename Graph::vertex_descriptor source,
             typename Graph::vertex_descriptor goal,
             std::vector<typename Graph::vertex_descriptor>& v,
             HeuristicEstimate& h)
{
  edge_property_map<Graph> wmap(g);
  return astar(g, wmap, source, goal, v, h);
}

///////////////////////////////////////////////////////////////////////////
/// @brief Calculate an A* path between the source and goal vertices with
/// default compare and combine work functions and no provided edge map, color
/// map or queue and store the vertices of the resulting path and their
/// respective edge properties as a pair in a vector in order.
/// @param g the graph
/// @param source the source vertex of the path
/// @param goal the target vertex of the path
/// @param v the vector in which to store the vertices of the resulting path
/// @param h the heuristic work function
/// @return Returns the size of the vector.
/// @note The compare work function is initialized to std::less and the
/// combination work function is initialized to std::plus.
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////////
template <typename Graph, typename HeuristicEstimate>
size_t astar(Graph& g,
             typename Graph::vertex_descriptor source,
             typename Graph::vertex_descriptor goal,
             std::vector< std::pair<typename Graph::vertex_property,
             typename Graph::edge_property> >& v,
             HeuristicEstimate& h)
{
  typedef typename Graph::vertex_descriptor VD;
  std::vector<VD> spath;
  size_t res = astar(g,source, goal, spath, h);
  if (res > 0){
    //convert vd to property
    typename Graph::vertex_property vp;
    typename Graph::edge_property ep;
    for (size_t i=0;i<spath.size()-1;++i){
      vp = get_vertex_data(g,spath[i]);
      ep = get_edge_data(g,spath[i],spath[i+1]);
      v.push_back(std::pair<typename Graph::vertex_property,
          typename Graph::edge_property>(vp,ep));
    }
    vp = get_vertex_data(g,spath[spath.size()-1]);
    v.push_back(std::pair<typename Graph::vertex_property,
        typename Graph::edge_property>(vp,ep));
  }
  return res;
}

}//end namespace sequential
}//end namespace stapl

#endif
