/////////////////////////////////////////////////////////////////////
/**@file RegionGraph.h
  *
  * Extends STAPL pgraph to be used as a mesh/grid like graph needed in
  *    parallel PRM 
  *   
  *
  *    The user must provide:
  *    -- the parameter type BoundaryInfo, which is the data type 
  *       (e.g., a pair of boundary dimension and candidate VIDs in each region stored in each vertex of the graph.
  *    -- the parameter type WEIGHT, could be of no type
  *      
  *
  *    Each graph vertex is assigned a unique vertex identifier (VID).by STAPL pGraph 
  *
  * @date 03/01/2011
  *
  */
/////////////////////////////////////////////////////////////////////

#ifndef REGIONGRAPH_H_
#define REGIONGRAPH_H_

////////////////////////////////////////////////////////////////////////////////////////////

// Parallel Region Graph 
template <class VERTEX, class WEIGHT>
class RegionGraph{
public:
#ifdef _PARALLEL
  typedef typename stapl::p_graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, VERTEX, WEIGHT> REGIONPGRAPH;
  typedef typename REGIONPGRAPH::vertex_descriptor VID;
#endif
typedef typename stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, VERTEX, WEIGHT> REGIONGRAPH;

typedef typename REGIONGRAPH::vertex_descriptor VID;
typedef typename REGIONGRAPH::vertex_iterator VI;
typedef typename REGIONGRAPH::const_vertex_iterator CVI;
typedef typename REGIONGRAPH::adj_edge_iterator EI;
};
#endif
