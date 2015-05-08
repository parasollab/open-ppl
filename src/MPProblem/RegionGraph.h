#ifndef REGION_GRAPH_H_
#define REGION_GRAPH_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief Parallel Region Graph
/// @tparam VERTEX Region type
/// @tparam WEIGHT Weight type
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class RegionGraph {
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
