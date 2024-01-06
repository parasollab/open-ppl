/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_GENERATORS_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_GENERATORS_HPP

#include <boost/random/uniform_int.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/plod_generator.hpp>
#include <boost/graph/erdos_renyi_generator.hpp>
#include <boost/graph/small_world_generator.hpp>

namespace stapl {
namespace sequential{

///////////////////////////////////////////////////////////////////////////
/// @brief Builds a 2-D torus in a graph, based on parameters for the x and
/// y dimensions.
/// @param g The graph object where the torus will be represented.
/// @param nx Number of vertices on the x dimension of the torus.
/// @param ny Number of vertices on the y dimension of the torus.
/// @param bidirectional Specifies whether the graph will be bidirectional.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_torus(Graph& g, size_t nx, size_t ny, bool bidirectional = false)
{
  for (size_t i=0;i < nx*ny; ++i)
    g.add_vertex();

  for (size_t i=0;i < ny; ++i) {
    for (size_t j=0;j < nx; ++j) {
      size_t s = (i*nx) + j;
      size_t t1 = (i*nx) + (j+1)%nx;
      size_t t2 = (((i+1)%ny)*nx) + j;

      g.add_edge(s, t1);
      g.add_edge(s, t2);
    }
  }

  if (g.get_num_edges() != (2*nx*ny)) {
    std::cerr << "ASSERT: build_torus failed - "
              << "Incorrect number of edges in torus.\n";
    exit(1);
  }
  return;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Builds a mesh in a graph based on parameters for the x and y
/// dimensions.
/// @param g The graph object where the mesh will be stored.
/// @param nx The x dimension of the mesh.
/// @param ny The y dimension of the mesh.
/// @param bidirectional Specifies whether or not the mesh should be
/// bidirectional.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_mesh(Graph& g, size_t nx, size_t ny, bool bidirectional=false)
{
  for (size_t i=0;i < nx*ny; ++i)
    g.add_vertex();

  //visit each vertex except the right-most column and add edges to the right
  for (size_t i=0;i < nx-1; ++i) {
    for (size_t j=0;j < ny; ++j) {
      size_t source = (j*nx) + i;
      size_t target = source + 1; //adds 'right'
      g.add_edge(source, target);
      if (bidirectional && Graph::d_type == DIRECTED)
        g.add_edge(target, source);
    }
  }

  //now visit each vertex except the bottom-most column and add edges downwards
  for (size_t i=0;i < nx; ++i) {
    for (size_t j=0;j < ny-1; ++j) {
      size_t source = (j*nx) + i;
      size_t target = source + nx; //adds 'down'
      g.add_edge(source, target);
      if (bidirectional && Graph::d_type == DIRECTED)
        g.add_edge(target, source);
    }
  }

  if (g.get_num_edges() != (2*nx*ny - nx - ny)) {
    std::cerr<<"ASSERT: build_mesh - Invalid number eges in mesh.\n";
    exit(1);
  }

  return;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Builds a random graph based on the number of vertices and edges
/// specified.
/// @param g The graph object where the random graph will be stored.
/// @param num_vertices The number of vertices that the graph should have.
/// @param num_edges The number of edges that the graph should have.
/// @param seed The random seed to be used.
/// @param self_edges Specifies whether or not the graph should contain self
/// edges.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_random_graph(Graph& g, size_t num_vertices,
                        size_t num_edges, size_t seed = 42,
                        bool self_edges = false)
{
  if (num_vertices == 1) return;
  if (Graph::m_type == NONMULTIEDGES) {
    size_t allowed_edges = num_vertices*(num_vertices-1);
    if (Graph::d_type == UNDIRECTED) allowed_edges /= 2;
    if (allowed_edges < num_edges) {
      std::cerr<<"build_random_graph: Requested too many edges.\n";
      std::cerr<<"build_random_graph: "<<num_edges<<" requested, ";
      std::cerr<<"but only "<<allowed_edges<<" are possible.\n";
      return;
    }
  }

  typedef boost::mt19937                                  gen_type;
  typedef boost::uniform_int<size_t>                      dist_type;
  typedef boost::variate_generator<gen_type&, dist_type>  rand_gen_type;
  gen_type gen;
  gen.seed(seed);
  dist_type dist(0, num_vertices - 1); //closed interval
  rand_gen_type rand_gen(gen, dist);

  for (size_t i=0; i<num_vertices; ++i)
    g.add_vertex();

  //now add edges between random vertices in the graph.
  size_t s, t;
  while (g.get_num_edges() < num_edges) {
    s = rand_gen();
    do { t = rand_gen(); }
    while (!self_edges && s==t);
    g.add_edge(s,t);
  }

  return;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Builds a complete graph based on the number of vertices specified.
/// @param g The graph object where the complete graph will be stored.
/// @param num_vertices The number of vertices in the complete graph.
/// @param bidirectional Specifies whether or not the graph should be
/// bidirectional.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_complete_graph(Graph& g, size_t num_vertices,
                          bool bidirectional=false)
{
  for (size_t i=0; i<num_vertices; ++i)
    g.add_vertex();

  //add an edge from each local vertex to every other vertex in the graph
  //except for itself.
  for (size_t i=0; i<num_vertices; ++i) {
    for (size_t j=i+1; j<num_vertices; ++j) {
      g.add_edge(i,j);
      if (bidirectional && Graph::d_type == DIRECTED)
        g.add_edge(j,i);
    }
  }

  return;
}


///////////////////////////////////////////////////////////////////////////
/// @brief Builds an Erdos-Renyi graph using the Boost Graph Library
/// Erdos-Renyi generator.
/// @param g The graph object where the Erdos-Renyi model will be stored.
/// @param num_vertices The number of vertices for the graph.
/// @param probability The probability that any pair of nodes will have an
/// edge between them.
/// @param seed The seed for the random generator.
/// @param self_edges Specifies whether or not the graph should have
/// self edges.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_erdos_renyi_graph(Graph& g,
                             size_t num_vertices,
                             double probability,
                             size_t seed = 42,
                             bool self_edges = false)
{
  typedef boost::adjacency_list<> BGL_TYPE;
  typedef boost::mt19937 gen_type;
  typedef boost::erdos_renyi_iterator<gen_type, BGL_TYPE> ERGen;

  for (size_t i=0; i<num_vertices; ++i)
    g.add_vertex();

  gen_type gen;
  gen.seed(seed);
  ERGen it, it_end;
  it = ERGen(gen, num_vertices, probability, self_edges);
  it_end = ERGen();
  //for NONMULTIEDGE graphs, this might result in fewer edges than intended.
  while (it != it_end) {
    g.add_edge((*it).first, (*it).second);
    it++;
  }

  return;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Constructs a graph where the out-degrees of the vertices follow the
/// power law. Uses the Boost Graph Library's plod_iterator.
/// @param g The graph object where the graph will be stored.
/// @param num_vertices The number of vertices for the graph.
/// @param alpha The alpha parameter for the Power Law Out Degree (PLOD)
/// algorithm.
/// @param beta The beta parameter for the PLOD algorithm.
/// @param seed The seed for the random generator.
/// @param self_edges Specifies whether or not the graph should have self
/// edges.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_power_law_graph(Graph& g,
                           size_t num_vertices,
                           double alpha,
                           double beta,
                           size_t seed = 42,
                           bool self_edges = false)
{
  for (size_t i=0; i<num_vertices; ++i)
    g.add_vertex();

  typedef boost::mt19937 gen_type;
  gen_type gen;
  gen.seed(seed);
  //split on Directed vs. Undirected since the boost iterator specializes
  //on those cases
  if (Graph::d_type == DIRECTED) {
    typedef boost::adjacency_list<>                   BGL_TYPE;
    typedef boost::plod_iterator<gen_type, BGL_TYPE>  PLGen;

    PLGen it, it_end;
    it = PLGen(gen, num_vertices, alpha, beta, self_edges);
    it_end = PLGen();
    //for NONMULTIEDGE graphs, this might result in fewer edges than intended.
    while (++it != it_end) { //++ first or the first iteration will get (0,0)
      g.add_edge((*it).first, (*it).second);
    }
  }
  else { //is undirected.
    typedef boost::adjacency_list<boost::vecS,boost::vecS,
                                  boost::undirectedS> BGL_TYPE;

    typedef boost::plod_iterator<gen_type, BGL_TYPE>  PLGen;

    PLGen it, it_end;
    it = PLGen(gen, num_vertices, alpha, beta, self_edges);
    it_end = PLGen();
    //for NONMULTIEDGE graphs, this might result in fewer edges than intended.
    while (it != it_end) {
      g.add_edge((*it).first, (*it).second);
      it++;
    }
  }
  return;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Constructs a connected Power Law Out Degree (PLOD) graph. Uses the
/// Boost Graph Library's plod_iterator.
/// @param g The graph object where the graph will be stored.
/// @param num_vertices The number of vertices for the graph.
/// @param alpha The alpha parameter for the Power Law Out Degree (PLOD)
/// algorithm.
/// @param beta The beta parameter for the PLOD algorithm.
/// @param seed The seed for the random generator.
/// @param self_edges Specifies whether or not the graph should have self
/// edges.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_cplod_graph(Graph& g,
                       size_t num_vertices,
                       double alpha,
                       double beta,
                       size_t seed = 42,
                       bool self_edges = false)
{
  build_power_law_graph(g,num_vertices,alpha,beta,seed,self_edges);

  //Build a circular chain.
  typename Graph::vertex_iterator vit, vit_end;
  vit = g.begin();
  vit_end = g.end();
  --vit_end;
  typename Graph::vertex_iterator src;
  while (vit != vit_end) {
    src = vit++;
    g.add_edge((*src).descriptor(),(*vit).descriptor());
  }
  g.add_edge((*vit).descriptor(),(*g.begin()).descriptor());

  return;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Constructs a Small World graph using the Boost Graph Library's
/// small_world_iterator.
/// @param g The graph object where the graph will be stored.
/// @param num_vertices The number of vertices for the graph.
/// @param k_neighbors Each vertex will be connected to its k_neighbors
/// nearest neighbors.
/// @param probability Specifies the probability with which edges are randomly
/// rewired.
/// @param seed The seed for the random generator.
/// @param self_edges Specifies whether or not the graph should have self
/// edges.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_small_world_graph(Graph& g,
                             size_t num_vertices,
                             size_t k_neighbors,
                             double probability,
                             size_t seed = 42,
                             bool self_edges = false)
{
  typedef boost::adjacency_list<> BGL_TYPE;
  typedef boost::mt19937          gen_type;
  typedef boost::small_world_iterator<gen_type, BGL_TYPE> SWGen;

  for (size_t i=0; i<num_vertices; ++i)
    g.add_vertex();

  gen_type gen;
  gen.seed(seed);
  SWGen it, it_end;
  it = SWGen(gen, num_vertices, k_neighbors, probability, self_edges);
  it_end = SWGen();
  //for NONMULTIEDGE graphs, this might result in fewer edges than intended.
  while (it != it_end) {
    g.add_edge((*it).first, (*it).second);
    it++;
  }

  return;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Constructs a star graph, where every vertex in the graph is
/// connected to a center vertex, and all edges in the graph are between the
/// center vertex and some other vertex.
/// @param g The graph object where the graph will be stored.
/// @param num_vertices The number of vertices for the graph.
/// @todo A test for this generator needs to be implemented.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
void build_star_graph(Graph& g, size_t num_vertices)
{
  for (size_t i=0; i<num_vertices; ++i)
    g.add_vertex();

  typename Graph::vertex_iterator it, it_end;
  it = g.begin();
  it_end = g.end();
  typename Graph::vertex_descriptor center = (*it++).descriptor();
  for (; it != it_end; ++it)
    g.add_edge(center, (*it).descriptor());

  return;
}

}//namespace sequential
}//namespace stapl

#endif
