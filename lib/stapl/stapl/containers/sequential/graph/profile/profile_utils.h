/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_PROFILE_UTILS_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_PROFILE_UTILS_HPP

#include <fstream>
#include <cstring>
#include <iostream>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/type_traits/is_same.hpp>

// Supress deprecated warnings in boost/graph/adjacency_list.hpp (uses auto_ptr)
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <boost/graph/adjacency_list.hpp>

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <boost/random/mersenne_twister.hpp>
#include "../algorithms/graph_algo_util.h"
#include "../algorithms/graph_generators.h"

//shared utils
namespace stapl {

template<typename T>
void print_vector(std::string name, std::vector<T>& vec) {
  std::cout<<"print_vector called for an unknown type.\n";
}

void print_vector(std::string name, std::vector<size_t>& vec) {
  for(size_t i=0; i < vec.size(); ++i)
    std::cout<<name<<".at("<<i<<") == "<<vec.at(i)<<std::endl;
}

void print_vector(std::string name, std::vector<std::pair<size_t, size_t> >& vec) {
  for(size_t i=0; i < vec.size(); ++i)
    std::cout<<name<<".at("<<i<<") == ("<<vec.at(i).first<<", "<<vec.at(i).second<<")\n";
}

enum graph_generator_type {
  random,
  complete,
  mesh,
  smesh,
  torus,
  plod,
  cplod,
  small_world,
  erdos_renyi,
  star
};

/**
 * Centralized class that holds runtime profiler configuration info
 * as well as 'collapsed graphs', if necessary. For information about
 * the individual parameters, see the member area of the class.
 */
class profile_config {
private:
//inner function calls for configure()
  void parse_options(int argc, char** argv) {
    for(int i = 1; i < argc; i++) {
      if(!strcmp("--num-verts", argv[i]) || !strcmp("--n", argv[i]))
        m_num_verts = atoi(argv[++i]);

      if(!strcmp("--edge-factor", argv[i]) || !strcmp("--ef", argv[i]))
        m_ef=atof(argv[++i]);

      if(!strcmp("--seed", argv[i]) || !strcmp("--s", argv[i]))
        m_seed = atoi(argv[++i]);

      if(!strcmp("--alpha", argv[i]) || !strcmp("--a", argv[i]))
        m_alpha = atof(argv[++i]);

      if(!strcmp("--beta", argv[i]) || !strcmp("--b", argv[i]))
        m_beta = atof(argv[++i]);

      if(!strcmp("--k-neighbors", argv[i]) || !strcmp("--k", argv[i]))
        m_k_neighbors = atof(argv[++i]);

      if(!strcmp("--probability", argv[i]) || !strcmp("--p",argv[i]))
        m_probability = atof(argv[++i]);

      if(!strcmp("--edge-factor-range", argv[i]) || !strcmp("--efr", argv[i])) {
        double min_fac = atof(argv[++i]);
        double max_fac = atof(argv[++i]);
        if((min_fac <= max_fac) && (min_fac > 0)) {
          m_min_edges = m_num_verts*min_fac;
          m_max_edges = m_num_verts*max_fac;
          m_efr = true;
        } else m_efr = false; //invalid range
      }

      if(!strcmp("--vdc", argv[i]))
        m_vdc = atoi(argv[++i]);

      if(!strcmp("--edc", argv[i])) {
        m_edc = atoi(argv[++i]);
        m_edc_defined = true;
      }

      if(!strcmp("--edp", argv[i]) && !m_edc_defined)
          m_edp = atof(argv[++i]);

      if(!strcmp("--strided", argv[i]))
        m_strided = true;
    }
  }

  void parse_graph_type(int argc, char** argv) {
    for(int i = 1; i < argc; i++) {
      if(!strcmp("--random", argv[i])) m_type = random;

      if(!strcmp("--complete", argv[i])) m_type = complete;

      if(!strcmp("--mesh", argv[i])) m_type = mesh;

      //smesh==square mesh; use N to make an N x N mesh.
      if(!strcmp("--smesh", argv[i])) m_type = smesh;

      if(!strcmp("--torus", argv[i])) m_type = torus;

      //erdos-renyi
      if(!strcmp("--er", argv[i])) m_type = erdos_renyi;

      if(!strcmp("--plod", argv[i])) m_type = plod;

      if(!strcmp("--cplod", argv[i])) m_type = cplod;

      //small world
      if(!strcmp("--sw", argv[i])) m_type = small_world;

      if(!strcmp("--star", argv[i])) m_type = star;
    }
  }

public:
  typedef std::vector<size_t>                     verts_type;
  typedef std::vector<std::pair<size_t, size_t> > edges_type;

  profile_config()
    : m_argc(0), m_argv(0), m_strided(false), m_efr(false), m_num_verts(5000), m_seed(time(NULL)),
      m_min_edges(1), m_max_edges(1000), m_vdc(30), m_edc(10000), m_edp(40),
      m_ef(5), m_alpha(2.72), m_beta(((double)1000) * 1.5), m_k_neighbors(6),
      m_probability(0.05), m_type(random), m_edc_defined(false) {}

  //TODO: Fill this out, explaining each parameter.
  static void print_help_message() {
    return;
  }

  /**
   * Parses the given arguments (presumably from the command-line)
   * and stores the results in the config for use by the various profilers.
   * Also stores the given args (argc,argv).
   *
   * NOTE: YOU WANT TO CALL THIS BEFORE USING CONFIG FOR ANYTHING ELSE.
   * Else the default params are used. You probably don't want those.
   */
  void configure(int argc, char** argv) {
    parse_options(argc, argv);
    parse_graph_type(argc, argv);
    m_argc = argc;
    m_argv = argv;
  }

  //take a graph and store its vertex/edge info in the config. Useful for
  //the graph experiments where a graph is reconstructed each iteration
  //(see rebuild_graph functions).
  template<typename Graph>
  void store_graph(Graph& g) {
    m_verts.clear(); m_verts.reserve(g.get_num_vertices());
    m_edges.clear(); m_edges.reserve(g.get_num_edges());

    typename Graph::vertex_iterator it, it_end = g.end();
    for(it = g.begin(); it != it_end; ++it)
      m_verts.push_back((*it).descriptor());

    typename Graph::edge_iterator ei, ei_end = g.edges_end();
    for(ei = g.edges_begin(); ei != ei_end; ++ei)
      m_edges.push_back(std::make_pair((*ei).source(), (*ei).target()));

    return;
  }

  //this should only be called if there's a graph being stored in the object,
  //which is currently only done during the graph method experiments.
  void build_deletion_lists() {
    m_vert_del_list.clear();
    m_edge_del_list.clear();

    if(!m_edc_defined) m_edc = (size_t)(m_edges.size() * m_edp / 100);

    if(!m_strided) { //delete random vertices/edges
      typedef boost::mt19937                               gen_type;
      typedef boost::uniform_int<size_t>                   dist_type;
      typedef boost::variate_generator<gen_type,dist_type> rnd_type;

      size_t seed = time(NULL);
      gen_type gen;
      gen.seed(seed);
      dist_type dist(0, m_verts.size() - 1);
      rnd_type rnd(gen,dist);
      std::cout<<"Generating random vertex and edge deletion lists using these params:\n";
      std::cout<<"VDC: "<<m_vdc<<"\nEDC: "<<m_edc<<"\nSeed: "<<seed<<"\n\n";

      verts_type verts(m_verts.begin(), m_verts.end());
      std::random_shuffle(verts.begin(),verts.end(),rnd);
      verts.resize(m_vdc);
      m_vert_del_list = verts;

      edges_type edges(m_edges.begin(), m_edges.end());
      std::random_shuffle(edges.begin(),edges.end(),rnd);
      edges.resize(m_edc);
      m_edge_del_list = edges;
    }
    else { //we'll delete in strides
      size_t vert_del_stride = m_verts.size() / m_vdc;
      size_t edge_del_stride = m_edges.size() / m_edc;

      std::cout<<"Generating strided vertex and edge deletion lists using these params:\n";
      std::cout<<"VDC: "<<m_vdc<<"\nEDC: "<<m_edc<<"\nVertex deletion stride: ";
      std::cout<<vert_del_stride<<"\nEdge deletion stride: "<<edge_del_stride<<"\n\n";

      for(size_t i=0; i < m_verts.size(); i+=vert_del_stride)
        m_vert_del_list.push_back(m_verts[i]);

      for(size_t i=0; i < m_edges.size(); i+=edge_del_stride)
        m_edge_del_list.push_back(m_edges[i]);
    }
  }

  //reconstructs in @g the graph stored by the config.
  template<typename Graph>
  void rebuild_graph(Graph& g) {
    for(size_t i = 0; i < m_verts.size(); ++i)
      g.add_vertex(m_verts[i], typename Graph::vertex_property());

    for(size_t i = 0; i < m_edges.size(); ++i)
      g.add_edge(m_edges[i].first, m_edges[i].second);

    return;
  }

  //using the BGL interface, reconstructs in @g the graph stored by the config
  template<typename Graph>
  void rebuild_bgl_graph(Graph& g) {
    for(size_t i=0; i<m_verts.size(); ++i)
      boost::add_vertex(g);

    typedef boost::graph_traits<Graph> traits;
    typedef typename traits::vertex_iterator   vi_type;
    typedef typename traits::vertex_descriptor vd_type;
    vi_type it,it_end;
    boost::tie(it,it_end) = boost::vertices(g);
    std::vector<vd_type> verts(it, it_end);

    typename edges_type::iterator eit, eit_end;
    eit_end = m_edges.end();
    for(eit = m_edges.begin(); eit != eit_end; ++eit)
      boost::add_edge(verts[eit->first], verts[eit->second], g);
  }

public:
  int    m_argc;
  char** m_argv;

  bool   m_strided;     //should vertex/edge deletions be strided (true) or random (false)?
  bool   m_efr;         //specify an edge factor range that random graphs should fall into.
  size_t m_num_verts;
  size_t m_seed;
  size_t m_min_edges;   //computed by using the lower bound of the EFR * num_verts
  size_t m_max_edges;   //similarly, upper bound * num_verts
  size_t m_vdc;         //how many vertices should vertex deletion profilers delete per iteration?
  size_t m_edc;         //edge deletion count for delete_edge profilers; takes precedence over --edp
  double m_edp;         //edge deletion percentage for delete_edge profilers
  double m_ef;          //|E| = N*EF, or y=EF (for mesh/torus)
  double m_alpha;       //power-law graph generator parameter
  double m_beta;        //power-law graph generator parameter
  double m_k_neighbors; //small-world graph generator parameter
  double m_probability; //probability param used in ER and small-world graphs.

  graph_generator_type m_type; //enum used to tell build_graph what type of graph to build
  verts_type m_verts;          //we store the verts in case we want to represent 'holes'
  edges_type m_edges;          //similarly, for edges.

  verts_type m_vert_del_list;  //vector of vertices to delete
  edges_type m_edge_del_list;  //similarly, for edges.

private:
  bool m_edc_defined;
};

//wrap around boost.random classes for use in std::random_shuffle
class rs_rand {
  typedef boost::mt19937                          gen_t;
  typedef boost::uniform_int<int>                 dist_t;
  typedef boost::variate_generator<gen_t, dist_t> rnd_t;
public:
  rs_rand(size_t _seed, size_t N)
    : m_gen(_seed), m_dist(0,N-1), m_rnd(m_gen, m_dist) {}

  void seed(size_t _seed) {
    m_gen.seed(_seed);
    m_rnd = rnd_t(m_gen,m_dist);
  }

  std::ptrdiff_t operator()(std::ptrdiff_t arg) {
    return static_cast<std::ptrdiff_t>(m_rnd(arg));
  }

private:
  gen_t   m_gen;
  dist_t  m_dist;
  rnd_t   m_rnd;
};

} //namespace stapl

//utils specific to profiling bgl
namespace stapl {

/**
 * Reads a graph from file and reconstructs it in the passed graph parameter.
 */
template <typename G>
std::vector<typename G::vertex_descriptor> read_from_file_bgl(G& g, std::string filename) {
  std::ifstream ifile;
  ifile.open(filename.c_str());
  size_t nverts = 0;
  ifile >> nverts;

  //because of BGL's changing semantics, have to hold VDs for edge construction.
  std::vector<typename G::vertex_descriptor> verts(nverts);
  for (size_t i=0; i<nverts; ++i)
    verts[i] = add_vertex(g);

  typedef int SVD;
  SVD s=0, t=0;

  while(!ifile.eof()) {
    ifile >> s >> t;
    while (t != -1) {
      add_edge(verts[s], verts[t], g);
      ifile >> t;
    }
  }
  ifile.close();

  return verts;
}

/**
 * Renames the VDs in a vector of BGL VDs so that when the
 * delete_vertex profiler uses the vector to perform deletion,
 * the 'same' vertices are deleted that would have been deleted
 * in the SSGL version of the profiler. This can be verified by
 * attaching size_t properties to the BGL vertices and checking
 * them before and after the deletions are performed.
 *
 * The first definition of the function is for when the boost
 * adjacency_list is storing vertices in a vector. The overload is
 * for when vertices are in a list, which requires no renaming since
 * list elements will not be physically shifted like in a vector when
 * deletion is performed...
 */
void rename_vertices(std::vector<size_t>& verts) {
  size_t num_verts = verts.size();
  for(size_t i=0; i < num_verts; ++i)
    for(size_t j=i; j < num_verts; ++j)
      if(std::less<size_t>()(verts[i],verts[j]))
        --(verts[j]);

  return;
}

//list case; do nothing.
void rename_vertices(std::vector<void*>&) {}

//generates random edge weights in [0,100). Expects the boost graph to have
//size_t edge properties, tagged with edge_weight_t.
template<typename G>
void propertize_bgl(G& g, size_t seed=42) {
  srand(seed);
  typename G::edge_iterator iter;
  for(iter = edges(g).first; iter != edges(g).second; ++iter)
    put(boost::edge_weight, g, *iter, rand()%100);
  return;
}

template<typename G>
void print_graph_bgl(G& g) {
/*
  typename G::vertex_iterator vi, vi_end;
  vi = vertices(g).first; vi_end = vertices(g).second;
  cout<<"BGL Vertices (vd,property):"<<endl;
  for(; vi != vi_end; ++vi)
    cout<<"("<<(*vi)<<", "<<get(vertex_distance,g,*vi)<<")"<<endl;
*/

  typename G::edge_iterator ei, ei_end;
  ei = edges(g).first; ei_end = edges(g).second;
  std::cout<<"\nBGL Edges (s,t,p):"<<std::endl;
  for(; ei != ei_end; ++ei)
    std::cout<<"("<<source(*ei,g)<<", "<<target(*ei,g)<<")\n";

  return;
}

//visitor class for testing BFS and DFS
template< class GRAPH>
class visitor_bgl {
  public:

  visitor_bgl() = default;
  visitor_bgl(size_t*) {}

  template < typename Vertex, typename Graph >
  void discover_vertex(Vertex u, Graph & g) {}

  template < typename Vertex, typename Graph >
  void examine_vertex(Vertex u, Graph & g) {}

  template < typename Vertex, typename Graph >
  void initialize_vertex(Vertex s, Graph& g) {}


  template < typename Edge, typename Graph >
  void examine_edge(Edge e, Graph& g) {}

  template<typename Vertex, typename Graph>
  void start_vertex(Vertex s, Graph& g) {}

  template<typename Edge, typename Graph>
  void back_edge(Edge e, Graph& g) {}

  template<typename Edge, typename Graph>
  void forward_or_cross_edge(Edge e, Graph& g) {}

  template < typename Edge, typename Graph >
  void tree_edge(Edge e, Graph& g) {}

  template < typename Edge, typename Graph >
  void non_tree_edge(Edge e, Graph& g) {}

  template < typename Edge, typename Graph >
  void black_target(Edge e, Graph& g) {}

  template < typename Edge, typename Graph >
  void gray_target(Edge e, Graph& g) {}

  template < typename Vertex, typename Graph >
  void finish_vertex(Vertex s, Graph& g) {}

};

}//namespace stapl

//utils specific to profiling stapl
namespace stapl {
template<>
struct graph_color<int> {
  typedef int value_type;
  static value_type white() {return 0;}
  static value_type gray()  {return 1;}
  static value_type black() {return 2;}
};

//helper used by build_graph
template<typename VR>
class _max_comp {
  typedef VR vertex_reference;
public:
  bool operator()(const VR& first, const VR& second) {
    if(first.size() < second.size()) return true;
    else return false;
  }
};

//using a variety of options given by the config object, this builds
//an SSGL graph.
template<typename Graph>
void build_graph(Graph& g, profile_config& config) {
  using std::cout; using std::endl;

  if(g.get_num_vertices() > 0)
    cout<<"in build_graph(): WARNING - Input graph is not empty.\n";

  //now build the graph
  bool   use_ef_range = config.m_efr;
  size_t N            = config.m_num_verts;
  size_t seed         = config.m_seed;
  size_t min_edges    = config.m_min_edges;
  size_t max_edges    = config.m_max_edges;
  double EF           = config.m_ef;
  double alpha        = config.m_alpha;
  double beta         = config.m_beta;
  double k_neighbors  = config.m_k_neighbors;
  double probability  = config.m_probability;
  if(config.m_type == random) {
    cout<<"Building random graph with:\n"<<N<<" vertices\n";
    cout<<N*EF<<" edges\nSeed: "<<seed<<endl<<endl;
    build_random_graph(g,N,N*EF,seed);
  }
  else if(config.m_type == complete) {
    cout<<"Building complete graph with "<<N<<" vertices.\n\n";
    build_complete_graph(g,N);
  }
  else if(config.m_type == mesh) {
    cout<<"Building mesh with:\n"<<N*EF<<" vertices.\n";
    cout<<"X = "<<N<<", Y = "<<(size_t)EF<<endl<<endl;
    build_mesh(g,N,(size_t)EF);
  }
  else if(config.m_type == smesh) {
    //smesh==square mesh; use N to make an N x N mesh.
    cout<<"Building square mesh with:\n"<<N*N<<" vertices.\n";
    cout<<"X = "<<N<<", Y = "<<N<<endl<<endl;
    build_mesh(g,N,N);
  }
  else if(config.m_type == torus) {
    cout<<"Building torus with:\n"<<N*EF<<" vertices.\n";
    cout<<"X = "<<N<<", Y = "<<(size_t)EF<<endl<<endl;
    build_torus(g,N,(size_t)EF);
  }
  else if(config.m_type == erdos_renyi) {
    cout<<"Building Erdos-Renyi graph with:\n"<<N<<" vertices\n";
    cout<<"Probability: "<<probability<<"\nSeed: "<<seed<<"\n\n";
    build_erdos_renyi_graph(g,N,probability,seed);
  }
  else if(config.m_type == plod) {
    cout<<"Building power law graph with:\n"<<N<<" vertices\n";
    cout<<"Alpha: "<<alpha<<"\nBeta: "<<beta<<"\n\n";
    build_power_law_graph(g,N,alpha,beta,seed);
    if(use_ef_range) {
      cout<<"Using |E| Range: "<<min_edges<<" - "<<max_edges<<"\n\n";
      size_t E = g.get_num_edges();
      while(E < min_edges || E > max_edges) {
        g.clear();
        seed = time(NULL);
        cout<<"."<<std::flush;
        build_power_law_graph(g,N,alpha,beta,seed);
        E = g.get_num_edges();
      }
    }
    cout<<"built.\nSeed: "<<seed<<endl;

    //print the highest out-degree
    typename Graph::vertex_iterator it, it_end;
    it = g.begin(); it_end = g.end();
    it = std::max_element(it,it_end,_max_comp<typename Graph::vertex_reference>());
    cout<<"Max out-degree: "<<(*it).size()<<endl;
  }
  else if(config.m_type == cplod) {
    cout<<"Building 'connected' power law graph with:\n"<<N<<" vertices\n";
    cout<<"Alpha: "<<alpha<<"\nBeta: "<<beta<<"\n\n";
    build_cplod_graph(g,N,alpha,beta,seed);
    if(use_ef_range) {
      cout<<"Using |E| Range: "<<min_edges<<" - "<<max_edges<<"\n\n";
      size_t E = g.get_num_edges();
      while(E < min_edges || E > max_edges) {
        g.clear();
        seed = time(NULL);
        cout<<"."<<std::flush;
        build_cplod_graph(g,N,alpha,beta,seed);
        E = g.get_num_edges();
      }
    }
    cout<<"built.\nSeed: "<<seed<<endl;

    //print the highest out-degree
    typename Graph::vertex_iterator it, it_end;
    it = g.begin(); it_end = g.end();
    it = std::max_element(it,it_end,_max_comp<typename Graph::vertex_reference>());
    cout<<"Max out-degree: "<<(*it).size()<<endl;
  }
  else if(config.m_type == small_world) {
    cout<<"Building small-world graph with:\n"<<N<<" vertices\n";
    cout<<"K: "<<k_neighbors<<"\nProbability: "<<probability<<endl;
    cout<<"Seed: "<<seed<<endl<<endl;
    build_small_world_graph(g,N,k_neighbors,probability,seed);
  }
  else if(config.m_type == star) {
    cout<<"Building a star graph with:\n"<<N<<" vertices\n";
    build_star_graph(g,N);
  }
  else {
    std::cerr<<"in build_graph(): WARNING - invalid type specified.\n";
  }

  std::cout<<"Graph built.\n|V| = "<<g.get_num_vertices();
  std::cout<<"\n|E| = "<<g.get_num_edges()<<"\n\n";

  return;
}

/**
 * takes an empty graph g, and reads from file.
 */
template <typename G>
std::vector<typename G::vertex_descriptor> read_from_file(G& g, std::string filename) {
  std::ifstream ifile;
  ifile.open(filename.c_str());
  size_t nverts = 0;
  ifile >> nverts;
  std::vector<typename G::vertex_descriptor> verts(nverts);
  for (size_t i=0; i<nverts; ++i)
    verts[i] = g.add_vertex();

  typedef int SVD;
  SVD s=0, t=0;

  while(!ifile.eof()) {
    ifile >> s >> t;
    while (t != -1) {
      g.add_edge(s, t);
      ifile >> t;
    }
  }
  ifile.close();

  return verts;
}

//writes a graph's vertex and edge information to an external file.
template <typename G>
void write_to_file(G& g, std::string filename) {
  std::ofstream ofile;
  ofile.open(filename.c_str());
  ofile << g.get_num_vertices() << std::endl;
  typedef typename G::vertex_iterator VI;
  typedef typename G::adj_edge_iterator AEI;
  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    ofile << (*vi).descriptor() << " ";
    for (AEI ei = (*vi).begin(); ei != (*vi).end(); ++ei) {
      ofile << (*ei).target() << " ";
    }
    ofile << "-1" << std::endl;
  }
  ofile.close();
}

template<typename G>
void print_graph(G& g) {
  typename G::vertex_iterator vi, vi_end;
  vi = g.begin(); vi_end = g.end();

/*
  cout<<"STAPL Vertices (vd,prop):"<<endl;
  for(; vi != vi_end; ++vi)
    cout<<"("<<vi->descriptor()<<", "<<vi->property()<<")\n";
*/

  typename G::edge_iterator ei, ei_end;
  ei = g.edges_begin(); ei_end = g.edges_end();
  std::cout<<"\nSTAPL Edges (s,t,p):"<<std::endl;
  for(; ei != ei_end; ++ei) {
    if(boost::is_same<typename G::vertex_property, size_t>::value)
      std::cout<<"("<<(*ei).source()<<", "<<(*ei).target()<<", "<<(*ei).property()<<")\n";
    else if(boost::is_same<typename G::vertex_property, int>::value)
      std::cout<<"("<<(*ei).source()<<", "<<(*ei).target()<<", "<<(*ei).property()<<")\n";
    else std::cout<<"("<<(*ei).source()<<", "<<(*ei).target()<<")\n";
  }

  return;
}

//generates random edge weights in [0,100). Expects the stapl graph to have
//size_t edge properties.
template<typename G>
void propertize(G& g, size_t seed=42) {
  srand(seed);
  typename G::edge_iterator iter;
  for(iter = g.edges_begin(); iter != g.edges_end(); ++iter)
    (*iter).property() = rand()%100;
  return;
}

//helper class for testing BFS and DFS
template< class GRAPH>
class visitor_test {
  typedef typename GRAPH::vertex_iterator   vertex_iterator;
  typedef typename GRAPH::adj_edge_iterator adj_edge_iterator;
  typedef typename GRAPH::vertex_descriptor VD;
  typedef typename GRAPH::edge_descriptor   ED;
  public:

  visitor_test() = default;
  visitor_test(size_t*) {}

  void discover_vertex(vertex_iterator _vi){}

  void examine_vertex(vertex_iterator _vi){}

  void examine_edge(vertex_iterator _vi, adj_edge_iterator _ei){}

  void tree_edge(vertex_iterator _vi, adj_edge_iterator _ei){}

  void non_tree_edge(vertex_iterator _vi, adj_edge_iterator _ei){}

  void gray_target(vertex_iterator _vi, adj_edge_iterator _ei) {}

  void black_target(vertex_iterator _vi, adj_edge_iterator _ei){}

  void finish_vertex(vertex_iterator _vi, int =-1){}
};

//vertex property used to store data used by Dijkstra's algorithm
template <class VD=size_t, class EP=size_t, class CT=int>
class dijkstra_property {
public:
  typedef VD  pred_type;
  typedef EP  distance_type;
  typedef CT  color_type;

  dijkstra_property() {
    m_parent = std::numeric_limits<pred_type>::max();
    m_distance = distance_type();
    m_color = stapl::graph_color<color_type>::white();
  }

  pred_type     m_parent;
  distance_type m_distance;
  color_type    m_color;
};

//mapping functions for property maps that need to be passed to Dijkstra's
template <class Property>
class dijkstra_pred_func {
public:
  typedef typename Property::pred_type value_type;

  value_type get(Property& p) {return p.m_parent;}
  void put(Property& p, value_type _v) {p.m_parent = _v;}

  template <class Functor>
  void apply(Property& p, Functor _f) {_f(p);}
};

template <class Property>
class dijkstra_dist_func {
public:
  typedef typename Property::distance_type value_type;

  value_type get(Property& p) {return p.m_distance;}
  void       put(Property& p, value_type _v) {p.m_distance = _v;}

  template <class Functor>
  void apply(Property& p, Functor _f) {_f(p);}
};

template <class Property>
class dijkstra_color_func {
public:
  typedef typename Property::color_type value_type;

  value_type get(Property& p) {return p.m_color;}
  void       put(Property& p, value_type _v) {p.m_color = _v;}

  template <class Functor>
  void apply(Property& p, Functor _f) {_f(p);}
};

}//end namespace stapl

#endif
