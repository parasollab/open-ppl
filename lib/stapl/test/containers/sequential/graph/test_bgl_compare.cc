/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <sys/time.h>

#include <stapl/containers/sequential/graph/bgl_directed_graph_adapter.hpp>
#include <stapl/containers/sequential/graph/algorithms/breadth_first_search.h>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>


size_t EF;//edge factor; this will be multiplied with N and that is the ne added
size_t bfs1, bfs2;

using namespace boost;

struct get_color
{
  typedef stapl::graph_color<size_t>::value_type value_type;
  template<typename Property>
  value_type get(Property& p)
  {
    return p;
  }

  template<typename Object,typename Property>
  void put(Object& o, Property p)
  {
    o = p;
  }
};

struct get_graph_color
{
  typedef stapl::graph_color<size_t>::value_type value_type;
  template<typename Property>
  value_type get(Property& p)
  {
    return p.get_color();
  }

  template<typename Object,typename Property>
  void put(Object& o, Property p)
  {
    o.set_color(p);
  }
};

class my_vertex_property{
  size_t m_flux;
  typedef stapl::graph_color<size_t>::value_type color_value_type;
  color_value_type m_color;
public:
  my_vertex_property(size_t i=999)
    : m_flux(i),m_color(stapl::graph_color<size_t>::white())
  {}

  void set_color(color_value_type c)
  { m_color = c; }

  color_value_type get_color() const
  { return m_color; }

  size_t flux()
  { return m_flux; }
};

struct my_vertex_property_bgl_composed {
  size_t m_flux;
  boost::default_color_type color;
  my_vertex_property_bgl_composed(size_t i=999)
    : m_flux(i),color(color_traits<boost::default_color_type>::white())
  {}

  size_t flux()
  { return m_flux; }
};

struct my_vertex_property_bgl {
  size_t m_flux;
  my_vertex_property_bgl(size_t i=999)
    : m_flux(i)
  {}

  size_t flux()
  { return m_flux; }
};


/////////////////////////////////////////////////// add_vertex no property
template <class Graph>
void  add_vertices_stapl(Graph& g){
  for (size_t i=0;i<stapl::N;++i)
    g.add_vertex();
}

template <class Graph>
void  add_vertices_bgl(Graph& g){
  for (size_t i=0;i<stapl::N;++i)
    add_vertex(g);
}

//////////////////////////////////////////////// add_edge no property
template <class Graph>
void  add_edges_stapl(Graph& g){
  for (size_t i=0;i<EF*stapl::N;++i)
    g.add_edge(rand() % stapl::N, rand() % stapl::N);
}

template <class Graph>
void  add_edges_bgl(Graph& g){
  for (size_t i=0;i<EF*stapl::N;++i)
    add_edge(rand() % stapl::N, rand() % stapl::N, g);
}

//////////////////////////////////////////////// delete_edge
template <class Graph>
void  delete_edges_stapl(Graph& g){
  for (size_t i=0;i<EF*stapl::N;++i)
    g.delete_edge(rand() % stapl::N, rand() % stapl::N);
}

template <class Graph>
void  delete_edges_bgl(Graph& g){
  for (size_t i=0;i<EF*stapl::N;++i)
    remove_edge(rand() % stapl::N, rand() % stapl::N, g);
}

//////////////////////////////////////////////// delete_vertex
//delete vertices from begining; more expensive because vector has to
//compact
template <class Graph>
void  delete_vertex_one_stapl(Graph& g){
  for (size_t i=0;i<100;++i)
    g.delete_vertex(i);
}

template <class Graph>
void  delete_vertex_one_bgl(Graph& g){
  for (size_t i=0;i<100;++i) {
    clear_vertex(i,g);
    remove_vertex(i, g);
  }
}

//delete from the end; faster than previous scenario
template <class Graph>
void  delete_vertex_two_stapl(Graph& g){
  for (size_t i=0;i<100;++i)
    g.delete_vertex(stapl::N-i-1);
}

template <class Graph>
void  delete_vertex_two_bgl(Graph& g){
  for (size_t i=0;i<100;++i) {
    clear_vertex(stapl::N-i-1,g);
    remove_vertex(stapl::N-i-1, g);
  }
}

///////////////////////////////////////////////// traverse non property
template <class Graph>
size_t  traverse_stapl(Graph& g){
  size_t s=0;
  typename Graph::vertex_iterator vi=g.begin();
  typename Graph::vertex_iterator vi_end=g.end();
  for (;vi!=vi_end;++vi){
    s+=(*vi).descriptor();
    typename Graph::adj_edge_iterator ei = (*vi).begin();
    typename Graph::adj_edge_iterator ei_end = (*vi).end();
    for (;ei != ei_end; ++ei){
      s += (*ei).source() + (*ei).target();
    }
  }
  return s;
}

template <class Graph>
size_t  traverse_bgl(Graph& g){
  size_t s=0;
  typedef typename graph_traits<Graph>::vertex_iterator   vertex_iter;
  typedef typename graph_traits<Graph>::out_edge_iterator edge_iter;
  std::pair<vertex_iter, vertex_iter> vp;
  for (vp = vertices(g); vp.first != vp.second; ++vp.first){
    s += (*(vp.first)).descriptor();
    std::pair<edge_iter, edge_iter> ep;
    for (ep = out_edges((*(vp.first)).descriptor(),g); ep.first != ep.second;
       ++ep.first){
      s += (*ep.first).source() + (*ep.first).target();
    }
  }
  return s;
}


/////////////////////////////////////////////////// add_vertex property
template <class Graph>
void  add_vertices_p_stapl(Graph& g){
  for (size_t i=0;i<stapl::N;++i)
      g.add_vertex(
      typename Graph::vertex_property(i));
}

template <class Graph>
void  add_vertices_p_bgl(Graph& g){
  typedef typename graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_map<Graph,
      boost::vertex_color_t>::type  vcmap_type;
  typedef typename boost::property_map<Graph,
      boost::vertex_name_t>::type  vpmap_type;

  for (size_t i=0;i<stapl::N;++i){
    vertex_descriptor vd = add_vertex(g);
    boost::get(boost::vertex_name, g)[vd]=my_vertex_property_bgl(i);
    boost::get(boost::vertex_color, g)[vd]=
        color_traits<boost::default_color_type>::white();
  }
}

//////////////////////////////////////////////// add_edge no property
template <class Graph>
void  add_edges_p_stapl(Graph& g){
    typedef typename Graph::vertex_descriptor VD;
  for (size_t i=0;i<EF*stapl::N;++i){
    VD vd1 = rand() % stapl::N;
    VD vd2 = rand() % stapl::N;
    g.add_edge(vd1, vd2); //, typename Graph::edge_property(1,1));
  }
}

template <class Graph>
void  add_edges_p_bgl(Graph& g){
  for (size_t i=0;i<EF*stapl::N;++i)
    add_edge(rand() % stapl::N, rand() % stapl::N, stapl::weight(1,1), g);
}

//traverse plus properties
template <class Graph>
size_t  traverse_p_stapl(Graph& g){
  size_t s=0;
  typename Graph::vertex_iterator vi=g.begin();
  typename Graph::vertex_iterator vi_end=g.end();
  for (;vi!=vi_end;++vi){
    s+= (*vi).descriptor();
    s+= (*vi).property().flux();
    typename Graph::adj_edge_iterator ei = (*vi).begin();
    typename Graph::adj_edge_iterator ei_end = (*vi).end();
    for (;ei != ei_end; ++ei){
      s += (*ei).source() + (*ei).target();
      s += (*ei).property().source;
    }
  }
  return s;
}

template <class Graph>
size_t  traverse_p_bgl(Graph& g){
  size_t s=0;
  typedef typename boost::property_map<Graph,
      boost::vertex_name_t>::type  vpmap_type;
  typedef typename boost::property_map<Graph,
      boost::edge_weight_t>::type  epmap_type;

  typedef typename graph_traits<Graph>::vertex_iterator   vertex_iter;
  typedef typename graph_traits<Graph>::out_edge_iterator edge_iter;
  std::pair<vertex_iter, vertex_iter> vp;
  vpmap_type vpmap = boost::get(boost::vertex_name, g);
  epmap_type epmap = boost::get(boost::edge_weight, g);
  for (vp = vertices(g); vp.first != vp.second; ++vp.first){
    s += *(vp.first);
    s += vpmap[*(vp.first)].flux();
    std::pair<edge_iter, edge_iter> ep;
    for (ep = out_edges(*(vp.first),g); ep.first != ep.second; ++ep.first){
      s += source(*(ep.first),g) + target(*(ep.first),g);
      s += epmap[*(ep.first)].source;
    }
  }
  return s;
}

////////////////////////////////////////////////////// invoke BFS
template< class GRAPH>
class visitor_test{
  typedef typename GRAPH::vertex_iterator   vertex_iterator;
  typedef typename GRAPH::adj_edge_iterator adj_edge_iterator;
  typedef typename GRAPH::vertex_descriptor VD;
  typedef typename GRAPH::edge_descriptor ED;
  public:

  size_t m_sum;

  visitor_test(){m_sum=0;}
  visitor_test(GRAPH& g){m_sum=0;}

  void discover_vertex(vertex_iterator vi){
    m_sum += (*vi).descriptor();
    //m_sum += 1;
  }

  void examine_vertex(vertex_iterator vi){
    m_sum -= (*vi).descriptor();
    //m_sum -= 1;
  }

  void examine_edge(vertex_iterator vi, adj_edge_iterator ei){
    //m_sum += 1;
    this->m_sum += (*ei).source();
    this->m_sum -= (*ei).target();
  }
  void tree_edge(vertex_iterator vi, adj_edge_iterator ei){
    //m_sum += 1;
    this->m_sum += (*ei).source();
    this->m_sum -= (*ei).target();
  }

  void non_tree_edge(vertex_iterator vi, adj_edge_iterator ei){
    //m_sum += 1;
    this->m_sum += (*ei).source();
    this->m_sum -= (*ei).target();
  }

  void gray_target(vertex_iterator vi, adj_edge_iterator ei) {
    //m_sum += 1;
    this->m_sum += (*ei).source();
    this->m_sum -= (*ei).target();
  }

  void black_target(vertex_iterator vi, adj_edge_iterator ei){
    //m_sum += 1;
    this->m_sum += (*ei).source();
    this->m_sum -= (*ei).target();
  }
  void finish_vertex(vertex_iterator vi, int =-1){
    //m_sum += 1;
    m_sum += (*vi).descriptor();
  }
  ~visitor_test() = default;
};

template <class Graph>
void  bfs_stapl(Graph& g){
  stapl::sequential::vector_property_map<Graph, size_t> cmap;
  visitor_test<Graph> vis;
  stapl::sequential::breadth_first_search(g,0,vis,cmap);
  bfs1 = vis.m_sum;
}


template <class Graph>
void  bfs_all_stapl(Graph& g){
  stapl::sequential::vector_property_map<Graph, size_t, get_color> cmap;
  visitor_test<Graph> vis;
  typename Graph::vertex_iterator vi=g.begin();
  typename Graph::vertex_iterator vi_end=g.end();
  for (;vi!=vi_end;++vi){
    if (cmap.get(*vi) == stapl::graph_color<size_t>::white()){
      stapl::sequential::breadth_first_search(g,(*vi).descriptor(),vis,cmap);
    }
  }
  bfs1 = vis.m_sum;
}


template <class Graph>
void  bfs_internal_stapl(Graph& g){
  stapl::graph_internal_property_map<Graph, get_graph_color > cmap(g);
  visitor_test<Graph> vis;
  stapl::sequential::breadth_first_search(g,0,vis,cmap);
  bfs1 = vis.m_sum;
}


template <class Graph>
void  bfs_all_internal_stapl(Graph& g){
  stapl::graph_internal_property_map<Graph, get_graph_color > cmap(g);
  cmap.reset();

  visitor_test<Graph> vis;
  typename Graph::vertex_iterator vi=g.begin();
  typename Graph::vertex_iterator vi_end=g.end();
  for (;vi!=vi_end;++vi){
    if (cmap.get(*vi) == stapl::graph_color<size_t>::white()){
      stapl::sequential::breadth_first_search(g,(*vi).descriptor(),vis,cmap);
    }
  }
  bfs1 = vis.m_sum;
}

template< class GRAPH>
class visitor_bgl{
  public:

  size_t& m_sum;

  visitor_bgl(size_t& b) : m_sum(b){
    m_sum = 0;
  }

  template < typename Vertex, typename Graph >
  void discover_vertex(Vertex u, Graph & g) {
    m_sum += u;
  }

  template < typename Vertex, typename Graph >
  void examine_vertex(Vertex u, Graph & g) {
    m_sum -= u;
  }

  template < typename Vertex, typename Graph >
  void initialize_vertex(Vertex s, Graph& g) {
  }


  template < typename Edge, typename Graph >
  void examine_edge(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void tree_edge(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void non_tree_edge(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void black_target(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void gray_target(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Vertex, typename Graph >
  void finish_vertex(Vertex s, Graph& g) {
    m_sum += s;
  }

};

template <class Graph>
void  bfs_bgl_boost(Graph& g){
  typename boost::graph_traits<Graph>::vertex_descriptor s = vertex(0, g);
  std::vector<default_color_type> colors(num_vertices(g), color_traits<
      boost::default_color_type>::white());
  visitor_bgl<Graph> vis(bfs2);
  boost::breadth_first_search(g, s,
      boost::color_map(make_iterator_property_map(colors.begin(),
          get(vertex_index,g))).visitor (vis));
  bfs2 = vis.m_sum;
}

template <class Graph>
void  bfs_all_bgl_boost(Graph& g){
  typedef typename graph_traits<Graph>::vertex_iterator   vertex_iter;
  std::vector<default_color_type> colors(num_vertices(g), color_traits<
      boost::default_color_type>::white());
  visitor_bgl<Graph> vis(bfs2);
  std::pair<vertex_iter, vertex_iter> vp;
  for (vp = vertices(g); vp.first != vp.second; ++vp.first){
    if (colors[*vp.first] == color_traits<boost::default_color_type>::white()){
      boost::breadth_first_search(g, vertex(*vp.first,g), visitor (vis));
      boost::breadth_first_visit(g,
          vertex(*(vp.first),g),
              boost::color_map(make_iterator_property_map(colors.begin(),
                  get(vertex_index,g))).visitor (vis));
    }
  }
  bfs2 = vis.m_sum;
}

template <class Graph>
void  bfs_internal_bgl(Graph& g){
  visitor_bgl<Graph> vis(bfs2);
  breadth_first_search(g, vertex(0,g), boost::color_map(boost::get(
      boost::vertex_color, g)).visitor(vis));
  bfs2 = vis.m_sum;
}


template <class Graph>
void  bfs_all_internal_bgl(Graph& g){
  typedef typename graph_traits<Graph>::vertex_iterator   vertex_iter;
  typedef typename boost::property_map<Graph,
      boost::vertex_color_t>::type  vcmap_type;
  visitor_bgl<Graph> vis(bfs2);
  //reset color
  vcmap_type cmap = boost::get(boost::vertex_color, g);
  std::pair<vertex_iter, vertex_iter> vp;
  for (vp = vertices(g); vp.first != vp.second; ++vp.first){
    cmap[*(vp.first)]=color_traits<boost::default_color_type>::white();
  }
  //invoke BFS forest
  for (vp = vertices(g); vp.first != vp.second; ++vp.first){
    if (cmap[*(vp.first)] == color_traits<boost::default_color_type>::white()){
      breadth_first_visit(g, vertex(*(vp.first),g), boost::color_map(
          boost::get(boost::vertex_color, g)).visitor(vis));
    }
  }
  bfs2 = vis.m_sum;
}

////////////////////////////////////////////////////print for debug
template <class Graph>
void  print_stapl(Graph& g){
  typename Graph::vertex_iterator vi;
  for (vi=g.begin();vi!=g.end();++vi){
    cout<<vi.descriptor()<<" ";
    for (typename Graph::adj_edge_iterator ei = vi.begin();ei != vi.end();++ei){
      cout<<ei.source()<<"-"<<ei.target()<<" ";
    }
    cout<<"\n";
  }
 }

template <class Graph>
void print_bgl(Graph& g){
  typedef typename graph_traits<Graph>::vertex_iterator   vertex_iter;
  typedef typename graph_traits<Graph>::out_edge_iterator edge_iter;
  std::pair<vertex_iter, vertex_iter> vp;
  for (vp = vertices(g); vp.first != vp.second; ++vp.first){
    cout<< *(vp.first)<<" ";
    std::pair<edge_iter, edge_iter> ep;
    for (ep = out_edges(*(vp.first),g); ep.first != ep.second; ++ep.first){
      cout << source(*(ep.first),g)<<"-"<<target(*(ep.first),g)<<" ";
    }
    cout<<"\n";
  }
}


class my_timer{
  timeval t1, t2;
  double elapsedTime;
public:
  void start(){
    gettimeofday(&t1, NULL);
  }

  void stop(char* s){
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    cout << s <<"="<<elapsedTime << " ms.\n";
  }

};

//======================================================= MAIN

int main(int argc, char** argv){
  size_t gvers=0;
  cout<<"Benchmark: Stapl Graph versus BGL\n";
  if (argc < 4) {
    cout<<"by default N is 10 and EF is 2 and run stapl only\n";
    stapl::N=10;
    EF=2;
  }
  else {
    stapl::N = atoi(argv[1]);
    EF=atoi(argv[2]);
    gvers=atoi(argv[3]);
  }


  if (stapl::N==1){cout<<"We need at least two vertices\n"; stapl::N=2;}

  for (size_t it=0;it<1;++it){
    srand(stapl::N+it);
    my_timer t;
    size_t s1,s2;
    if (gvers==0)
    {
      std::cout<<it<<">>> Testing graph<DIRECTED, MULTIEDGES>\n";
      stapl::sequential::graph<stapl::DIRECTED,stapl::MULTIEDGES> g1;
      t.start();
      add_vertices_stapl(g1);
      t.stop((char*)"add_vertex");

      t.start();
      add_edges_stapl(g1);
      t.stop((char*)"add_edge");

      t.start();
      for (size_t i=0;i<10;++i)
        s2=traverse_stapl(g1);
      t.stop((char*)"traverse_graph");

      t.start();
      bfs_stapl(g1);
      t.stop((char*)"bfs");

      t.start();
      bfs_all_stapl(g1);
      t.stop((char*)"bfs_all");

      t.start();
       delete_edges_stapl(g1);
      t.stop((char*)"delete_edge");

      t.start();
      add_edges_stapl(g1);
      t.stop((char*)"add_edge");

      t.start();
      delete_vertex_two_stapl(g1);
      t.stop((char*)"delete_vertex");

    }//the graph is cleaned up

    if (gvers==1)
    {
      std::cout<<"\n>>> Testing boost_graph<>\n";
      srand(stapl::N+it);
      // typedef boost::adjacency_list<boost::vecS,
      //   boost::vecS,
      //   boost::directedS> Graph;
      typedef stapl::sequential::graph<stapl::DIRECTED,stapl::MULTIEDGES,int,
          int> Graph;
      Graph bg;

      num_vertices(bg);

      t.start();
      add_vertices_bgl(bg);
      t.stop((char*)"add_vertex");

      t.start();
      add_edges_bgl(bg);
      t.stop((char*)"add_edge");

      t.start();
      for (size_t i=0;i<10;++i)
        s1 = traverse_bgl(bg);
      t.stop((char*)"traverse_graph");

      t.start();
      /**/
      std::vector<default_color_type> colors(num_vertices(bg),color_traits<
          boost::default_color_type>::white());
      visitor_bgl<Graph> vis(bfs2);
      boost::queue<Graph::vertex_descriptor> q;

      // Function is having trouble recognizing queue and vertex iterator
      //boost::breadth_first_search(bg,
      //                            *vertices(bg).first,
      //                            q,
      //                            vis,
      //                            make_iterator_property_map(colors.begin(),
      //                                                  get(vertex_index,bg))
      //                            );

      std::vector < graph_traits < Graph >::vertex_descriptor >
        p(num_vertices(bg));
      std::vector<size_t> distances(num_vertices(bg),
      std::numeric_limits<int>::max());
      std::vector<size_t> indexes(num_vertices(bg), 0);
      std::vector<int> weights(num_vertices(bg));
      
      /*
       boost::distance_map(make_iterator_property_map(distances.begin(),
        get(vertex_index, bg)));
       boost::weight_map(make_iterator_property_map(indexes.begin(),
        get(vertex_index, bg)));
      
        property_map<Graph, vertex_distance_t>::type distance = get(
            vertex_distance, bg);
        property_map<Graph, Graph::vertex_property>::type indexmap = get(
            vertex_index, bg);
        property_map<Graph, Graph::edge_property>::type weightmap = get(
            edge_weight, bg);
      */
      //prim_minimum_spanning_tree(bg, *(*vertices(bg).first).descriptor(),
      //                          &p[0],
      //                          make_iterator_property_map(distances.begin(),
      //                          get(vertex_index, bg)),
      //                          make_iterator_property_map(indexes.begin(),
      //                          get(edge_weight, bg)), //weight,
      //                          make_iterator_property_map(weights.begin(),
      //                          get(vertex_index, bg)), //index,
      //                          default_dijkstra_visitor());
      /**/
      stapl::display(bg);
      cout << endl;
      for (size_t i=0; i < p.size(); ++i)
        cout << p[i] << "\t";
      cout << endl;

      // prim_minimum_spanning_tree(bg, 0);
      t.stop((char*)"mst_time");

      t.start();
      // delete_edges_bgl(bg);
      t.stop((char*)"delete_edge");

      t.start();
      add_edges_bgl(bg);
      t.stop((char*)"add_edge");

      t.start();
      // delete_vertex_two_bgl(bg);
      t.stop((char*)"delete_vertex");

    }

    /////////////////////////////////////////////////////////////////// PROPERTY
    //if (s1 != s2 ) cout<<"no properties different!!!\n";

    if (gvers==0)
    {
      srand(stapl::N+it);
      std::cout<<it<<">>> Testing graph<DIRECTED, MULTIEDGES, mvp, weight>\n";
      stapl::sequential::graph<stapl::DIRECTED,stapl::MULTIEDGES,
          my_vertex_property, stapl::weight> g1;

      t.start();
      add_vertices_p_stapl(g1);
      t.stop((char*)"add_vertex(property)");

      t.start();
      add_edges_p_stapl(g1);
      t.stop((char*)"add_edge(property)");

      t.start();
      for (size_t i=0;i<10;++i)
        s2=traverse_p_stapl(g1);
      t.stop((char*)"traverse_graph");

      t.start();
      bfs_stapl(g1);
      t.stop((char*)"bfs");

      t.start();
      bfs_all_stapl(g1);
      t.stop((char*)"bfs_all");

      t.start();
      bfs_internal_stapl(g1);
      t.stop((char*)"bfs_internal");

      t.start();
      bfs_all_internal_stapl(g1);
      t.stop((char*)"bfs_all_internal");

      t.start();
       delete_edges_stapl(g1);
      t.stop((char*)"delete_edge");

      t.start();
      add_edges_p_stapl(g1);
      t.stop((char*)"add_edge");

      t.start();
      delete_vertex_two_stapl(g1);
      t.stop((char*)"delete_vertex");
    }//the graph is cleaned up here

    if (gvers==1)
    {
      std::cout<<"\n>>> Testing boost_graph<vertex_property, edge_property>\n";
      srand(stapl::N+it);
      typedef boost::adjacency_list<boost::vecS,
        boost::vecS,
        boost::directedS,
        boost::property<boost::vertex_color_t,default_color_type,
        boost::property<boost::vertex_name_t,my_vertex_property_bgl> >,
        boost::property<boost::edge_weight_t,stapl::weight> > Graph;

      Graph bg;

      t.start();
      add_vertices_p_bgl(bg);
      t.stop((char*)"add_vertex(property)");

      t.start();
      add_edges_p_bgl(bg);
      t.stop((char*)"add_edge(property)");

      t.start();
      for (size_t i=0;i<10;++i)
        s1 = traverse_p_bgl(bg);
      t.stop((char*)"traverse_graph");

      t.start();
      bfs_bgl_boost(bg);
      t.stop((char*)"bfs");

      t.start();
      bfs_all_bgl_boost(bg);
      t.stop((char*)"bfs_all");

      t.start();
      bfs_internal_bgl(bg);
      t.stop((char*)"bfs_internal");

      t.start();
      bfs_all_internal_bgl(bg);
      t.stop((char*)"bfs_all_internal");

      t.start();
      // delete_edges_bgl(bg);
      t.stop((char*)"delete_edge");

      t.start();
      add_edges_p_bgl(bg);
      t.stop((char*)"add_edge");

      t.start();
      // delete_vertex_two_bgl(bg);
      t.stop((char*)"delete_vertex");
    }

    std::cout<<"bfs1="<<bfs1<<" bfs2="<<bfs2<<"\n";
    //if (stapl::N<100) {
    //  print_bgl(bg);
    //  cout<<"========================\n";
    //  print_stapl(g1);
    //}

    if (s1 != s2 ) cout<<"Different!!!"<<s1<<":"<<s2<<"\n";
  }//end for
  return 0;
}
