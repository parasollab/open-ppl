/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BGL_PROFILERS_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BGL_PROFILERS_HPP

#include <limits>
#include <vector>
#include "profile_utils.h"
#include "container_profiler.h"
#include <stapl/runtime/counter/default_counters.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

//////////////////////////////////////////////////////////////////////
/// @file
/// @brief Contains profilers for BGL's graph methods and algorithms.
//////////////////////////////////////////////////////////////////////

namespace stapl {

//====================================================================
// Basic profilers.
//====================================================================
/**
 * Add vertices to bgl::graph
 */
template <class C, class Counter=counter<default_timer> >
class add_vertex_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter> base_type;

 public:
 add_vertex_profiler_bgl(std::string pcname, C* pc, profile_config& config) :
  base_type(pc,pcname+"_add_vertex_bgl", config.m_argc, config.m_argv) {
    this->n_times = config.m_verts.size();
  }

  void run(){
    for(size_t i=0;i<this->n_times;++i) add_vertex(*(this->m_c));
  }

  void finalize_iteration(){
    this->m_c->clear();
  }

};

/**
 * Delete random vertices from the graph; takes a seed and generates
 * a vector of the VDs to be deleted.
 */
template <class C, class Counter=counter<default_timer> >
class delete_vertex_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef boost::graph_traits<C>                  traits;
  typedef typename traits::vertex_descriptor      vd_type;
  typedef typename traits::vertex_iterator        vi_type;
  typedef std::vector<vd_type>                    del_list_t;
  typedef typename del_list_t::iterator           del_iter;

  del_list_t m_to_del;
  del_iter   m_it, m_it_end;

  profile_config& m_config;

public:
  delete_vertex_profiler_bgl(std::string pcname, C* pc, profile_config& config) : 
   base_type(pc,pcname+"_delete_vertex_bgl", config.m_argc, config.m_argv), m_config(config) {
    this->n_times = config.m_vdc;
  }

  void initialize_iteration() {
    //NOTE: Tacit assumption here that the graph blueprint is stored in m_config;
    //if we ever decide to use external memory to store the graph, the
    //conditions on this loop need to change. And probably other loops too.
    for(size_t i=0;i<m_config.m_verts.size(); ++i) 
      add_vertex(*this->m_c);

    vi_type it, it_end;
    boost::tie(it, it_end) = vertices(*this->m_c);
    del_list_t verts(it, it_end);

    m_to_del.reserve(this->n_times);
    for(size_t i=0; i<m_config.m_vert_del_list.size(); ++i)
      m_to_del.push_back(verts[m_config.m_vert_del_list[i]]);

    //NOTE: At this point, we're casing between vector and list vertex storage.
    if(boost::is_same<size_t, vd_type>::value) { //then we're using vector storage.
      //now rename appropriately; NOTE: O(n^2) for this.
      rename_vertices(m_to_del);
    } 
    //else it's a list of void*, and we need do nothing else.

    m_it = m_to_del.begin();
    m_it_end = m_to_del.end();
  }

  //we time both clear and remove, as this is closer to being equivalent
  //to SSGL's delete_vertex() method.
  void run() {
    C &g = *this->m_c;
    for(; m_it != m_it_end; ++m_it) {
      clear_vertex(*m_it, g);
      remove_vertex(*m_it, g);
    }
  }

  void finalize_iteration() {
    m_to_del.clear();
    this->m_c->clear();
  }
};

/**
 * Add edges to bgl::graph
 */
template <class C, class Counter=counter<default_timer> >
class add_edge_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>            base_type;
  typedef boost::graph_traits<C>                    traits;
  typedef typename traits::vertex_descriptor        vd_type;
  typedef typename traits::vertex_iterator          vi_type;
  typedef std::vector<std::pair<vd_type, vd_type> > edges_type;

  profile_config& m_config;
  edges_type      m_edges;

  typename edges_type::iterator it, it_end;

public:
  add_edge_profiler_bgl(std::string pcname, C* pc, profile_config& config) :
   base_type(pc,pcname+"_add_edge_bgl", config.m_argc, config.m_argv), m_config(config) { 
    this->n_times = config.m_edges.size();
  }

  void initialize_iteration() {
    for(size_t i=0; i<m_config.m_verts.size(); ++i)
      add_vertex(*this->m_c);

    vi_type vit, vit_end;
    boost::tie(vit,vit_end) = vertices(*this->m_c);
    std::vector<vd_type> verts(vit, vit_end);

    m_edges.reserve(m_config.m_edges.size());
    for(size_t i=0; i<m_config.m_edges.size(); ++i) {
      profile_config::edges_type::value_type edge = m_config.m_edges[i];
      m_edges.push_back(std::make_pair(verts[edge.first],
                                       verts[edge.second]));
    }

    it = m_edges.begin();
    it_end = m_edges.end();
  }

  void run() {
    for(; it != it_end; ++it)
      add_edge(it->first, it->second, *this->m_c);
  }

  void finalize_iteration() {
    m_edges.clear();
    this->m_c->clear();
  }
};

/**
 * Delete random edges of a bgl::graph
 */
template <class C, class Counter=counter<default_timer> >
class delete_edge_profiler_bgl
  : public container_profiler<C,Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef boost::graph_traits<C>                  traits;
  typedef typename traits::vertex_descriptor      vd_type;
  typedef typename traits::vertex_iterator        vi_type;
  typedef std::vector<vd_type>                    verts_type;
  typedef std::pair<vd_type, vd_type>             ed_type;
  typedef std::vector<ed_type>                    edges_type;

  profile_config& m_config;
  edges_type      m_to_del;
  typename edges_type::iterator it, it_end;

 public:
 delete_edge_profiler_bgl(std::string pcname, C* pc, profile_config& config) :
   base_type(pc,pcname+"_delete_edge_bgl", config.m_argc, config.m_argv), m_config(config) {
     this->n_times = config.m_edge_del_list.size();
 }
  
  void initialize_iteration() {
    //build the graph.
    C &g = *this->m_c;
    m_config.rebuild_bgl_graph(g);

    //collect the VDs and use them to fill m_to_del.
    vi_type vit, vit_end;
    boost::tie(vit,vit_end) = vertices(g);
    verts_type verts(vit,vit_end);

    m_to_del.reserve(m_config.m_edge_del_list.size());
    typename profile_config::edges_type::iterator eit, eit_end;
    eit = m_config.m_edge_del_list.begin();
    eit_end = m_config.m_edge_del_list.end();
    for(; eit!=eit_end; ++eit) {
      m_to_del.push_back(std::make_pair(verts[eit->first], 
                                        verts[eit->second]));
    }

    it = m_to_del.begin();
    it_end = m_to_del.end();
  }

  void run() {
    for(; it!=it_end; ++it)
      remove_edge(it->first, it->second, *this->m_c);
  }

  void finalize_iteration() {
    m_to_del.clear();
    this->m_c->clear();
  }
};

//====================================================================
// Traversal profiler
//====================================================================
/**
 * Traverse the out-edge list of each vertex.
 */
template <class C, class Counter=counter<default_timer> >
class traversal_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C,Counter>           base_type;
  typedef typename C::vertex_iterator             vertex_iterator;
  typedef typename C::out_edge_iterator           out_edge_iterator;

  size_t            m_sum;
  vertex_iterator   vi, vi_end;
  out_edge_iterator oei, oei_end;

 public:
 traversal_profiler_bgl(std::string pcname, C* pc, int argc=0, char** argv=NULL) : 
   base_type(pc,pcname+"_traversal_bgl", argc, argv) { this->n_times = 1; }

  void initialize_iteration() { 
    m_sum = 0;
    boost::tie(vi,vi_end) = boost::vertices(*this->m_c);
  }

  void run(){
    for(; vi != vi_end; ++vi) {
      boost::tie(oei,oei_end) = boost::out_edges(*vi, *this->m_c);
      for(; oei != oei_end; ++oei) 
        m_sum++;
    }
  }
};

//====================================================================
// Algorithm profilers
//====================================================================
/**
 * Profiles the boost::breadth_first_visit method. Uses external
 * storage for the color data.
 */
template<class C, class Counter=counter<default_timer> >
class bfs_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef visitor_bgl<C>                          vis_type;
  typedef boost::graph_traits<C>                  traits;

  typedef typename traits::vertex_descriptor      vd_type;
  typedef boost::queue<vd_type>                   queue_type;

  //typedef'ing an external color map, stored in a vector.
  typedef std::vector<boost::default_color_type>  color_cont_t;
  typedef typename color_cont_t::iterator         color_cont_iter_t;
  typedef typename boost::property_map<C,boost::vertex_index_t>::type  idmap_t;
  typedef boost::iterator_property_map<color_cont_iter_t, idmap_t,
    typename std::iterator_traits<color_cont_iter_t>::value_type,
    typename std::iterator_traits<color_cont_iter_t>::reference> cmap_t;

  size_t                bfs;
  boost::vertex_index_t vertex_index;
  color_cont_t          color_cont;
  cmap_t                colors;
  queue_type            queue;
  vis_type              vis;

 public:
 bfs_profiler_bgl(std::string pcname, C* pc, int argc=0, char **argv=NULL) : 
  base_type(pc,pcname+"_bfs_bgl", argc, argv) { this->n_times = 1; bfs = 0; }

  void initialize_iteration() {
    color_cont = color_cont_t(num_vertices(*this->m_c));
    colors = cmap_t(color_cont.begin(), get(boost::vertex_index, *this->m_c));
    queue = queue_type();
    bfs = 0;
    vis = vis_type(&bfs);

    typedef typename boost::property_traits<cmap_t>::value_type color_value;
    typedef boost::color_traits<color_value> color_set;
    typename boost::graph_traits<C>::vertex_iterator i,i_end;
    for(boost::tie(i,i_end) = vertices(*this->m_c); i != i_end; ++i) {
      vis.initialize_vertex(*i, *this->m_c);
      put(colors, *i, color_set::white());
    }
  }

  void run() { 
    boost::breadth_first_visit(*this->m_c, vertex(0,*this->m_c),
                                queue, vis, colors);
  }
};

/**
 * Profiles the boost::breadth_first_visit method. Expects the color
 * data to be stored on the graph's vertices under the vertex_color_t
 * tag.
 */
template<class C, class Counter=counter<default_timer> >
class bfs_internal_profiler_bgl
  : public container_profiler<C,Counter>
{
  typedef container_profiler<C,Counter>                               base_type;
  typedef typename boost::graph_traits<C>::vertex_descriptor          vd_type;
  typedef typename boost::property_map<C,boost::vertex_color_t>::type c_type;
  typedef visitor_bgl<C>                                              vis_type;
  typedef boost::queue<vd_type>                                      queue_type;

  size_t                bfs;
  boost::vertex_index_t vertex_index;
  c_type                colors;
  vis_type              vis;
  queue_type            queue;

 public:
 bfs_internal_profiler_bgl(std::string pcname, C* pc, int argc=0, char **argv=NULL) : 
  base_type(pc,pcname+"_bfs_bgl", argc, argv) { this->n_times = 1; bfs = 0; }

  void initialize_iteration() {
    colors = boost::get(boost::vertex_color, *this->m_c);
    bfs = 0;
    vis = vis_type(&bfs);
    queue = queue_type();

    typedef typename boost::property_traits<c_type>::value_type color_value;
    typedef boost::color_traits<color_value> color_set;
    typename boost::graph_traits<C>::vertex_iterator i,i_end;
    for(boost::tie(i,i_end) = vertices(*this->m_c); i != i_end; ++i) {
      vis.initialize_vertex(*i, *this->m_c);
      put(colors, *i, color_set::white());
    }
  }

  void run() { 
    boost::breadth_first_visit(*this->m_c,vertex(0,*this->m_c),
                                queue,vis,colors);
  }
};

/**
 * Profiles the boost::depth_first_visit() method. Uses external storage
 * for the algorithm data.
 */
template<class C, class Counter=counter<default_timer> >
class dfs_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter> base_type;

  typedef std::vector<boost::default_color_type>  color_cont_t;
  typedef typename color_cont_t::iterator         color_cont_iter_t;
  typedef typename boost::property_map<C,boost::vertex_index_t>::type  idmap_t;

  typedef boost::iterator_property_map<color_cont_iter_t, idmap_t,
    typename std::iterator_traits<color_cont_iter_t>::value_type,
    typename std::iterator_traits<color_cont_iter_t>::reference> cmap_t;

  typedef visitor_bgl<C>                            vis_type;

  size_t dfs;
  color_cont_t color_cont;
  cmap_t colors;
  vis_type vis;

 public:
 dfs_profiler_bgl(std::string pcname, C* pc, int argc=0, char **argv=NULL) : 
  base_type(pc,pcname+"_dfs_bgl", argc, argv) { this->n_times = 1; dfs = 0; }

  void initialize_iteration() {
    color_cont = color_cont_t(num_vertices(*this->m_c));
    colors = cmap_t(color_cont.begin(), get(boost::vertex_index, *this->m_c));
    dfs = 0;
    vis = vis_type(&dfs);

    typedef typename boost::property_traits<cmap_t>::value_type color_value;
    typedef boost::color_traits<color_value> color_set;
    typename boost::graph_traits<C>::vertex_iterator i,i_end;
    for(boost::tie(i,i_end) = vertices(*this->m_c); i != i_end; ++i) {
      vis.initialize_vertex(*i, *this->m_c);
      put(colors, *i, color_set::white());
    }
  }

  void run() { 
    boost::depth_first_visit(*this->m_c, 0, vis, colors);
  }
};

template<class C, class Counter=counter<default_timer> >
class dfs_internal_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>                              base_type;
  typedef typename boost::property_map<C,boost::vertex_color_t>::type c_type;
  typedef visitor_bgl<C>                                              vis_type;

  size_t    dfs;
  c_type    colors;
  vis_type  vis;

 public:
 dfs_internal_profiler_bgl(std::string pcname, C* pc, int argc=0, char **argv=NULL) : 
  base_type(pc,pcname+"_dfs_bgl", argc, argv) { this->n_times = 1; dfs = 0; }

  void initialize_iteration() {
    colors = boost::get(boost::vertex_color, *this->m_c);
    dfs = 0;
    vis = vis_type(&dfs);

    typedef typename boost::property_traits<c_type>::value_type color_value;
    typedef boost::color_traits<color_value> color_set;
    typename boost::graph_traits<C>::vertex_iterator i,i_end;
    for(boost::tie(i,i_end) = vertices(*this->m_c); i != i_end; ++i) {
      vis.initialize_vertex(*i, *this->m_c);
      put(colors, *i, color_set::white());
    }
  }

  void run() { 
    boost::depth_first_visit(*this->m_c, 0, vis, colors);
  }
};

template<class C, class Counter=counter<default_timer> >
class dijkstra_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>              base_type;
  typedef boost::graph_traits<C>                      traits;
  typedef typename traits::vertex_descriptor          vd_type;
  typedef typename traits::vertex_iterator            vi_type;
  typedef boost::default_dijkstra_visitor             vis_type;

  typedef typename boost::property_map
                     <C, boost::edge_weight_t>::type   wmap_type;
  typedef typename boost::property_map
                     <C, boost::vertex_index_t>::type  imap_type;
  typedef typename boost::property_traits<wmap_type>::value_type weight_type;
  
  typedef std::vector<vd_type>                    pmap_cont_t;
  typedef std::vector<weight_type>                dmap_cont_t;
  typedef std::vector<boost::default_color_type>  cmap_cont_t;
  typedef typename pmap_cont_t::iterator          pmap_cont_iter_t;
  typedef typename dmap_cont_t::iterator          dmap_cont_iter_t;
  typedef typename cmap_cont_t::iterator          cmap_cont_iter_t;
  
  typedef boost::iterator_property_map<pmap_cont_iter_t, imap_type,
    typename std::iterator_traits<pmap_cont_iter_t>::value_type,
    typename std::iterator_traits<pmap_cont_iter_t>::reference> pmap_type;

  typedef boost::iterator_property_map<dmap_cont_iter_t, imap_type,
    typename std::iterator_traits<dmap_cont_iter_t>::value_type,
    typename std::iterator_traits<dmap_cont_iter_t>::reference> dmap_type;

  typedef boost::iterator_property_map<cmap_cont_iter_t, imap_type,
    typename std::iterator_traits<cmap_cont_iter_t>::value_type,
    typename std::iterator_traits<cmap_cont_iter_t>::reference> cmap_type;

  vi_type        src;
  wmap_type      weightmap;
  imap_type      indexmap;

  pmap_cont_t    pmap_cont;
  dmap_cont_t    dmap_cont;
  cmap_cont_t    cmap_cont;
  pmap_type      parentmap;
  dmap_type      distmap;
  cmap_type      colormap;

  vis_type       vis;
  weight_type    max;
  std::less<weight_type> comp;
  std::plus<weight_type> combine;

 public:
 dijkstra_profiler_bgl(std::string pcname, C* pc, int argc=0, char **argv=NULL) : 
  base_type(pc,pcname+"_dijkstra_bgl", argc, argv) { 
    this->n_times = 1;
    max = std::numeric_limits<weight_type>::max();
    comp = std::less<weight_type>();
    combine = std::plus<weight_type>();
  }

  void initialize_iteration() {
    C &g = *this->m_c;
    src = vertices(g).first;

    pmap_cont = pmap_cont_t(num_vertices(*this->m_c));
    dmap_cont = dmap_cont_t(num_vertices(*this->m_c));
    cmap_cont = cmap_cont_t(num_vertices(*this->m_c));
    parentmap = pmap_type(pmap_cont.begin(), get(boost::vertex_index, *this->m_c));
    distmap   = dmap_type(dmap_cont.begin(), get(boost::vertex_index, *this->m_c));
    colormap  = cmap_type(cmap_cont.begin(), get(boost::vertex_index, *this->m_c));

    weightmap = get(boost::edge_weight, g);
    indexmap =  get(boost::vertex_index, g);
    vis = vis_type();

    typedef typename boost::property_traits<cmap_type>::value_type ColorValue;
    typedef boost::color_traits<ColorValue> Color;
    typename boost::graph_traits<C>::vertex_iterator ui, ui_end;
    for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui) {
      vis.initialize_vertex(*ui, g);
      put(distmap, *ui, max);
      put(parentmap, *ui, *ui);
      put(colormap, *ui, Color::white());
    }
    put(distmap, *src, 0);
  }

  void run() { 
    dijkstra_shortest_paths_no_init(*this->m_c, *src, parentmap, 
                                    distmap, weightmap, indexmap, 
                                    comp, combine, 0, vis, colormap);
  }
};

/**
 *  Profiler for DijkstraSSSP; expects the necessary data to be stored
 *  on the graph's vertices. The tags expected are vertex_distance_t
 *  and vertex_name_t. distance corresponds to... distance. name
 *  is for the predecessor/parent map parameter that holds the
 *  shortest paths. All the value types are expected to be size_t.
 *
 *  Problems will almost certainly occur if we change the vertex storage
 *  to std::list, since we get the vertex_index from within the graph, but
 *  boost's adjacency_list will not provide this automatically if vertex
 *  storage is std::list.
 */
template<class C, class Counter=counter<default_timer> >
class dsssp_in_profiler_bgl
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>                     base_type;
  typedef boost::graph_traits<C>                             traits;
  typedef typename traits::vertex_descriptor                 vd_type;
  typedef typename traits::vertex_iterator                   vi_type;
  typedef boost::default_dijkstra_visitor                    vis_type;
 
  typedef typename boost::property_map
                     <C, boost::vertex_name_t>::type        pmap_type; 
  typedef typename boost::property_map
                     <C, boost::edge_weight_t>::type        wmap_type;
  typedef typename boost::property_map
                     <C, boost::vertex_distance_t>::type    dmap_type;
  typedef typename boost::property_map
                     <C, boost::vertex_index_t>::type       imap_type;
  typedef typename boost::property_map
                     <C, boost::vertex_color_t>::type       cmap_type;
  
  typedef typename boost::property_traits<wmap_type>::value_type  ep_t;

  typedef std::less<ep_t> comp_t;
  typedef std::plus<ep_t> combine_t;

  vi_type           src;
  wmap_type         weightmap;
  pmap_type         parentmap;
  dmap_type         distmap;
  imap_type         indexmap;
  cmap_type         colormap;
  vis_type          vis;
  ep_t              max;
  comp_t            comp;
  combine_t         combine;

 public:
 dsssp_in_profiler_bgl(std::string pcname, C* pc, int argc=0, char **argv=NULL) : 
  base_type(pc,pcname+"_dijkstra_bgl", argc, argv) { 
    this->n_times = 1;
    max = std::numeric_limits<ep_t>::max();
    comp = std::less<ep_t>();
    combine = std::plus<ep_t>();
  }

  void initialize_iteration() {
    C &g = *this->m_c;
    src = vertices(g).first; //should be VD == 0
    weightmap = get(boost::edge_weight, g);
    parentmap = get(boost::vertex_name, g);
    distmap = get(boost::vertex_distance, g);
    indexmap = get(boost::vertex_index, g);
    colormap= get(boost::vertex_color, g);
    vis = vis_type();

    //init stuff
    typedef typename boost::property_traits<cmap_type>::value_type ColorValue;
    typedef boost::color_traits<ColorValue> Color;
    typename boost::graph_traits<C>::vertex_iterator ui, ui_end;
    for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui) {
      vis.initialize_vertex(*ui, g);
      put(distmap, *ui, max);
      put(parentmap, *ui, *ui);
      put(colormap, *ui, Color::white());
    }
    put(distmap, *src, 0);
  }

  void run() { 
    dijkstra_shortest_paths_no_init(*this->m_c, *src, parentmap, 
                                    distmap, weightmap, indexmap, 
                                    comp, combine, 0, vis, colormap);
  }
};

}//namespace stapl

#endif
