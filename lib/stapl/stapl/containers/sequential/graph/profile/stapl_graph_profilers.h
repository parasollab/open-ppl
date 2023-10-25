/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_STAPL_GRAPH_PROFILERS_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_STAPL_GRAPH_PROFILERS_HPP

#include <limits>
#include <vector>
#include "profile_utils.h"
#include "container_profiler.h"
#include <stapl/runtime/counter/default_counters.hpp>
#include "../algorithms/dijkstra.h"
#include "../algorithms/depth_first_search.h"
#include "../algorithms/breadth_first_search.h"
#include "../algorithms/graph_algo_util.h"
#include <stapl/containers/sequential/graph/algorithms/graph_algo_util.h>
#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>

//////////////////////////////////////////////////////////////////////
/// @file
/// @brief Contains profilers for the stapl sequential graph.
//////////////////////////////////////////////////////////////////////

namespace stapl {

//====================================================================
// Basic profilers.
//====================================================================
template <class C, class Counter=counter<default_timer> >
class add_vertex_profiler
  : public container_profiler<C, Counter>
{
private:
  typedef container_profiler<C, Counter> base_type;

public:
  add_vertex_profiler(std::string pcname, C* pc, profile_config& config) :
    base_type(pc,pcname+"_add_vertex", config.m_argc, config.m_argv) {
      this->n_times = config.m_verts.size();
  }

  void run() {
    for (size_t i=0; i < this->n_times; ++i) this->m_c->add_vertex();
  }

  void finalize_iteration(){
    this->m_c->clear();
  }
};

/**
 * Delete random vertices from a graph.
 */
template <class C, class Counter=counter<default_timer> >
class delete_vertex_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef typename C::vertex_descriptor           vd_type;
  typedef std::vector<vd_type>                    verts_type;
  typedef typename verts_type::iterator           vit_type;

  size_t     m_num_verts;
  verts_type m_to_del; //the VDs to delete are precomputed.
  vit_type   it, it_end;

  profile_config& m_config;

public:
  delete_vertex_profiler(std::string pcname, C* pc, profile_config& config)
   : base_type(pc,pcname+"_delete_vertex", config.m_argc, config.m_argv),
     m_config(config)
  {
    m_num_verts = config.m_verts.size();
    this->n_times = config.m_vert_del_list.size();

    if (this->n_times > m_num_verts) {
      std::cerr << "ASSERT: delete_vertex_profiler"
                << " - Requested to delete too many vertices.\n";
      std::cerr << "VDC: " << this->n_times
                << "\n|V|: " << m_num_verts << std::endl;
      exit(1);
    }

    profile_config::verts_type::iterator vit, vit_end;
    vit = config.m_vert_del_list.begin();
    vit_end = config.m_vert_del_list.end();
    m_to_del = verts_type(vit,vit_end);
  }

  void initialize_iteration() {
    //add the vertices specified by the config.
    profile_config::verts_type::iterator vit, vit_end;
    vit = m_config.m_verts.begin();
    vit_end = m_config.m_verts.end();
    for (; vit != vit_end; ++vit)
      this->m_c->add_vertex(*vit, typename C::vertex_property());

    it = m_to_del.begin();
    it_end = m_to_del.end();
  }

  void run() {
    C &g = *this->m_c;
    for (; it!=it_end; ++it)
      g.delete_vertex(*it);
  }

  void finalize_iteration() { this->m_c->clear(); }
};

/**
 * Add edges to a relational pcontainer (graph/p_graph)
 */
template <class C, class Counter=counter<default_timer> >
class add_edge_profiler
  : public container_profiler<C,Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef profile_config::edges_type              edges_type;
  typedef profile_config::edges_type::iterator    ei_type;

  edges_type m_edges;
  ei_type    it, it_end;

  profile_config& m_config;

public:
  add_edge_profiler(std::string pcname, C* pc, profile_config& config)
    : base_type(pc,pcname+"_add_edge", config.m_argc, config.m_argv),
      m_config(config)
  {
    this->n_times = config.m_edges.size();
    m_edges = edges_type(config.m_edges.begin(), config.m_edges.end());
  }

  void initialize_iteration() {
    profile_config::verts_type::iterator vit, vit_end;
    vit = m_config.m_verts.begin();
    vit_end = m_config.m_verts.end();
    for (; vit != vit_end; ++vit)
      this->m_c->add_vertex(*vit, typename C::vertex_property());

    it = m_edges.begin();
    it_end = m_edges.end();
  }

  void run() {
    C &g = *this->m_c;
    for (; it != it_end; ++it)
      g.add_edge(it->first, it->second);
  }

  void finalize_iteration() { this->m_c->clear(); }
};

/**
 * Delete edges of a relational pcontainer (graph/p_graph)
 */
template <class C, class Counter=counter<default_timer> >
class delete_edge_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef typename C::vertex_descriptor           vd_type;
  typedef std::vector<std::pair<vd_type, vd_type> > edges_type;
  typedef typename edges_type::iterator           ei_type;

  edges_type m_to_del;
  ei_type it, it_end;

  profile_config& m_config;

public:
  delete_edge_profiler(std::string pcname, C* pc, profile_config& config)
    : base_type(pc,pcname+"_delete_edge", config.m_argc, config.m_argv),
      m_config(config)
  {
     this->n_times = config.m_edge_del_list.size();
     m_to_del = edges_type(config.m_edge_del_list.begin(),
                           config.m_edge_del_list.end());
  }

  void initialize_iteration() {
    //builds a graph from the blueprint stored in m_config
    m_config.rebuild_graph(*this->m_c);

    it = m_to_del.begin();
    it_end = m_to_del.end();
  }

  void run() {
    C &g = *this->m_c;
    for (;it!=it_end;++it)
      g.delete_edge(it->first, it->second);
  }

  void finalize_iteration() {
    this->m_c->clear();
  }
};

/**
 * Find vertices in a relational pcontainer (graph/p_graph)
 */
template <class C, class Counter=counter<default_timer> >
class find_vertex_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef typename C::vertex_descriptor           vd_type;
  typedef std::vector<vd_type>                    verts_type;
  typedef typename verts_type::iterator           vi_type;

  verts_type  m_verts;
  vi_type     it, it_end;

  profile_config& m_config;

public:
  find_vertex_profiler(std::string pcname, C* pc, profile_config& config)
    : base_type(pc,pcname+"_find_vertex", config.m_argc, config.m_argv),
      m_config(config)
  {
    this->n_times = config.m_verts.size();
    m_verts = verts_type(config.m_verts.begin(), config.m_verts.end());
    rs_rand rnd(time(NULL), this->n_times);
    std::random_shuffle(m_verts->begin(), m_verts->end(), rnd);
  }

  void initialize_iteration() {
    //build the graph specified by the blueprint in m_config
    profile_config::verts_type::iterator cvit, cvit_end;
    cvit_end = m_config.m_verts.end();
    for (cvit = m_config.m_verts.begin(); cvit != cvit_end; ++cvit)
      this->m_c->add_vertex(*cvit);

    it = m_verts.begin();
    it_end = m_verts.end();
  }

  void run() {
    typename C::vertex_iterator vit;
    for (; it != it_end; ++it)
      vit = this->m_c->find_vertex(*it);
  }

  void finalize_iteration() {
    this->m_c->clear();
  }
};

/**
 * Find Edges in a relational pcontainer (graph/p_graph)
 *
 * Left empty for now; easy to build if we end up interested.
 */
template <class C, class Counter=counter<default_timer> >
class find_edge_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter> base_type;

public:
  find_edge_profiler(std::string pcname, C* pc, profile_config& config) :
   base_type(pc,pcname+"_find_edge", config.m_argc, config.m_argv) {
    this->n_times = config.m_edges.size();
  }

  void initialize_iteration() {}

  void run(){}

  void finalize_iteration() {}
};

//====================================================================
// Traversal profilers
//====================================================================
/**
 * Traverse each vertex's out-edge list.
 */
template <class C, class Counter=counter<default_timer> >
class traversal_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef typename C::vertex_iterator             vertex_iterator;
  typedef typename C::adj_edge_iterator           adj_edge_iterator;

  size_t            m_sum;
  vertex_iterator   vi, vi_end;
  adj_edge_iterator aei, aei_end;

 public:
 traversal_profiler(std::string pcname, C* pc, int argc=0, char **argv=NULL) :
   m_sum(0),
   base_type(pc,pcname+"_traversal", argc, argv) {}

  void initialize_iteration() {
    m_sum = 0;
    vi = this->m_c->begin();
    vi_end = this->m_c->end();
  }

  void run(){
    for (; vi != vi_end; ++vi) {
      aei = (*vi).begin();
      aei_end = (*vi).end();
      for (; aei != aei_end; ++aei) {
        m_sum++;
      }
    }
  }
};

//====================================================================
// Algorithm profilers
//====================================================================
/**
 * Run BFS on a pre-constructed STAPL graph.
 */
template <class C, class Counter=counter<default_timer> >
class bfs_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>            base_type;
  typedef visitor_test<C>                           vis_type;
  typedef boost::queue<typename C::vertex_iterator> queue_type;


  //we fully instantiate the vector property map so we can pre-specify
  //the size of the vector; this allows us to match boost's iterator
  //property map using a vector for storage.
  typedef std::vector<int>                                        color_cont_t;
  typedef sequential::graph_external_property_map<C,color_cont_t> color_map_t;

  size_t        bfs;
  color_cont_t* color_cont;
  color_map_t*  cmap;
  vis_type      vis;
  queue_type    queue;

 public:
 bfs_profiler(std::string pcname, C* pc, int argc=0, char **argv=NULL) :
  base_type(pc,pcname+"_bfs", argc, argv) {
    this->n_times = 1; //just once per iter.
  }

  void initialize_iteration() {
    color_cont = new color_cont_t((*this->m_c).get_num_vertices());
    cmap = new color_map_t(*color_cont);
    queue = queue_type();
    bfs = 0;
    vis = vis_type(&bfs);
    cmap->reset();
  }

  void run() {
    breadth_first_search1(*this->m_c,0,vis,*cmap,queue);
  }

  void finalize_iteration() {
    //bfs = vis.m_sum;
    delete cmap;
    delete color_cont;
  }
};

/**
 * Run BFS on a pre-constructed STAPL graph. Expects the colors for the
 * BFS algo. to be stored on the graph's vertices.
 */
template <class C, class Counter=counter<default_timer> >
class bfs_internal_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>            base_type;
  typedef typename C::vertex_descriptor             vd_t;
  /*
  typedef typename C::vertex_property               vp_t;
  typedef ident_prop_func<vp_t>                     func_t;
  typedef graph_property_map<C,func_t>              color_map_t;
  */
  typedef vertex_property_map<C>                    color_map_t;
  typedef visitor_test<C>                           vis_t;
  typedef boost::queue<typename C::vertex_iterator> queue_t;

  size_t        bfs;
  color_map_t*  cmap;
  vis_t         vis;
  queue_t       queue;

 public:
  bfs_internal_profiler(std::string pcname, C* pc, int argc=0, char **argv=NULL)
    : base_type(pc,pcname+"_bfs", argc, argv)
  {
    this->n_times = 1; //just once per iter.
  }

  void initialize_iteration() {
    cmap = new color_map_t(*this->m_c);
    bfs = 0;
    vis = vis_t(&bfs);
    queue = queue_t();
    cmap->reset();
  }

  void run() {
    breadth_first_search1(*this->m_c,0,vis,*cmap,queue);
  }

  void finalize_iteration() {
    //std::cout<<"STAPL BFS: "<<bfs<<std::endl;
    delete cmap;
  }
};

/**
 * Run DFS on a pre-constructed STAPL graph.
 */
template <class C, class Counter=counter<default_timer> >
class dfs_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C,Counter>               base_type;
  typedef visitor_test<C>                             vis_type;

  //we fully instantiate the vector property map so we can pre-specify
  //the size of the vector; this allows us to match boost's iterator
  //property map using a vector for storage.
  typedef std::vector<size_t>                                     color_cont_t;
  typedef sequential::graph_external_property_map<C,color_cont_t> color_map_t;

  size_t        dfs;
  color_cont_t* color_cont;
  color_map_t*  cmap;
  vis_type      vis;

 public:
 dfs_profiler(std::string pcname, C* pc, int argc=0, char **argv=NULL) :
  base_type(pc,pcname+"_dfs", argc, argv) {
    this->n_times = 1; //just once per iter.
  }

  void initialize_iteration() {
    color_cont = new color_cont_t(this->m_c->get_num_vertices());
    cmap = new color_map_t(*color_cont);
    dfs = 0;
    vis = vis_type(&dfs);
  }

  void run() {
    depth_first_search(*this->m_c,0,vis,*cmap);
  }

  void finalize_iteration() {
    delete cmap;
    delete color_cont;
  }
};

/**
 * Run DFS on a pre-constructed STAPL graph.
 */
template <class C, class Counter=counter<default_timer> >
class dfs_internal_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>          base_type;
  typedef typename C::vertex_descriptor           vd_t;
  typedef typename C::vertex_property             vp_t;
  typedef vertex_property_map<C>                  color_map_t;
  typedef visitor_test<C>                         vis_t;

  size_t        dfs;
  color_map_t*  cmap;
  vis_t         vis;

public:
  dfs_internal_profiler(std::string pcname, C* pc, int argc=0, char **argv=NULL)
    : base_type(pc,pcname+"_dfs", argc, argv)
  {
    this->n_times = 1; //just once per iter.
  }

  void initialize_iteration() {
    cmap = new color_map_t(*this->m_c);
    dfs = 0;
    vis = vis_t(&dfs);
    cmap->reset();
  }

  void run() {
    depth_first_search(*this->m_c,0,vis,*cmap);
  }

  void finalize_iteration() {
    delete cmap;
  }
};

/**
 * Profiles Dijkstra's Algorithm.
 */
template <class C, class Counter=counter<default_timer> >
class dijkstra_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter> base_type;
  typedef typename C::vertex_descriptor  vd_t;
  typedef typename C::vertex_reference   vr_t;
  typedef typename C::edge_property      ep_t;
  typedef int                             color_value_t;

  //unlike the bfs/dfs external profilers, we go ahead and use
  //the shorter instantiation since we pre-fill the property map;
  //it ends up at the right size anyway.
  typedef vector_property_map<C, vd_t>          pmap_t;
  typedef vector_property_map<C, ep_t>          dmap_t;
  typedef vector_property_map<C, color_value_t> cmap_t;

  typedef boost::scoped_array<vd_t>                imap_cont_t;
  typedef ident_prop_func<vd_t>                    imap_func_t;
  typedef graph_external_property_map<C,
                                      imap_cont_t,
                                      imap_func_t> imap_t;
  typedef std::less<ep_t>                          comp_t;
  typedef std::plus<ep_t>                          combine_t;

  typedef d_ary_heap_indirect<vr_t, 4, imap_t, dmap_t, comp_t>   queue_t;

  ep_t        weight_max;
  pmap_t*     pmap;
  dmap_t*     dmap;
  cmap_t*     cmap;
  imap_cont_t index_map_cont;
  imap_t*     index_map;
  comp_t      comp;
  combine_t   combine;
  queue_t*    queue;

 public:
 dijkstra_profiler(std::string pcname, C* pc, int argc=0, char **argv=NULL) :
  base_type(pc,pcname+"_dijkstra", argc, argv) {
    this->n_times = 1; //just once per iter.
    weight_max = std::numeric_limits<ep_t>::max();
    comp = comp_t();
    combine = combine_t();
  }

  void initialize_iteration() {
    C &g = *this->m_c;
    pmap = new pmap_t();
    dmap = new dmap_t();
    cmap = new cmap_t();

    //index map is needed by the d_ary_heap.
    index_map_cont.reset(new vd_t[g.get_max_descriptor()]);
    std::fill(index_map_cont.get(),
              index_map_cont.get() + g.get_max_descriptor(), vd_t());
    index_map = new imap_t(index_map_cont);
    queue = new queue_t(*dmap, *index_map, comp);

    typename C::vertex_iterator vi, vi_end;
    vi = g.begin(); vi_end = g.end();
    for (; vi != vi_end; ++vi) {
      dmap->put(*vi, weight_max);
      pmap->put(*vi, (*vi).descriptor());
      cmap->put(*vi, stapl::graph_color<color_value_t>::white());
    }

    //set the source
    dmap->put(*g.begin(), ep_t());
  }

  void run() {
    stapl::sequential::dijkstra_sssp(*this->m_c, *pmap, *dmap, *cmap, *queue,
                            (*this->m_c->begin()).descriptor(), comp, combine);
  }

  void finalize_iteration() {
    delete pmap;
    delete dmap;
    delete cmap;
    delete index_map;
    delete queue;
  }
};

/**
 * Profiles Dijkstra's Algorithm, storing the Dijkstra data on the
 * graph's vertices. This expects the graph's vertex property to be
 * dijkstra_property objects.
 */
template <class C, class Counter=counter<default_timer> >
class dsssp_in_profiler
  : public container_profiler<C, Counter>
{
  typedef container_profiler<C, Counter>            base_type;

  typedef typename C::vertex_descriptor             vd_t;
  typedef typename C::vertex_reference              vr_t;
  typedef typename C::vertex_property               vp_t;
  typedef typename C::edge_property                 ep_t;

  typedef dijkstra_pred_func<vp_t>                  pmap_func_t;
  typedef dijkstra_dist_func<vp_t>                  dmap_func_t;
  typedef dijkstra_color_func<vp_t>                 cmap_func_t;
  typedef vertex_property_map<C, pmap_func_t>       pred_map_t;
  typedef vertex_property_map<C, dmap_func_t>       dist_map_t;
  typedef vertex_property_map<C, cmap_func_t>       color_map_t;

  typedef std::less<ep_t>                           comp_t;
  typedef std::plus<ep_t>                           combine_t;

  typedef boost::scoped_array<vd_t>                 imap_cont_t;
  typedef ident_prop_func<vd_t>                     imap_func_t;
  typedef graph_external_property_map<C,
                                      imap_cont_t,
                                      imap_func_t>  index_map_t;
  typedef d_ary_heap_indirect<vr_t, 4, index_map_t,
                              dist_map_t, comp_t>   queue_t;

  ep_t              weight_max;
  pred_map_t*       pmap;
  dist_map_t*       dmap;
  color_map_t*      cmap;
  imap_cont_t       imap_cont;
  index_map_t*      imap;
  queue_t*          queue;
  comp_t            comp;
  combine_t         combine;

 public:
 dsssp_in_profiler(std::string pcname, C* pc, int argc=0, char **argv=NULL) :
  base_type(pc,pcname+"_dijkstra", argc, argv) {
    this->n_times = 1; //just once per iter.
    weight_max = std::numeric_limits<ep_t>::max();
    comp = comp_t();
    combine = combine_t();
  }

  void initialize_iteration() {
    C &g = *this->m_c;
    pmap = new pred_map_t(g, pmap_func_t());
    dmap = new dist_map_t(g, dmap_func_t());
    cmap = new color_map_t(g, cmap_func_t());

    imap_cont.reset(new vd_t[g.get_max_descriptor()]);
    std::fill(imap_cont.get(), imap_cont.get()+g.get_max_descriptor(),vd_t());
    imap = new index_map_t(imap_cont, imap_func_t());
    queue = new queue_t(*dmap, *imap, comp);

    typename C::vertex_iterator vi, vi_end;
    vi = g.begin(); vi_end = g.end();
    for (; vi != vi_end; ++vi) {
      dmap->put(*vi, weight_max);
      pmap->put(*vi, (*vi).descriptor());
      cmap->put(*vi,
              graph_color<typename color_map_t::property_value_type>::white());
    }

    //set the source
    dmap->put(*g.begin(), ep_t());
  }

  void run() {
    sequential::dijkstra_sssp(*this->m_c, *pmap, *dmap, *cmap, *queue,
                          (*(this->m_c->begin())).descriptor(), comp, combine);
  }

  void finalize_iteration() {
    delete pmap;
    delete dmap;
    delete cmap;
    delete imap;
    delete queue;
  }
};

} // namespace stapl

#endif
