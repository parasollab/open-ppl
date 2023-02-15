/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/utility/do_once.hpp>
#include "../adt.hpp"
#include "../profiling_util.hpp"
#include "../graph.hpp"

using namespace stapl;
using namespace profiling;

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes three containers: one with local vertex descriptors, one
/// with remote vertex descriptors, and one with edges descriptors.
///
/// @param p The source container
/// @param vit An iterator for the source container
/// @param lvv The container of local vertex descriptors
/// @param vv The container of remote vertex descriptors
/// @param ve The container of edge descriptors
////////////////////////////////////////////////////////////////////////////////
template <class ADT, class VIT, class vverts_type, class vedges_type>
void init (ADT& p, VIT vit, vverts_type& lvv, vverts_type& vv, vedges_type& ve,
           size_t NElems)
{
  typedef typename ADT::vertex_descriptor VD;
  typedef VIT vit_type;

  rand_gen random_num(NElems);

  array<VD> p_verts(NElems);
  size_t nlv   = NElems / get_num_locations();
  size_t start = nlv*get_location_id();
  size_t stop  = (get_location_id() == get_num_locations()-1) ?
    NElems : nlv*(get_location_id()+1);

  for (size_t i=start; i<stop; ++i) {
    VD vd = (*vit).descriptor();
    p_verts[i] = vd;
    lvv.push_back(vd);
    ++vit;
  }
  stapl::rmi_fence();

  for (size_t i=start; i<stop; ++i)
    vv.push_back(p_verts[random_num() % NElems]);

  for (size_t i=0; i<NElems/get_num_locations(); ++i) {
    //the source vertex is local and  the destination is local or remote
    //some set of random edges
    ve.push_back(std::pair<VD,VD>(lvv[random_num() % (stop-start)],
                                              p_verts[random_num() % NElems]));
  }

  stapl::rmi_fence();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes an iterator for the container if one is not provided
///
/// @param p The source container
/// @param lvv The container of local vertex descriptors
/// @param vv The container of remote vertex descriptors
/// @param ve The container of edge descriptors
////////////////////////////////////////////////////////////////////////////////
template <class ADT, class vverts_type, class vedges_type>
void init (ADT& p, vverts_type& lvv, vverts_type& vv, vedges_type& ve,
           size_t NElems)
{
  typename ADT::distribution_type::container_manager_type
               ::base_container_type::vertex_iterator vit
    = p.distribution().container_manager().begin()->begin();

  init(p, vit, lvv, vv, ve, NElems);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the add_vertex() and add_edge() functions.
///
/// @param name A string containing the name of the container
/// @param p The container to use for testing
/// @param NElems The size of the container
////////////////////////////////////////////////////////////////////////////////
template <class ADT>
void profile_av(std::string name, ADT& p, size_t NElems, int argc, char** argv)
{
  typedef typename ADT::vertex_descriptor VD;
  size_t nverts = NElems/get_num_locations();

  nverts += (get_location_id() == get_num_locations()-1) ? NElems - nverts : 0;

  std::vector<VD>     lverts;
  std::vector<VD>     rverts;
  std::vector<std::pair<VD,VD> > redges;
  typename std::vector<std::pair<VD,VD> >::iterator it;

  for (size_t i=0; i<nverts; ++i)
    p.add_vertex();
  rmi_fence();

  init(p, lverts, rverts, redges, NElems);

  add_vertex_profiler<ADT/*, counter_type*/>
    avp(name, &p, nverts, argc, argv);
  avp.collect_profile();
  avp.report();

  add_edge_profiler<ADT/*, counter_type*/>
    aep(name, &p, nverts, redges, argc, argv);
  aep.collect_profile();
  aep.report();

  rmi_fence();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the remaining graph functions.
///
/// @param name A string containing the name of the container
/// @param p The container to use for testing
/// @param NElems The size of the container
////////////////////////////////////////////////////////////////////////////////
template <class ADT>
void profile(std::string name, ADT& p, size_t NElems, int argc, char** argv)
{
  typedef typename ADT::vertex_descriptor VD;
  size_t nverts = NElems/get_num_locations();

  nverts += (get_location_id() == get_num_locations()-1) ? NElems - nverts : 0;

  std::vector<VD>     lverts;
  std::vector<VD>     rverts;
  std::vector<std::pair<VD,VD> > redges;
  typename std::vector<std::pair<VD,VD> >::iterator it;

  p.clear();
  rmi_fence();

  for (size_t i=0; i<nverts; ++i)
    p.add_vertex();

  rmi_fence();

  init(p, lverts, rverts, redges, NElems);

  rmi_fence();

  for (it = redges.begin(); it != redges.end(); ++it)
    p.add_edge(typename ADT::edge_descriptor(it->first, it->second));

  rmi_fence();

  find_vertex_profiler<ADT/*, counter_type*/>
    fvp(name, &p, nverts, rverts, argc, argv);
  fvp.collect_profile();
  fvp.report();

  find_vertex_profiler<ADT/*, counter_type*/>
    flvp(name+"_localvds", &p, nverts, lverts, argc, argv);
  flvp.collect_profile();
  flvp.report();

  // FIXME: change for has_edge function
  // find_edge_profiler<ADT/*, counter_type*/>
  //   fep(name, &p, nverts, redges, argc, argv);
  // fep.collect_profile();
  // fep.report();

  delete_edge_profiler<ADT/*, counter_type*/>
    dep(name, &p, nverts, redges, argc, argv);
  dep.collect_profile();
  dep.report();

  add_edge_async_profiler<ADT/*, counter_type*/>
    aeap(name, &p, nverts, redges, argc, argv);
  aeap.collect_profile();
  aeap.report();

  size_t nsz = 1000<lverts.size()?1000:lverts.size();
  lverts.resize(nsz);

  p.clear();
  lverts.clear();
  rverts.clear();
  redges.clear();
  rmi_fence();

  for (size_t i=0; i<nverts; ++i)
    p.add_vertex();

  rmi_fence();

  init(p, lverts, rverts, redges, NElems);

  rmi_fence();


  delete_vertex_profiler<ADT/*, counter_type*/>
    dvp(name, &p, nverts, lverts, argc, argv);
  dvp.collect_profile();
  dvp.report();

  rmi_fence();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for all of the sequential graph functions.
///
/// @param name A string containing the name of the container
/// @param p The container to use for testing
/// @param NElems The size of the container
////////////////////////////////////////////////////////////////////////////////
template <class ADT>
void profile_seq(std::string name, ADT& p, size_t NElems, int argc, char** argv)
{
  if (get_num_locations()!=1)
    return;

  typedef typename ADT::vertex_descriptor VD;
  size_t nverts = NElems/get_num_locations();

  std::vector<VD>     lverts;
  std::vector<VD>     rverts;
  std::vector<std::pair<VD,VD> > redges;
  typename std::vector<std::pair<VD,VD> >::iterator it;

  for (size_t i=0; i<nverts; ++i)
    p.add_vertex();

  init(p, p.begin(), lverts, rverts, redges, NElems);

  add_vertex_profiler<ADT/*, counter_type*/>
    avp(name, &p, nverts, argc, argv);
  avp.collect_profile();
  avp.report();

  add_edge_profiler<ADT/*, counter_type*/>
    aep(name, &p, nverts, redges, argc, argv);
  aep.collect_profile();
  aep.report();

  p.clear();

  for (size_t i=0; i<nverts; ++i)
    p.add_vertex();

  for (it = redges.begin(); it != redges.end(); ++it)
    p.add_edge(typename ADT::edge_descriptor(it->first, it->second));

  find_vertex_profiler<ADT/*, counter_type*/>
    fvp(name+"_random", &p, nverts, rverts, argc, argv);
  fvp.collect_profile();
  fvp.report();

  find_vertex_profiler<ADT/*, counter_type*/>
    flvp(name+"_linear", &p, nverts, lverts, argc, argv);
  flvp.collect_profile();
  flvp.report();

  find_edge_profiler<ADT/*, counter_type*/>
    fep(name, &p, nverts, redges, argc, argv);
  fep.collect_profile();
  fep.report();

  delete_edge_profiler<ADT/*, counter_type*/>
    dep(name, &p, nverts, redges, argc, argv);
  dep.collect_profile();
  dep.report();

  add_edge_profiler<ADT/*, counter_type*/>
    ae2ap(name+"_2", &p, nverts, redges, argc, argv);
  ae2ap.collect_profile();
  ae2ap.report();

  size_t nsz = 1000<lverts.size() ? 1000:lverts.size();
  lverts.resize(nsz);
  delete_vertex_profiler<ADT/*, counter_type*/>
   dvp(name, &p, nverts, lverts, argc, argv);
   dvp.collect_profile();
   dvp.report();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t case_id;
  size_t NElems;

  stapl::do_once([]{
     std::cout << "p_graph Container Performance Evaluation\n";
     });

  if (argc > 2) {
    case_id=atoi(argv[1]);
    NElems = atoi(argv[2]);
  }
  else {
    stapl::do_once([]{
    std::cout << "Input size NElems required;\n"
              << "Using NElems=10 vertices and 2*NElems edges by default\n";
        });
    NElems=10;
    case_id=0;
  }

  if (case_id==0 && get_num_locations()==1) {
    stapl_print(">>>  Test graph<DIRECTED,MULTIEDGES> \n");
    typedef sequential::graph<DIRECTED,MULTIEDGES> GDM;
    GDM  gdm;
    profile_seq("graph<DIRECTED,MULTIEDGES>", gdm, NElems, argc, argv);
  }

  if (get_num_locations() > 1) {
    stapl_print(">>>  Test p_graph<DIRECTED,MULTIEDGES> \n");
    typedef dynamic_graph<DIRECTED, MULTIEDGES> PGDM;
    PGDM  pgdm;
    profile_av("p_graph<DIRECTED,MULTIEDGES>", pgdm, NElems, argc, argv);
    profile("p_graph<DIRECTED,MULTIEDGES>", pgdm, NElems, argc, argv);
  }

  return EXIT_SUCCESS;
}
