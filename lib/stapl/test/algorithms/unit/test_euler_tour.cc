/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/vector.hpp>
#include <stapl/set.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/binary_tree.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <stapl/algorithms/euler_tour.hpp>
#include "../../test_report.hpp"

using namespace std;
using namespace stapl;

//------------- graph -------------
typedef stapl::dynamic_graph<UNDIRECTED, MULTIEDGES, size_t, size_t> graph_type;
typedef graph_type::edge_descriptor                        ED;
typedef graph_type::vertex_descriptor                      vertex_descriptor;
typedef graph_type::vertex_iterator                        vertex_iterator;
typedef graph_type::vertex_property                        vertex_property;
typedef stapl::graph_view<graph_type>                      graph_view_type;

//------------- vector -------------
typedef stapl::vector<ED>                                  vector_type;
typedef stapl::vector_view<vector_type>                    vector_view_type;

//------------- set -------------
typedef continuous_domain<ED>                               domain_type;
typedef explicit_partition<domain_type>                     partition_type;
typedef stapl::set<ED, std::less<ED>, partition_type>       set_type;
typedef set_view<set_type>                                  set_view_type;



namespace stapl {
inline
bool operator<(ED const& ed1, ED const& ed2)
{
  if (ed1.source() < ed2.source())
    return true;
  else if (ed1.source() == ed2.source()) {
    if (ed1.target() < ed2.target())
      return true;
    else
      return false;
  }
  else
    return false;
}

}//stapl namespace

template<typename ED>
struct follow
{
  typedef ED result_type;

  ED m_false;

  follow(ED fake)
    : m_false(fake)
  {}

  ED operator()(ED ed1, ED ed2)
  {
    if (ed1.target() == ed2.source())
      return ED(ed1.source(), ed2.target());
    else
      return m_false;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_false);
  }
};

template<typename ED>
struct valid_edge
{
  typedef bool result_type;

  bool operator()(ED e)
  {
    size_t val;
    if (e.target() != e.source()){
      if (e.target() > e.source()){
        val = e.target()-2*e.source();
      }else{
        val = e.source()-2*e.target();
      }
      return val==1 || val==2;
    }
    else
      return false;
  }
};

template<typename T>
struct insert
{
  template<typename Val, typename PSView>
  void operator()(Val v, PSView psview)
  {
    T value = v;
    psview.insert(value);
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{

  stapl::counter<stapl::default_timer> t;
  double tps;

  //------------- graph -------------
  graph_type pg;
  size_t num_levels = 3;
  if (argc==2)
    num_levels = atoi(argv[1]);

  size_t N = pow(2, num_levels)-1;
  graph_view_type pgview(pg);
  generators::make_binary_tree(pgview, N, false);

  //------------- vector -------------
  vector_type pvect(2*pgview.num_edges());
  vector_view_type pvview(pvect);

t.start();

  euler_tour(pgview, pvview);

tps = t.stop();
if (get_location_id()==0) std::cout <<  "Tour    : "
                                    << tps << std::endl << std::endl;
t.reset();


  //Tests :
  bool passed;
  size_t size = 2*pgview.num_edges();
  //Size
  STAPL_TEST_REPORT(pvect.size()==size,"Checking size        ");

  //Valid edges
  passed = map_reduce(valid_edge<ED>(), stapl::logical_and<bool>(), pvview);
  STAPL_TEST_REPORT(passed,"Checking valid edges ");

  //Unicity
  //------------- map -------------
  domain_type dom(ED(0,0), ED(N-1,N-1));
  std::vector<domain_type> doms;
  size_t num_loc = get_num_locations();
  for (size_t i = 0; i < num_loc-1; ++i){
    doms.push_back(domain_type(ED(i*N/num_loc, i*N/num_loc),
                   ED((i+1)*N/num_loc, (i+1)*N/num_loc-1)));
  }
  doms.push_back(domain_type(ED((num_loc-1)*N/num_loc, (num_loc-1)*N/num_loc),
                   ED(N-1, N-1)));
  partition_type part(dom, doms);
  set_type ps(part);
  set_view_type psview(ps);

  map_func(insert<ED>(),
           pvview, make_repeat_view(psview));

  STAPL_TEST_REPORT(psview.size()==size,"Checking unique edges");

  //Follow
  ED fake(-1, -1);
  ED e = map_reduce(stapl::identity<ED>(), follow<ED>(fake), pvview);
  STAPL_TEST_REPORT(e!=fake && e.source()==e.target(),"Checking following   ");

  return EXIT_SUCCESS;
}
