/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include "../../confint.hpp"

#define VB_GRAPH

#ifdef VB_GRAPH
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#endif

typedef stapl::counter<stapl::default_timer> counter_t;


#ifdef VB_GRAPH
namespace vb_functors {


// GID to PID functor that just wraps an indexed_partition and calls its
// .find( GID )
template <typename RaytubeDescrType>
class vertex_desc_to_PID
{
public:
  typedef RaytubeDescrType gid_type;
  typedef size_t index_type;
private:
  size_t m_num_locs;
  gid_type m_min;
  gid_type m_max;
  gid_type m_block;

public:

  vertex_desc_to_PID(size_t num_locs = stapl::get_num_locations())
    : m_num_locs(num_locs),
      m_min(0),
      m_max(100)
  {
    m_block= m_max/m_num_locs - m_min/m_num_locs;
  }


  index_type operator()(gid_type const& g) const
  {
    return g / m_block;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_locs);
    t.member(m_min);
    t.member(m_max);
    t.member(m_block);
  }


  void
  update(std::vector<
           std::tuple<std::pair<gid_type,gid_type>, unsigned long int,
           stapl::location_type>> const&,
         size_t)
  { stapl::abort("mapping_base::update called."); }

};//end vertex_desc_to_PID class



// PID to LID that just wraps a mapper
template <typename GID>
class mapper_functor
{

private:
  stapl::mapper<size_t> m_mapper;
  typedef size_t cid_type;
  typedef typename stapl::indexed_domain<long unsigned int> map_dom_t;
public:
  typedef stapl::location_type index_type;
  typedef GID   gid_type;

  mapper_functor()
    : m_mapper(map_dom_t(stapl::get_num_locations()))
  { }

  index_type operator()(cid_type const& cid) const
  {
    return m_mapper.map(cid);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_mapper);
  }

  void
  update(std::vector<
           std::tuple<std::pair<gid_type,gid_type>, unsigned long int,
           stapl::location_type>> const&,
         size_t)
  { stapl::abort("mapping_base::update called."); }

};


}

#endif

namespace misc_test{

// Mapping only return 1 per element
struct map_wf
{
  typedef int result_type;

  template <typename E>
  result_type operator()(E const e1)
  {
    return 1;
  }
};


// Reduce sums all the "1"s , the total sum is the number of elements
struct reduce_wf
{
  typedef int result_type;

  template <typename E>
  result_type operator()(E const e1, E const e2)
  {
    return (e1 + e2);
  }

};


struct print_wf
{

  typedef void result_type;

  template <typename T>
  void operator()(T const& v) const
  {
      std::cout << "IN_MAP_FUNC:   Hello from a vertex" << std::endl;
  }
};


struct NOP
{
  typedef void result_type;

  template<typename T1>
  void operator()(T1 const&) const
  { }
};


}


using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{

  using std::cout;
  using std::endl;
  using namespace misc_test;

  #ifdef VB_GRAPH
  typedef stapl::distribution_spec<> graph_distribution_spec;
  typedef stapl::view_based_partition<graph_distribution_spec>
                                     graph_vb_partition;
  typedef stapl::view_based_mapper<graph_distribution_spec>
                                     graph_vb_mapping;
  #endif

  typedef stapl::dynamic_graph <stapl::DIRECTED,
                                stapl::MULTIEDGES,
                                int,
                                int
  #ifdef VB_GRAPH
                                ,graph_vb_partition,
                                graph_vb_mapping
  #endif
                                >                              graph_type;
  typedef graph_view<graph_type>                               graph_view_type;


  int desired_n = 100;
  size_t num_iterations = 32;

  if (argc > 2)
  {
    desired_n = boost::lexical_cast<int>(argv[1]);
    num_iterations = boost::lexical_cast<size_t>(argv[2]);
  }
  else
  {
    desired_n = 100;
    num_iterations = 32;
  }

  std::stringstream    add_report_printer, mapred_report_printer;
  std::vector<double>  add_report_times, mapred_report_times;
  counter_t add_timer, mapred_timer;
  bool passed_add = true;
  bool passed_mapred = true;

  add_report_printer << "Test : add_in_empty_vb_arb_graph\n";
  add_report_printer << "Version : STAPL\n";

  mapred_report_printer << "Test : map_reduce_sparse_vb_arb_graph\n";
  mapred_report_printer << "Version : STAPL\n";

  for(size_t iter = 0; iter < num_iterations; ++iter) {


    #ifdef VB_GRAPH
    auto ds = stapl::arbitrary(
      stapl::indexed_domain<typename graph_type::vertex_descriptor>(),
      stapl::get_num_locations(),
      vb_functors::vertex_desc_to_PID<
        typename graph_type::vertex_descriptor>(),
      vb_functors::mapper_functor<typename graph_type::vertex_descriptor>() );


    graph_type g(ds);
    #else
    graph_type g;
    #endif


    // Testing addition of 100 vertices in an empty
    // dynamic graph with viewbased distribution

    add_timer.reset();
    add_timer.start();

    int local_sz = desired_n / (int)stapl::get_num_locations();
    for (int i = 0; i < local_sz ; ++i)
    {
      g.add_vertex(i + local_sz  * (int)stapl::get_location_id(), i,
                   misc_test::NOP());
    }
    stapl::rmi_fence();

    add_timer.stop();
    add_report_times.push_back(add_timer.value());

    int real_n = local_sz * (int)stapl::get_num_locations();
    passed_add = passed_add & ((int)g.size() == real_n);


    // Testing map_reduce on a partially populated dynamic
    // graph with viewbased distribution

    graph_view_type g_vw(g);
    mapred_timer.reset();
    mapred_timer.start();

    int total_size = map_reduce(map_wf(), reduce_wf(), g_vw);

    mapred_timer.stop();
    mapred_report_times.push_back(mapred_timer.value());

    passed_mapred = passed_mapred & (total_size == real_n);

    // To see printed 100 times "Hello from a vertex" :
    // map_func(print_wf(), cv2);

  }

  if (passed_add)
    add_report_printer << "Status : PASS\n";
  else
    add_report_printer << "Status : FAIL\n";
  report(add_report_printer, add_report_times);

  if (passed_mapred)
    mapred_report_printer << "Status : PASS\n";
  else
    mapred_report_printer << "Status : FAIL\n";
  report(mapred_report_printer, mapred_report_times);


  stapl::do_once([&](void) {
    std::cerr << add_report_printer.str();
    std::cerr << mapred_report_printer.str();
  });

  return EXIT_SUCCESS;
}
