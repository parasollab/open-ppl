/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/partitioners/graph_partitioners.h>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <time.h>
#include <test/test_report.hpp>

using namespace std;

//Collapser work function needed by the balance collapser
struct balance_collapser_wf
{
private:
  size_t num_partition;
  size_t avg_num_elts_per_partition;

public:
  typedef void result_type;

  balance_collapser_wf(size_t const& num_part, size_t const& num_elt)
   : num_partition(num_part),avg_num_elts_per_partition(num_elt)
  { }

  template<class ELT1, class ELT2, class Partition_vw>
  void operator()(ELT1 elt1, ELT2 elt2, Partition_vw& part_graph_vw)
  {
    size_t elt_partition_id= avg_num_elts_per_partition ?
                             (elt1/avg_num_elts_per_partition) : 0;
    if (elt_partition_id<num_partition)
      part_graph_vw.vp_apply_async(
        elt_partition_id, stapl::add_to_supervertex(elt2.descriptor()));
    else
      part_graph_vw.vp_apply_async(
        num_partition-1, stapl::add_to_supervertex(elt2.descriptor()));
  }

  void define_type(stapl::typer &t)
  {
    t.member(num_partition);
    t.member(avg_num_elts_per_partition);
  }

};

//General balance collapser
struct balance_collapser
{
private:
  size_t num_partition;

public:
  balance_collapser(size_t const& num)
    : num_partition(num)
  { }

  template<class GView>
  stapl::gpartition<GView> operator()(GView const& graph_vw) const
  {
    typedef typename
      stapl::partition_view_type<GView>::type partition_view_type;

    partition_view_type partition_view = stapl::create_partition_view(
                                           graph_vw, num_partition);

    size_t num_elts=graph_vw.num_vertices();

    stapl::map_func(balance_collapser_wf(num_partition, num_elts/num_partition),
                    stapl::counting_view<size_t>(num_elts),
                    graph_vw,stapl::make_repeat_view(partition_view));

    return stapl::gpartition<GView>(partition_view);
  }

  void define_type(stapl::typer &t)
  {
    t.member(num_partition);
  }

};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::dynamic_graph<
            stapl::DIRECTED, stapl::NONMULTIEDGES> pgraph_type;
  typedef pgraph_type::vertex_property             vertex_property;
  typedef stapl::graph_view<pgraph_type>           pgraph_view;
  typedef stapl::gpartition<pgraph_view>           gpartition_t;
  pgraph_type pg;
  pgraph_view vw(pg);

  size_t myid=stapl::get_location_id();
  size_t num_locs=stapl::get_num_locations();

  if (argc!=3)
  {
    cout << "Format: " << argv[0]
         << " number_of_vertices number_of_partitions" << endl;
    return EXIT_FAILURE;
  }
  size_t num_vertices=atoi(argv[1]);
  size_t num_partitions=atoi(argv[2]);

  stapl::array<size_t> random_distribution(num_locs);

  if (myid==0)
  {
    //Initialize random number generator
    srand ( time(NULL) );
    size_t avail_vertices=num_vertices;
    for (stapl::array<size_t>::iterator it = random_distribution.begin();
         it != random_distribution.end(); ++it)
    {
      if (it!=(random_distribution.end()-1))
        *it=rand()%(avail_vertices+1);
      else
        *it=avail_vertices;
      avail_vertices-=*it;
    }
  }
  stapl::rmi_fence();

  size_t max_vertices =random_distribution[myid];

  for (size_t i=0;i<max_vertices+1;++i)
  {
    vw.add_vertex(myid*(num_vertices+1)+i,vertex_property());
  // FIXME when view coarsenig works with no data in some locations, uncomment
  // lines below
  // for (size_t i=0;i<max_vertices;++i){
  //  vw.add_vertex(myid*num_vertices+i,vertex_property());
  }

  stapl::rmi_fence();
  gpartition_t gp=stapl::graph_partition(vw,balance_collapser(num_partitions));
  //Check partition coherency
  size_t res_num_vertices=0;
  for (size_t i=0;i<gp.size();++i)
  {
    res_num_vertices+=gp.vertex_size(i);
  }

  bool empty_intersection=true;
  for (size_t i=0;i<gp.size();++i)
  {
    for (size_t j=i+1;j<gp.size();++j)
    {
      empty_intersection= empty_intersection &&
                          (gp.vertices(i)&gp.vertices(j)).empty();
    }
  }

  //FIXME when view coarsenig works with no data in some locations, uncomment
  //lines below
  //bool res = (gp.size()==num_partitions) && (res_num_vertices==num_vertices)
  //           && empty_intersection;
  bool res = (gp.size()==num_partitions) &&
             (res_num_vertices==(num_vertices+num_locs)) &&
             empty_intersection;

  STAPL_TEST_REPORT(res,"Testing balance_partitioner_example\t");

//        if (myid == 0){
//    cout << "test_graph gpartition_size: " << gp.size() << endl;
//          for (size_t i=0;i<gp.size();++i){
//        cout << "domain " << i << ": " << gp.vertices(i) << endl;
//    }
//        }

  return EXIT_SUCCESS;
}
