/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/list/list.hpp>
#include "../p_container_profiler.hpp"
#include "../profiler_util.h"
#include "../value_type_util.h"
#include "../list_profiler.hpp"

using namespace stapl;
using std::cout;
using std::endl;

//simple timer
typedef counter<default_timer> counter_type;

size_t N;
std::string caseId;
size_t k;

////////////////////////////////////////////////////////////////////////////////
/// @brief Generates a random number.
///
/// @tparam Val The value type
////////////////////////////////////////////////////////////////////////////////
template <class Val>
struct AGenerator
{
  typedef std::vector<typename boost::remove_const<Val>::type> vtype;
  static Val extract()
  {
    return lrand48() % N;
  }
};

stapl::exit_code stapl_main(int argc,char** argv)
{
  typedef int DTYPE;
  typedef list<DTYPE> LIST;

  if (get_location_id()==0)
    std::cout<<"pContainer Performance Evaluation\n";
  if (argc > 2) {
    N = atol(argv[1]);
    k = atol(argv[2]); //used by list ranking
    srand48(N+get_location_id());
  }
  else {
    std::cout<<"Input size N required; Using 10 by default\n";
    N=10;
  }

  size_t nlocations = get_num_locations();
  size_t block = N / nlocations;
  int locid = get_location_id();

  std::vector<DTYPE> indices(block);
  for (size_t i=0;i<indices.size();++i) {
    indices[i] = lrand48() % N;
  }

  if (locid==0) cout<<"Using N="<<N<<"\n";

  default_constructor_profiler<list<DTYPE> >
    dcp("list", NULL, argc, argv);
  dcp.collect_profile();
  dcp.report();

  constructor_size_profiler<list<DTYPE>, counter_type>
    cep("list", NULL, N, argc, argv);
  cep.collect_profile();
  cep.report();

  LIST* pl_push_back = new LIST(nlocations);
  push_back_profiler_list<list<DTYPE>, counter_type>
    pbp(pl_push_back, "list", indices, argc, argv);
  pbp.collect_profile();
  pbp.report();
  delete pl_push_back;

  list_push_back_profiler<list<DTYPE>, counter_type>
    lpbp("list", indices, argc, argv);
  lpbp.collect_profile();
  lpbp.report();

  LIST* pl_push_front = new LIST();
  push_front_profiler_list<list<DTYPE>, counter_type>
    pfp(pl_push_front, "list", indices, argc, argv);
  pfp.collect_profile();
  pfp.report();
  delete pl_push_front;

  insert_profiler<list<DTYPE>, counter_type>
    ip1("list", "0", indices, argc, argv);
  ip1.collect_profile();
  ip1.report();

  insert_profiler<list<DTYPE>, counter_type>
    ip2("list", "50", indices, argc, argv);
  ip2.collect_profile();
  ip2.report();

  insert_profiler<list<DTYPE>, counter_type>
    ip3("list", "100", indices, argc, argv);
  ip3.collect_profile();
  ip3.report();

  list_insert_profiler<list<DTYPE>, counter_type>
    lip("list", indices, argc, argv);
  lip.collect_profile();
  lip.report();

  list_add_profiler<list<DTYPE>, counter_type>
    pap("list", indices, argc, argv);
  pap.collect_profile();
  pap.report();

  //////////////////////////////////////////////////////////////////////////////
  /// @note The erase profiler cannot currently be used because the function
  /// invalidates the iterator that is passed in rather than return the next
  /// element.
  //////////////////////////////////////////////////////////////////////////////
/*
  erase_profiler<list<DTYPE>, counter_type>
    epb("list", "best", N, argc, argv);
  epb.collect_profile();
  epb.report();

  //////////////////////////////////////////////////////////////////////////////
  /// @note The profilers below are commented out because the following
  /// functions are not yet implemented: split(), splice(), and list_rank().
  //////////////////////////////////////////////////////////////////////////////

  size_t num_splice = 500;
  splice_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice, num_splice * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice = 1000;
  splice_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice, num_splice * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice = 2000;
  splice_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice, num_splice * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice = 3000;
  splice_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice, num_splice * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice = 4000;
  splice_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice, num_splice * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice_range = 10;
  splice_range_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice_range, num_splice_range * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice_range = 20;
  splice_range_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice_range, num_splice_range * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice_range = 40;
  splice_range_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice_range, num_splice_range * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  size_t num_splice_range = 80;
  splice_range_profiler<list<DTYPE>, counter_type>
    sp("list", num_splice_range, num_splice_range * nlocations, argc, argv);
  sp.collect_profile();
  sp.report();

  split_profiler<list<DTYPE>, counter_type>
    stp("list", nlocations, argc, argv);
  stp.collect_profile();
  stp.report();

  LIST* pl_list_ranking =
    new LIST(true, nlocations, k); //k components per location
  //LIST* pl_list_ranking =
  //  new LIST(true, nlocations, k/nlocations); //k components total
  list_ranking_profiler<list<DTYPE>, counter_type>
    lrp(pl_list_ranking, argc, argv);
  lrp.collect_profile();
  lrp.report();
  delete pl_list_ranking;
*/
  return EXIT_SUCCESS;
}
