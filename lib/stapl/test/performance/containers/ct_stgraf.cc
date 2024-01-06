#include <stapl/containers/vector/vector.hpp>
#include <stapl/containers/array/array.hpp>

#include <cstdlib>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "conthelp.hpp"
using namespace std;

#define CONF_INT_REP 32

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };

stapl::exit_code experiment(int, int, const char *, double *, string *);
bool run_atom_test( size_t, size_t, double *, string *);
bool run_stl_test( size_t, size_t, double *, string *);
bool run_nest_test( size_t, size_t, double *, string *);

struct timeval tp;

char *opt_data = 0;
bool opt_list = false;
bool opt_quiet = false;
int  opt_test = -1;
bool opt_verbose = false;

///////////////////////////////////////////////////////////////////////////
// FEATURE: static graph container
///////////////////////////////////////////////////////////////////////////

stapl::exit_code stapl_main(int argc, char **argv) {

  double atom_times[6] = {0,0,0,0,0,0};
  double stl_times[6] =  {0,0,0,0,0.0};
  double nest_times[6] = {0,0,0,0,0.0};

  string atom_labels[6] = {"", "", "", "", "", ""};
  string stl_labels[6] =  {"", "", "", "", "", ""};
  string nest_labels[6] = {"", "", "", "", "", ""};

  for ( int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ('-' == opt[0] ) {
      switch ( opt[1] ) {
      case 'h': cerr << "HELP\n";              break;
      case 'd': opt_data = argv[argi++];       break;
      case 'l': opt_list = true;               break;
      case 'q': opt_quiet = true;              break;
      case 't': opt_test = atoi(argv[argi++]); break;
      case 'v': opt_verbose = true;            break;
      }
    } else {
      cerr << "unknown command line argument " << opt << endl;
    }
  }

  int model = -1;
  switch ( opt_data[0] ) {
  case 't': model = 1;         break;
  case 's': model = 100;       break;
  case 'm': model = 10000;     break;
  case 'b': model = 1000000;   break;
  case 'h': model = 100000000; break;
  default:
    cerr << "opt_data " << opt_data << endl;
    break;
  }
  if (model == -1) {
    std::cerr << "usage: exe -data tiny/small/medium/big/huge\n";
    exit(1);
  }

  stapl::exit_code code1 = experiment(1, model, "stgraf_ct atom", 
                           atom_times, atom_labels);
  stapl::exit_code code2 = experiment(2, model, "stgraf_ct stl", 
                           stl_times, stl_labels);
#ifdef NESTED_ACTIVE
  stapl::exit_code code3 = experiment(3, model, "stgraf_ct nest", 
                           nest_times, nest_labels);
  if ( code1==EXIT_SUCCESS && code2==EXIT_SUCCESS && code3==EXIT_SUCCESS ) {
#else
  if ( code1==EXIT_SUCCESS && code2==EXIT_SUCCESS ) {
#endif
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}

///////////////////////////////////////////////////////////////////////////
// execute test enough times to achieve desired confidence interval
///////////////////////////////////////////////////////////////////////////


stapl::exit_code experiment(int test, int model, const char *test_name, 
                            double *times, string *labels)
{
  counter_t timer;
  bool success = true;
  size_t total_size = 1000 * model;
  size_t outer_size = total_size / 100;
  size_t inner_size = total_size / outer_size;

  bool continue_iterating = true;
  int repeat = 0;
  confidence_interval_controller iter_control(CONF_INT_REP, 100, 0.05);
  while (continue_iterating && success) {
    repeat++;
    switch(test) {
    case 1:
      timer.reset();
      timer.start();
      success = run_atom_test(outer_size, inner_size, times, labels);
      iter_control.push_back(timer.stop());
      break;
    case 2:
      timer.reset();
      timer.start();
      success = run_stl_test(outer_size, inner_size, times, labels);
      iter_control.push_back(timer.stop());
      break;
#ifdef NESTED_ACTIVE
    case 3:
      timer.reset();
      timer.start();
      success = run_nest_test(outer_size, inner_size, times, labels);
      iter_control.push_back(timer.stop());
      break;
#endif
    }

    stapl::array<int> continue_ct(stapl::get_num_locations());
    stapl::array_view<stapl::array<int>> continue_vw(continue_ct);
    continue_vw[stapl::get_location_id()] = iter_control.iterate() ? 1 : 0;

    int iterate_sum = stapl::accumulate(continue_vw, (int) 0);

    continue_iterating = (iterate_sum != 0 ? true : false);
  }

  if ( !opt_quiet ) {
    iter_control.report(test_name);
  }
  if ( opt_verbose ) {
    stapl::do_once([&]() {
      int i=0;
      while( i<6 && labels[i].size() > 0 ) {
        double avg = times[i] / repeat;
        cout << labels[i] << " : " << avg << endl;
        i++;
      }
    });
  }

  if ( success ) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}

///////////////////////////////////////////////////////////////////////////
// PLOT: static graph with atom properties
///////////////////////////////////////////////////////////////////////////

typedef int atom_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               atom_tp, atom_tp>
               stgraf_atom_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               atom_tp, atom_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> >
               stgraf_atom_dist_tp;


bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // METRIC: construct container
  srand(16807);
  stgraf_atom_tp g_ct(outer);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: static graph, construct, defaults");
  seconds(time_start);

  // METRIC: construct container with non-default distribution
  dist_spec_tp old_dist = stapl::block(outer,10);
  stgraf_atom_dist_tp h_ct(old_dist);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: static graph, construct, non-default distribution");
  seconds(time_start);

  // METRIC: add edges to vertices with sequential loop
  size_t vert_cnt = g_ct.num_vertices();
  size_t k = 0;
  size_t src = 0;
  for ( auto vtx_it = g_ct.begin(); vtx_it != g_ct.end(); ++vtx_it ) {
    for ( size_t j = 0; j < vert_cnt/2; j++ ) {
      size_t dest = 0;
      do {
        dest = rand() % vert_cnt;
      } while ( src != dest );
      auto ed = g_ct.add_edge( src, dest, prime_nums[k%init_count]/2 );
      if (numeric_limits<size_t>::max() == ed.id() ) { // FAILED
      }
      k++;
    }
    src++;
  }
  stapl::rmi_fence();

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: static graph, add edges");
  seconds(time_start);

  // METRIC: redistribute container
  dist_spec_tp new_dist = stapl::block_cyclic(outer,2);
  h_ct.redistribute(new_dist);
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("container: static graph, redistribute");

#if 0
  // METRIC: initialize vertex properties with sequential loop
  size_t i=0;
  for ( auto vtx_it = g_ct.begin(); vtx_it != g_ct.end(); ++vtx_it ) {
    auto vtx = *vtx_it;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prime_nums[i%init_count];
      i++;
    }
  }
  size_t j=0;
  for ( auto vtx_it = h_ct.begin(); vtx_it != h_ct.end(); ++vtx_it ) {
    auto vtx = *vtx_it;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prime_nums[j%init_count]/2;
      j++;
    }
  }
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[4] += time_delta;
  labels[4] = string("container: static graph, initialize properties");
  seconds(time_start);
#endif

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}

///////////////////////////////////////////////////////////////////////////
// PLOT: static graph with STL vector properties
///////////////////////////////////////////////////////////////////////////

typedef vector<atom_tp> stl_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               stl_tp, stl_tp>
               stgraf_stl_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               stl_tp, stl_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> >
               stgraf_stl_dist_tp;


bool run_stl_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // METRIC: construct container
  srand(16807);
  stgraf_stl_tp g_ct(outer);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: static graph, construct, defaults");
  seconds(time_start);

  // METRIC: construct container with non-default distribution
  dist_spec_tp old_dist = stapl::block(outer,10);
  stgraf_stl_dist_tp h_ct(old_dist);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: static graph, construct, non-default distribution");
  seconds(time_start);

  size_t base = inner / 2;

#if 0
  // METRIC: initialize vertex properties with sequential loop
  size_t i=0;
  for ( auto vtx_it = g_ct.begin(); vtx_it != g_ct.end(); ++vtx_it ) {
    auto vtx = *vtx_it;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      size_t size = base + (rand() % inner );
      (*edge_it).property() = stl_tp(size, prime_nums[i%init_count]);
      i++;
    }
  }
  size_t j=0;
  for ( auto vtx_it = h_ct.begin(); vtx_it != h_ct.end(); ++vtx_it ) {
    auto vtx = *vtx_it;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      size_t size = base + (rand() % inner );
      (*edge_it).property() = stl_tp(size, prime_nums[j%init_count]);
      j++;
    }
  }
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: static graph, initialize properties");
#endif

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}

///////////////////////////////////////////////////////////////////////////
// plot: static graph with STAPL vector properties
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<int> nest_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               nest_tp, nest_tp>
               stgraf_nest_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               nest_tp, nest_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> >
               stgraf_nest_dist_tp;


#ifdef NESTED_ACTIVE
bool run_nest_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // metric: construct container
  srand(16807);
  stgraf_nest_tp g_ct(outer);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: static graph, construct, defaults");
  seconds(time_start);

  // metric: construct container with non-default distribution
  // CODE

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: static graph, construct, non-default distribution");
  seconds(time_start);

  // metric: initialize container using sequential loop
  // CODE
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: static graph, initialize properties");

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}
#endif

