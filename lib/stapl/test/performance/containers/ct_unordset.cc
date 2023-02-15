#include <stapl/domains/indexed.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/containers/set/set.hpp>
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
// FEATURE: unordered set
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

  stapl::exit_code code1 = experiment(1, model, "unordset_ct atom", 
                           atom_times, atom_labels);
#ifdef SUPPORTED_BY_STAPL
  stapl::exit_code code2 = experiment(2, model, "unordset_ct stl", 
                           stl_times, stl_labels);
#ifdef NESTED_ACTIVE
  stapl::exit_code code3 = experiment(3, model, "unordset_ct nest", 
                           nest_times, nest_labels);
  if ( code1==EXIT_SUCCESS && code2==EXIT_SUCCESS && code3==EXIT_SUCCESS ) {
#else
  if ( code1==EXIT_SUCCESS && code2==EXIT_SUCCESS ) {
#endif
#else
  if ( code1==EXIT_SUCCESS ) {
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
#ifdef SUPPORTED_BY_STAPL
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
// PLOT: unordered set of atoms
///////////////////////////////////////////////////////////////////////////

typedef int atom_tp;
typedef stapl::set<atom_tp> set_atom_tp;
typedef stapl::set<atom_tp, stapl::equal_to<atom_tp>,
        stapl::view_based_partition<dist_spec_tp>,
        stapl::view_based_mapper<dist_spec_tp> > set_atom_dist_tp;
typedef stapl::indexed_domain<atom_tp> ndx_dom_tp;


bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // METRIC: construct container
  set_atom_tp s_ct;

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: unordered set, construct, defaults");
  seconds(time_start);

  // METRIC: construct container with non-default distribution
#ifdef STAPL_OR_TEST_BUG
  dist_spec_tp old_dist = stapl::block_cyclic(outer,10);
  set_atom_dist_tp t_ct(old_dist);
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: unordered set, construct, non-default distribution");
  seconds(time_start);

  // METRIC: initialize container using sequential loop
  for ( int i=0; i<outer; i++ ) {
    s_ct.insert(fibo[i%20]);
#ifdef STAPL_OR_TEST_BUG
    t_ct.insert(fibo[i%20]/2);
#endif
  }
  for ( int i=0; i<outer; i++ ) {
    atom_tp elem = prime_nums[i%init_count];
    s_ct.insert(elem);
#ifdef STAPL_OR_TEST_BUG
    t_ct.insert(i+prime_nums[i%init_count]/2);
#endif
  }
  stapl::rmi_fence();

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: unordered set, initialize");
  seconds(time_start);

  // METRIC: find an element
  atom_tp key = prime_nums[init_count/2];
  auto it = s_ct.find(key);
  if ( it != s_ct.end() ) {
    // found
  }
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("container: unordered set, find element");

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}

///////////////////////////////////////////////////////////////////////////
// plot: unordered set of STL vectors
///////////////////////////////////////////////////////////////////////////

typedef vector<atom_tp> stl_tp;
typedef stapl::set<stl_tp> set_stl_tp;
typedef stapl::set<stl_tp, stapl::equal_to<stl_tp>,
        stapl::view_based_partition<dist_spec_tp>,
        stapl::view_based_mapper<dist_spec_tp> > set_stl_dist_tp;


#ifdef SUPPORTED_BY_STAPL
bool run_stl_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // metric: construct container
  set_stl_tp s_ct;

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: unordered set, construct, defaults");
  seconds(time_start);

  // metric: construct container with non-default distribution
  // CODE

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: unordered set, construct, non-default distribution");
  seconds(time_start);

  // metric: initialize container using sequential loop
  // CODE
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: unordered set, initialize");
  seconds(time_start);

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}
#endif

///////////////////////////////////////////////////////////////////////////
// plot: unordered set of STAPL vectors
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<int> nest_tp;
typedef stapl::set<nest_tp> set_nest_tp;
typedef stapl::set<nest_tp, stapl::equal_to<nest_tp>,
        stapl::view_based_partition<dist_spec_tp>,
        stapl::view_based_mapper<dist_spec_tp> > set_nest_dist_tp;

#ifdef SUPPORTED_BY_STAPL
bool run_nest_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // metric: construct container
  set_nest_tp s_ct;

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: unordered set, construct, defaults");
  seconds(time_start);

  // metric: construct container with non-default distribution
  // CODE

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: unordered set, construct, non-default distribution");
  seconds(time_start);

  // metric: initialize container using sequential loop
  // CODE
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: unordered set, initialize");

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}
#endif

