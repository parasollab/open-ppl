
#include <stapl/containers/vector/vector.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "conthelp.hpp"
using namespace std;

#define CONF_INT_REP 32

stapl::exit_code experiment(int, int, const char *, double *, string *);
bool run_atom_test( size_t, size_t, double *, string *);
bool run_stl_test( size_t, size_t, double *, string *);
bool run_nest_test( size_t, size_t, double *, string *);

struct timeval tp;

char *opt_data = 0;
bool opt_list = false;
bool opt_quiet = false;
int  opt_test = -1;
bool opt_verbose = true;

///////////////////////////////////////////////////////////////////////////
// FEATURE: array container
///////////////////////////////////////////////////////////////////////////

stapl::exit_code stapl_main(int argc, char **argv) {

  double atom_times[6] = {0,0,0,0,0,0};
  double stl_times[6] =  {0,0,0,0,0,0};
  double nest_times[6] = {0,0,0,0,0,0};

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

  stapl::exit_code code1 = experiment(1, model, "array_ct atom", 
                           atom_times, atom_labels);
  stapl::exit_code code2 = experiment(2, model, "array_ct stl", 
                           stl_times, stl_labels);
#ifdef NESTED_ACTIVE
  stapl::exit_code code3 = experiment(3, model, "array_ct nest", 
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
  size_t inner_size = 100;

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
// PLOT: array of atoms
///////////////////////////////////////////////////////////////////////////

typedef int atom_tp;
typedef stapl::array<atom_tp> ary_atom_tp;
typedef stapl::array<atom_tp,
        stapl::view_based_partition<dist_spec_tp>,
        stapl::view_based_mapper<dist_spec_tp> > ary_atom_dist_tp;

bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  size_t size = outer * inner;
  // METRIC: construct container
  ary_atom_tp a_ct(size);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: array, construct, defaults");
  seconds(time_start);

  // METRIC: construct container with non-default initializer
  ary_atom_tp b_ct(size, 314159);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: array, construct, non-default initializer");
  seconds(time_start);

  // METRIC: construct container with non-default distribution
  dist_spec_tp old_dist = stapl::block(size,1000);
  ary_atom_dist_tp c_ct(old_dist);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: array, construct, non-default distribution");
  seconds(time_start);

  // METRIC: redistribute container
  dist_spec_tp new_dist = stapl::block_cyclic(size,size/100);
  c_ct.redistribute(new_dist);
 
  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("container: array, redistribute");

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}

///////////////////////////////////////////////////////////////////////////
// PLOT: array of STL vectors
///////////////////////////////////////////////////////////////////////////

typedef vector<atom_tp> stl_tp;
typedef stapl::array<stl_tp> ary_stl_tp;
typedef stapl::array<stl_tp,
        stapl::view_based_partition<dist_spec_tp>,
        stapl::view_based_mapper<dist_spec_tp> > ary_stl_dist_tp;

bool run_stl_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // METRIC: construct container
  ary_stl_tp a_ct(outer);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: array, construct, defaults");
  seconds(time_start);

  // METRIC: construct container with non-default initializer
  ary_stl_tp b_ct(outer, vector<atom_tp>(inner,314159) );

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: array, construct, non-default initializer");
  seconds(time_start);

  // METRIC: construct container with non-default distribution
  dist_spec_tp old_dist = stapl::block(outer,1000);
  ary_stl_dist_tp c_ct(old_dist);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: array, construct, non-default distribution");
  seconds(time_start);

#ifdef REDISTRIB_TERMINATES
  seconds(time_start);

  // METRIC: redistribute container
  dist_spec_tp new_dist = stapl::block_cyclic(outer,100);
  c_ct.redistribute(new_dist);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("container: array, redistribute");
#endif

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}

///////////////////////////////////////////////////////////////////////////
// plot: array of STAPL vectors
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<int> nest_tp;
typedef stapl::array<nest_tp> ary_nest_tp;
typedef stapl::array<nest_tp,
        stapl::view_based_partition<dist_spec_tp>,
        stapl::view_based_mapper<dist_spec_tp> > ary_nest_dist_tp;

#ifdef NESTED_ACTIVE
bool run_nest_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;
  seconds(time_start);

  // metric: construct container
  ary_nest_tp a_ct(outer);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("container: array, construct");
  seconds(time_start);

  // metric: construct container with non-default distribution
  dist_spec_tp old_dist = stapl::block(outer,10);
  ary_nest_dist_tp b_ct(old_dist);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("container: array, construct, non-default distribution");
  seconds(time_start);

  // metric: redistribute container
  dist_spec_tp new_dist = stapl::block_cyclic(outer,100);
  // CODE 

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("container: array, redistribute");

  // for p_objects on the stack
  stapl::rmi_fence();
  return true;
}
#endif

