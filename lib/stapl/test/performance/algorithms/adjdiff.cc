#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
 
#include "algohelp.hpp"
using namespace std;

#define CONF_INT_REP 32

stapl::exit_code experiment(int, int, const char *, double *, string *);
bool run_atom_test( size_t, size_t, double *, string *);

struct timeval tp;

char *opt_data = 0;
bool opt_list = false;
bool opt_quiet = false;
int  opt_test = -1;
bool opt_verbose = false;

///////////////////////////////////////////////////////////////////////////
// FEATURE: array view 
///////////////////////////////////////////////////////////////////////////

stapl::exit_code stapl_main(int argc, char **argv) {

  double atom_times[6] = {0,0,0,0,0,0};
  double stl_times[6] =  {0,0,0,0,0,0};

  string atom_labels[6] = {"", "", "", "", "", ""};
  string stl_labels[6] =  {"", "", "", "", "", ""};

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

  stapl::exit_code code1 = experiment(1, model, "adjdiff_algo atom", 
                                      atom_times, atom_labels);
  if( code1==EXIT_SUCCESS ) {
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
  confidence_interval_controller iter_control(CONF_INT_REP, 100, 0.05); // (32
  while (continue_iterating && success) {
    repeat++;
    switch(test) {
    case 1:
      timer.reset();
      timer.start();
      success = run_atom_test(outer_size, inner_size, times, labels);
      iter_control.push_back(timer.stop());
      break;
    }

    stapl::array<int> continue_ct(stapl::get_num_locations());
    stapl::array_view<stapl::array<int>> continue_vw(continue_ct);
    continue_vw[stapl::get_location_id()] = iter_control.iterate() ? 1 : 0;

    int iterate_sum = stapl::accumulate(continue_vw, (int) 0);

    continue_iterating = (iterate_sum != 0 ? true : false);
  }

  if( !opt_quiet ) {
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
typedef stapl::vector<atom_tp> vec_atom_tp;
typedef stapl::vector_view<vec_atom_tp> vec_atom_vw_tp;

typedef stapl::minus<atom_tp> sub_atom_wf;

bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  size_t size = outer * inner;
  // construct container and view over it
  vec_atom_tp v_ct(size), w_ct(size), x_ct(size), y_ct(size);
  vec_atom_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<atom_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));

  seconds(time_start);

  // METRIC: apply algorithm
  stapl::adjacent_difference(v_vw, x_vw, sub_atom_wf());

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("algorithm, adjacent_difference, user function");
  seconds(time_start);

  // METRIC: apply algorithm
  stapl::adjacent_difference(w_vw, y_vw);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("algorithm, adjacent_difference, default function");

  return true;
}
