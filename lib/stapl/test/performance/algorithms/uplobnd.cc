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
bool run_stl_test( size_t, size_t, double *, string *);

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

  stapl::exit_code code1 = experiment(1, model, "uplobnd_algo atom", 
                                      atom_times, atom_labels);
  stapl::exit_code code2 = experiment(2, model, "uplobnd_algo stl", 
                                      stl_times, stl_labels);
  if( code1==EXIT_SUCCESS && code2==EXIT_SUCCESS ) {
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
    case 2:
      timer.reset();
      timer.start();
      success = run_stl_test(outer_size, inner_size, times, labels);
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
typedef stapl::array<atom_tp> ary_atom_tp;
typedef stapl::array_view<ary_atom_tp> ary_atom_vw_tp;

typedef stapl::plus<atom_tp> add_atom_wf;
typedef stapl::less<atom_tp> less_atom_wf;

struct init_ary_wf {
  typedef void result_type;
  template<typename Val, typename Ndx, typename Ary>
  result_type operator() (Val val, Ndx ndx, Ary &ary) {
    ary[ndx] = val;
  }
};


bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  size_t size = outer * inner;
  // construct container and view over it
  vec_atom_tp v_ct(size), w_ct(size), x_ct(size), y_ct(size);
  vec_atom_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  ary_atom_tp seq_ct(size);
  ary_atom_vw_tp seq_vw(seq_ct);
  // initialize container with sequential loop
  stapl::do_once([&]() {
    for( size_t i=0; i<init_count; i++ ) {
      seq_vw[i] = rand_nums[i];
    }
  });
  stapl::rmi_fence();
  stapl:: map_func( init_ary_wf(), 
                    seq_vw, stapl::counting_view<int>(size),
                    stapl::make_repeat_view(v_vw) );
  stapl:: map_func( init_ary_wf(), 
                    seq_vw, stapl::counting_view<int>(size),
                    stapl::make_repeat_view(w_vw) );
  stapl:: map_func( init_ary_wf(), 
                    seq_vw, stapl::counting_view<int>(size),
                    stapl::make_repeat_view(x_vw) );
  stapl:: map_func( init_ary_wf(), 
                    seq_vw, stapl::counting_view<int>(size),
                    stapl::make_repeat_view(y_vw) );

  seconds(time_start);
  atom_tp val = v_vw[v_vw.size()*.9];

  // METRIC: apply algorithm
  int v_ndx = -1;
  auto v_ref = stapl::lower_bound( v_vw, val, less_atom_wf() );
  if ( !stapl::is_null_reference(v_ref) ) {
    v_ndx = stapl::index_of(v_ref);
  }

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("algorithm: lower_bound, user predicate");
  seconds(time_start);

  // METRIC: apply algorithm
  int w_ndx = -1;
  auto w_ref = stapl::lower_bound( w_vw, val );
  if ( !stapl::is_null_reference(w_ref) ) {
    w_ndx = stapl::index_of(w_ref);
  }

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("algorithm: lower_bound");
  seconds(time_start);

  // METRIC: apply algorithm
  int x_ndx = -1;
  auto x_ref = stapl::upper_bound( x_vw, val, less_atom_wf() );
  if ( !stapl::is_null_reference(x_ref) ) {
    x_ndx = stapl::index_of(x_ref);
  }

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("algorithm: upper_bound, user predicate");
  seconds(time_start);

  // METRIC: apply algorithm
  int y_ndx = -1;
  auto y_ref = stapl::upper_bound( y_vw, val );
  if ( !stapl::is_null_reference(y_ref) ) {
    y_ndx = stapl::index_of(y_ref);
  }

  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("algorithm: upper_bound");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// PLOT: array of STL vectors
///////////////////////////////////////////////////////////////////////////

typedef vector<atom_tp> stl_tp;
typedef stapl::array<stl_tp> ary_stl_tp;
typedef stapl::array_view<ary_stl_tp> ary_stl_vw_tp;


bool run_stl_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  // construct container and view over it
  ary_stl_tp v_ct(outer), w_ct(outer), x_ct(outer), y_ct(outer);
  ary_stl_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  // initialize containers in parallel
  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( init_rand_vec_wf(), v_vw, len_vw );
  stapl::map_func( init_rand_vec_wf(), w_vw, len_vw );
  stapl::map_func( init_rand_vec_wf(), x_vw, len_vw );
  stapl::map_func( init_rand_vec_wf(), y_vw, len_vw );
  stl_tp val = v_vw[v_vw.size()*.9];

  seconds(time_start);
 
  // METRIC: apply algorithm
  int v_ndx = -1;
  auto v_ref = stapl::lower_bound( v_vw, val, less_stl_pred<stl_tp>() );
  if ( !stapl::is_null_reference(v_ref) ) {
    v_ndx = stapl::index_of(v_ref);
  }

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("algorithm: lower_bound, user predicate");
  seconds(time_start);

  // METRIC: apply algorithm
  int x_ndx = -1;
  auto x_ref = stapl::upper_bound( x_vw, val, less_stl_pred<stl_tp>() );
  if ( !stapl::is_null_reference(x_ref) ) {
    x_ndx = stapl::index_of(x_ref);
  }

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("algorithm: upper_bound, user predicate");

  return true;
}
