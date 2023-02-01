#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/views/strided_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
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
bool opt_verbose = false;

///////////////////////////////////////////////////////////////////////////
// FEATURE: strided view
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

  stapl::exit_code code1 = experiment(1, model, "strided_vw atom", 
                           atom_times, atom_labels);
  stapl::exit_code code2 = experiment(2, model, "strided_vw stl", 
                           stl_times, stl_labels);
#ifdef NESTED_ACTIVE
  stapl::exit_code code3 = experiment(3, model, "strided_vw nest", 
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
// PLOT: strided view over container of atoms
///////////////////////////////////////////////////////////////////////////

typedef int atom_tp;
typedef stapl::array<int> ary_atom_tp;
typedef stapl::array_view<ary_atom_tp> ary_atom_vw_tp;
typedef stapl::strided_view<ary_atom_vw_tp>::type str_ary_atom_vw_tp;

struct add_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z) {
    z = x + y;
  }
};

typedef stapl::plus<atom_tp> add_atom_wf;


bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  size_t size = outer * inner;
  // construct container
  ary_atom_tp a_ct(size), b_ct(size*2), c_ct(size*4);

  seconds(time_start);
  // construct view over container
  ary_atom_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  // METRIC: construct view 
  int stride = 2;
  str_ary_atom_vw_tp a_str_vw = stapl::make_strided_view(a_vw, stride, 0);
  str_ary_atom_vw_tp b_str_vw = stapl::make_strided_view(b_vw, stride, 0);
  stride = 4;
  str_ary_atom_vw_tp c_str_vw = stapl::make_strided_view(c_vw, stride, 0);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: strided, construct view");

  // METRIC: initialize containers in parallel
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  int rep = 49;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vw, repeat_wf(base,rep));

  seconds(time_start);
  // METRIC: process elements through view
  stapl::map_func(add_wf(), a_vw, b_str_vw, c_str_vw);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: strided, process elements");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// PLOT: strided view over container of STL vectors
///////////////////////////////////////////////////////////////////////////

typedef vector<atom_tp> stl_tp;
typedef stapl::array<stl_tp> ary_stl_tp;
typedef stapl::array_view<ary_stl_tp> ary_stl_vw_tp;
typedef stapl::strided_view<ary_stl_vw_tp>::type str_ary_stl_vw_tp;


bool run_stl_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  // construct container
  ary_stl_tp a_ct(outer), b_ct(outer*2), c_ct(outer*4);

  seconds(time_start);

  // construct view over container
  ary_stl_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  seconds(time_start);
  // METRIC: construct view
  int stride = 2;
  str_ary_stl_vw_tp a_str_vw = stapl::make_strided_view(a_vw, stride, 0);
  str_ary_stl_vw_tp b_str_vw = stapl::make_strided_view(b_vw, stride, 0);
  stride = 4;
  str_ary_stl_vw_tp c_str_vw = stapl::make_strided_view(c_vw, stride, 0);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: strided, construct view");
  seconds(time_start);
 
  // initialize containers in parallel
  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

#if 0
  stapl::map_func( init_stl_vec_wf(), a_vw, len_vw );
  stapl::map_func( init_stl_vec_wf(), b_vw, len_vw );
  stapl::map_func( init_stl_vec_wf(), c_vw, len_vw );
#endif

  seconds(time_start);
#if 0
  // METRIC: process elements in parallel
  auto neqtest = stapl::map_reduce( neq_stl_pred<stl_tp>(),
                                stapl::logical_or<bool>(),
                                a_str_vw, b_str_vw );
  auto eqtest  = stapl::map_reduce( eq_stl_pred<stl_tp>(),
                                stapl::logical_and<bool>(),
                                b_str_vw, c_str_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: strided");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// plot: strided view over container of STAPL vectors
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<atom_tp> nest_tp;
typedef stapl::array<nest_tp> ary_nest_tp;
typedef stapl::array_view<ary_nest_tp> ary_nest_vw_tp;
typedef stapl::strided_view<ary_nest_vw_tp>::type str_ary_nest_vw_tp;

#ifdef NESTED_ACTIVE
struct fill_wf
{
  typedef void result_type;
  template <typename Elem, typename View>
  result_type operator()(View const &vw, Elem count)
  {
    // CODE
  }
};

struct max_stapl_inner_wf
{
  typedef void result_type;
  template <typename Elem1, typename Elem2, typename Elem3>
  result_type operator()(Elem1 v1, Elem2 v2, Elem3 &v3) {
    // CODE
  }
};

struct max_stapl_outer_wf
{
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const v1, View2 const v2, View3 const &v3) {
    stapl::map_func(max_stapl_inner_wf(), v1, v2, v3 );
  }
};


bool run_nest_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  // construct container

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  seconds(time_start);

  // metric: construct view over container
  // CODE 

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: strided, construct viwe");
  seconds(time_start);

  // initialize containers in parallel
  // CODE 

  seconds(time_start);
  // metric: process elements in parallel
  // CODE 

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: strided, process elements");

  return true;
}
#endif

