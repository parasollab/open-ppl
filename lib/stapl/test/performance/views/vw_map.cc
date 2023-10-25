#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/containers/unordered_map/unordered_map.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

#define CONF_INT_REP 32


template <typename T>
struct map_val_wf {
  typedef T result_type;
  template<typename Pair>
  result_type operator() (Pair const &pair) {
    return pair.second;
  }
};

int fibo[] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
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
// FEATURE: map view
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

  stapl::exit_code code1 = experiment(1, model, "map_vw atom", 
                           atom_times, atom_labels);
  stapl::exit_code code2 = experiment(2, model, "map_vw stl", 
                           stl_times, stl_labels);
#ifdef NESTED_ACTIVE
  stapl::exit_code code3 = experiment(3, model, "map_vw nest", 
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
// PLOT: map of atoms
///////////////////////////////////////////////////////////////////////////

typedef int atom_tp;
typedef stapl::map<atom_tp,atom_tp> map_atom_tp;
typedef stapl::unordered_map<atom_tp,atom_tp> unord_map_atom_tp;
typedef stapl::map_view<map_atom_tp> map_atom_vw_tp;
typedef stapl::map_view<unord_map_atom_tp> unord_map_atom_vw_tp;
typedef stapl::indexed_domain<atom_tp> ndx_dom_tp;

typedef stapl::plus<atom_tp> add_atom_wf;

typedef stapl::array<atom_tp> ary_atom_tp;
typedef stapl::array_view<ary_atom_tp> ary_atom_vw_tp;

struct init_map_wf {
  typedef void result_type;
  template<typename Key, typename Val, typename Map>
  result_type operator() (Key key, Val val, Map &map) {
    map[key] = val;
  }
};


bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  // construct key and value containers
  ary_atom_tp key_ct(init_count), val_ct(init_count);
  ary_atom_vw_tp key_vw(key_ct), val_vw(val_ct);

  // initialize key and value containers with sequential loop
  stapl::do_once([&]() {
    for ( size_t i=0; i<init_count; i++ ) {
      key_vw[i] = prime_nums[i];
      val_vw[i] = fibo[i%20];
    }
  });
  stapl::rmi_fence();

  // construct container
  map_atom_tp m_ct(ndx_dom_tp(0,16777216));
  size_t size = outer * inner;

  seconds(time_start);

  // METRIC: construct view over container
  map_atom_vw_tp m_vw(m_ct);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: map");
  seconds(time_start);

  // METRIC: initialize container in parallel
  stapl::map_func( init_map_wf(), 
                   key_vw, val_vw, stapl::make_repeat_view(m_vw) );

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: map");
  seconds(time_start);

  // METRIC: process elements in parallel
  atom_tp sum = stapl::map_reduce( map_val_wf<atom_tp>(),
                                  stapl::plus<atom_tp>(), m_vw );

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: map");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// PLOT: map of STL vectors
///////////////////////////////////////////////////////////////////////////
//
typedef vector<atom_tp> stl_tp;
typedef stapl::map<atom_tp,stl_tp> map_stl_tp;
typedef stapl::unordered_map<atom_tp,stl_tp> unord_map_stl_tp;
typedef stapl::map_view<map_stl_tp> map_stl_vw_tp;
typedef stapl::map_view<unord_map_stl_tp> unord_map_stl_vw_tp;

struct map_sz_wf
{
  typedef size_t result_type;
  template <typename Vec1>
  result_type operator()(Vec1 const v1) {
    return v1.size();
  }
};

typedef stapl::plus<size_t> add_sz_wf;


bool run_stl_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  // construct container
  map_stl_tp m_ct;

  seconds(time_start);

  // METRIC: construct view over container
  map_stl_vw_tp m_vw(m_ct);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: map");
  seconds(time_start);
 
  // METRIC: initialize containers in parallel
  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));
#ifdef FIXME
  stapl::map_func( init_stl_vec_wf(), a_vw, len_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: map");
  seconds(time_start);

  // METRIC: process elements in parallel
#ifdef FIXME
  atom_tp sum = stapl::map_reduce( map_sz_wf(), add_sz_wf(), a_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: map");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// plot: map of STAPL vectors
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<atom_tp> nest_tp;
typedef stapl::map<atom_tp,nest_tp> map_nest_tp;
typedef stapl::unordered_map<atom_tp,nest_tp> unord_map_nest_tp;
typedef stapl::map_view<map_nest_tp> map_nest_vw_tp;
typedef stapl::map_view<unord_map_nest_tp> unord_map_nest_vw_tp;

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
  map_nest_tp m_ct;

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  seconds(time_start);

  // metric: construct view over container
  map_nest_vw_tp m_vw(m_ct);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: map");
  seconds(time_start);

  // metric: initialize containers in parallel
  // CODE

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: map");
  seconds(time_start);

  // metric: process elements in parallel
  // CODE

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: map");

  return true;
}
#endif

