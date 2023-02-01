// cont/dygraf.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

#define CONF_INT_REP 32

int prime[] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
               31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

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
// FEATURE: graph view
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

  stapl::exit_code code1 = experiment(1, model, "graph_vw atom", 
                           atom_times, atom_labels);
  stapl::exit_code code2 = experiment(2, model, "graph_vw stl", 
                           stl_times, stl_labels);
#ifdef NESTED_ACTIVE
  stapl::exit_code code3 = experiment(3, model, "graph_vw nest", 
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
// PLOT: graph of atoms
///////////////////////////////////////////////////////////////////////////

typedef int atom_tp;
typedef stapl::dynamic_graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
                             atom_tp, atom_tp> dygraf_int_tp;
typedef stapl::graph_view<dygraf_int_tp> dygraf_int_vw_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                             atom_tp, atom_tp> stgraf_int_tp;
typedef stapl::graph_view<stgraf_int_tp> stgraf_int_vw_tp;

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

struct init_prop_wf {
  typedef void result_type;
  template <typename Vertex>
  result_type operator()(Vertex vtx) {
    int i = 0;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prime[i++];
    }
  }
};
 
struct add_vert_wf {
  typedef void result_type;
  template <typename Property, typename Graph>
  result_type operator()(Property prop, Graph graf) {
    graf.add_vertex(prop);
  }
};
 
struct add_edge_wf {
  typedef void result_type;
  template <typename Vertex, typename Count, typename Graph>
  result_type operator()(Vertex vtx, Count cnt, Graph graf) {
    size_t i = 0;
    size_t src = 0;
    for ( auto vtx_it = vtx.begin(); vtx_it != vtx.end(); ++vtx_it ) {
      for ( size_t j = 0; j < cnt/2; j++ ) {
        size_t dest = 0;
        do {
          dest = rand() % cnt;
        } while ( src != dest );
        auto ed = graf.add_edge( src, dest, prime[i%20]/2 );
        if (numeric_limits<size_t>::max() == ed.id() ) { // FAILED?
        }
        i++;
      }
      src++;
    }
  }
};

struct update_prop_wf {
  typedef void result_type;
  template <typename Vertex, typename Property>
  result_type operator()(Vertex vtx, Property prop) {
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prop * 100;
    }
  }
};

typedef stapl::plus<atom_tp> add_atom_wf;


bool run_atom_test(size_t outer, size_t inner, double *times, string *labels ) {

  double time_start, time_end, time_delta;

  size_t size = outer * inner;
  // construct container
  dygraf_int_tp dg_ct(size);

  seconds(time_start);

  // METRIC: construct view over container
  dygraf_int_vw_tp dg0_vw(dg_ct);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: graph, construct view");
  seconds(time_start);

  // METRIC: initialize vertex properties in parallel
  stapl::map_func( init_prop_wf(),
                   dg0_vw );
 
  size_t count = dg0_vw.num_vertices();

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: graph, initialize vertex properties");
  seconds(time_start);

  // METRIC: add graph vertices to dynamic graph in parallel
  vec_int_tp prop_vec(10);
  vec_int_vw_tp prop_vec_vw(prop_vec);
  stapl::iota(prop_vec_vw, 100 );
  stapl::map_func( add_vert_wf(),
                   prop_vec_vw,
                   stapl::make_repeat_view<dygraf_int_vw_tp>(dg0_vw) );
  
  dygraf_int_vw_tp dg_vw(dg_ct);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: graph, add vertices");
  seconds(time_start);
 
  // METRIC: add edges in parallel
  stapl::map_func( add_edge_wf(),
                   dg_vw, stapl::make_repeat_view(count),
                   stapl::make_repeat_view<dygraf_int_vw_tp>(dg_vw) );

  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("view: graph, add edges");
  seconds(time_start);

  // METRIC: process graph elements in parallel
  count = dg_vw.num_vertices();
  stapl::map_func( update_prop_wf(),
                   dg_vw, stapl::counting_view<int>(count) );

  seconds(time_end);
  time_delta = time_end - time_start;
  times[4] += time_delta;
  labels[4] = string("view: graph, process elements");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// PLOT: graph of STL vectors
///////////////////////////////////////////////////////////////////////////

typedef vector<atom_tp> stl_tp;

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
#ifdef FINISH
  // CODE
#endif

  seconds(time_start);

  // METRIC: construct view over container
#ifdef FINISH
  // CODE
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: graph, construct view");
  seconds(time_start);
 
  // METRIC: initialize containers in parallel
#ifdef FINISH
  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( init_stl_vec_wf(), a_vw, len_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: graph, initialize properties");
  seconds(time_start);

  // METRIC: process elements in parallel
#ifdef FINISH
  atom_tp sum = stapl::map_reduce( map_sz_wf(), add_sz_wf(), a_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: graph, process elements");
  seconds(time_start);

  return true;
}


///////////////////////////////////////////////////////////////////////////
// plot: graph of STAPL vectors
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<atom_tp> nest_tp;

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

  size_t inner = 100 * (rand() % 10);
  size_t outer = pages * rows * cols;
  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  seconds(time_start);

  // metric: construct view over container

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: graph");
  seconds(time_start);

  // metric: initialize containers in parallel
  // CODE

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: graph");
  seconds(time_start);

  // metric: process elements in parallel
  // CODE

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: graph");
  seconds(time_start);

  return true;
}
#endif
