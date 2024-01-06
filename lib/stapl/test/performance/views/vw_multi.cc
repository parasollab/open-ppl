#include <stapl/containers/vector/vector.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/multiarray/traversals.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

#define CONF_INT_REP 32

typedef stapl::indexed_domain<size_t>                        vec_dom_tp;
typedef stapl::balanced_partition<vec_dom_tp>                bal_part_tp;
typedef stapl::tuple<size_t, size_t, size_t>                 gid_tp;

typedef stapl::default_traversal<3>::type                    trav3_yz_tp;
typedef stapl::xyplane_major                                 trav3_xy_tp;
typedef stapl::xzplane_major                                 trav3_xz_tp;

typedef stapl::nd_partition<
          stapl::tuple<bal_part_tp, bal_part_tp, bal_part_tp>,
          trav3_yz_tp>                                       part3_tp;
typedef stapl::nd_partition<
          stapl::tuple<bal_part_tp, bal_part_tp, bal_part_tp>,
          trav3_yz_tp>                                       part3_yz_tp;
typedef stapl::nd_partition<
          stapl::tuple<bal_part_tp, bal_part_tp, bal_part_tp>,
          trav3_xy_tp>                                       part3_xy_tp;
typedef stapl::nd_partition<
          stapl::tuple<bal_part_tp, bal_part_tp, bal_part_tp>,
          trav3_xz_tp>                                       part3_xz_tp;


stapl::exit_code experiment(int, int, const char *, double *, string *);
bool run_atom_test( size_t, size_t, size_t, double *, string *);
bool run_stl_test( size_t, size_t, size_t, double *, string *);
bool run_nest_test( size_t, size_t, size_t, double *, string *);

struct timeval tp;

char *opt_data = 0;
bool opt_list = false;
bool opt_quiet = false;
int  opt_test = -1;
bool opt_verbose = false;

///////////////////////////////////////////////////////////////////////////
// FEATURE: multi array view
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

  stapl::exit_code code1 = experiment(1, model, "multi_vw atom",
                           atom_times, atom_labels);
  stapl::exit_code code2 = experiment(2, model, "multi_vw stl",
                           stl_times, stl_labels);
#ifdef NESTED_ACTIVE
  stapl::exit_code code3 = experiment(3, model, "multi_vw nest",
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
  size_t size = std::sqrt(model);
  size_t atom_pages = size;
  size_t atom_rows = size;
  size_t atom_cols = size;
  size_t stl_pages = size;
  size_t stl_rows = size;
  size_t stl_cols = size;
  size_t stapl_pages = size;
  size_t stapl_rows = size;
  size_t stapl_cols = size;

  bool continue_iterating = true;
  int repeat = 0;
  confidence_interval_controller iter_control(CONF_INT_REP, 100, 0.05);
  while (continue_iterating && success) {
    repeat++;
    switch(test) {
    case 1:
      timer.reset();
      timer.start();
      success = run_atom_test(atom_pages,atom_rows,atom_cols, times, labels);
      iter_control.push_back(timer.stop());
      break;
    case 2:
      timer.reset();
      timer.start();
      success = run_stl_test(stl_pages,stl_rows,stl_cols, times, labels);
      iter_control.push_back(timer.stop());
      break;
#ifdef NESTED_ACTIVE
    case 3:
      timer.reset();
      timer.start();
      success = run_nest_test(nest_pages,nest_rows,nest_cols, times, labels);
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
// PLOT: multiarray of atoms
///////////////////////////////////////////////////////////////////////////

typedef int atom_tp;

typedef stapl::multiarray<3, atom_tp, trav3_yz_tp, part3_tp> ary3_yz_atom_tp;
typedef stapl::multiarray_view<ary3_yz_atom_tp>             ary3_yz_atom_vw_tp;

typedef stapl::multiarray<3, atom_tp, trav3_xy_tp, part3_tp> ary3_xy_atom_tp;
typedef stapl::multiarray_view<ary3_xy_atom_tp>             ary3_xy_atom_vw_tp;

typedef stapl::multiarray<3, atom_tp, trav3_xz_tp, part3_tp> ary3_xz_atom_tp;
typedef stapl::multiarray_view<ary3_xz_atom_tp>             ary3_xz_atom_vw_tp;

typedef stapl::plus<atom_tp> add_atom_wf;


bool run_atom_test(size_t pages, size_t rows, size_t cols,
                   double *times, string *labels ) {

  double time_start, time_end, time_delta;

  // construct container
  stapl::tuple<size_t,size_t,size_t> dims = stapl::make_tuple(pages,rows,cols);

  ary3_yz_atom_tp a_ct(dims), b_ct(dims), c_ct(dims);
  ary3_xy_atom_tp u_ct(dims), v_ct(dims), w_ct(dims);
  ary3_xz_atom_tp x_ct(dims), y_ct(dims), z_ct(dims);

  seconds(time_start);

  // METRIC: construct view over container
  ary3_yz_atom_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);
  ary3_xy_atom_vw_tp u_vw(u_ct), v_vw(v_ct), w_vw(w_ct);
  ary3_xz_atom_vw_tp x_vw(x_ct), y_vw(y_ct), z_vw(z_ct);

  auto a_lin_vw = stapl::linear_view(a_vw);
  auto b_lin_vw = stapl::linear_view(b_vw);
  auto u_lin_vw = stapl::linear_view(u_vw);
  auto v_lin_vw = stapl::linear_view(v_vw);
  auto x_lin_vw = stapl::linear_view(x_vw);
  auto y_lin_vw = stapl::linear_view(y_vw);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: multi-array");
  seconds(time_start);

  // METRIC: initialize containers in parallel
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  int rep = 12;
  typedef stapl::block_sequence<int> repeat_wf;

  stapl::generate(a_lin_vw, step_wf(base,step));
  stapl::generate(b_lin_vw, repeat_wf(base,rep));
  stapl::iota(u_lin_vw, 0);

  stapl::generate(v_lin_vw, step_wf(base,step));
  stapl::generate(x_lin_vw, repeat_wf(base,rep));
  stapl::iota(y_lin_vw, 0);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: multi-array");
  seconds(time_start);

#if 0
  // METRIC: process elements in parallel in yzplan major order
  stapl::transform( a_vw, b_vw, c_vw, add_atom_wf() );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: multi-array");
  seconds(time_start);

#if 0
  // METRIC: process elements in parallel in xyplane major order
  stapl::transform( u_vw, v_vw, w_vw, add_atom_wf() );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("view: multi-array");
  seconds(time_start);

#if 0
  // METRIC: process elements in parallel xzplane major order
  stapl::transform( x_vw, y_vw, z_vw, add_atom_wf() );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[4] += time_delta;
  labels[4] = string("view: multi-array");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// PLOT: multiarray of STL vectors
///////////////////////////////////////////////////////////////////////////

typedef vector<atom_tp> stl_tp;

typedef stapl::multiarray<3, stl_tp, trav3_yz_tp, part3_tp> ary3_yz_stl_tp;
typedef stapl::multiarray_view<ary3_yz_stl_tp>           ary3_yz_stl_vw_tp;

typedef stapl::multiarray<3, stl_tp, trav3_xy_tp, part3_tp> ary3_xy_stl_tp;
typedef stapl::multiarray_view<ary3_xy_stl_tp>           ary3_xy_stl_vw_tp;

typedef stapl::multiarray<3, stl_tp, trav3_xz_tp, part3_tp> ary3_xz_stl_tp;
typedef stapl::multiarray_view<ary3_xz_stl_tp>           ary3_xz_stl_vw_tp;

struct map_sz_wf
{
  typedef size_t result_type;
  template <typename Vec1>
  result_type operator()(Vec1 const v1) {
    return v1.size();
  }
};

typedef stapl::plus<size_t> add_sz_wf;


bool run_stl_test(size_t pages, size_t rows, size_t cols,
                  double *times, string *labels ) {

  double time_start, time_end, time_delta;

stapl::stream<ofstream> zout;
zout.open("vw_multi_stl.txt");

stapl::do_once( msg_val<int>( zout, "Pages ", pages ));
stapl::do_once( msg_val<int>( zout, "Rows ", rows ));
stapl::do_once( msg_val<int>( zout, "Cols ", cols ));

  // construct container
  stapl::tuple<size_t,size_t,size_t> dims = stapl::make_tuple(pages,rows,cols);
  ary3_yz_stl_tp a_ct(dims), b_ct(dims), c_ct(dims);
  ary3_xy_stl_tp u_ct(dims), v_ct(dims), w_ct(dims);
  ary3_xz_stl_tp x_ct(dims), y_ct(dims), z_ct(dims);
stapl::do_once( msg_val<int>( zout, "Trace ",1 ));

  seconds(time_start);

  // METRIC: construct view over container
  ary3_yz_stl_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);
  ary3_xy_stl_vw_tp u_vw(u_ct), v_vw(v_ct), w_vw(w_ct);
  ary3_xz_stl_vw_tp x_vw(x_ct), y_vw(y_ct), z_vw(z_ct);

  auto a_lin_vw = stapl::linear_view(a_vw);
  auto b_lin_vw = stapl::linear_view(b_vw);
  auto u_lin_vw = stapl::linear_view(u_vw);
  auto v_lin_vw = stapl::linear_view(v_vw);
  auto x_lin_vw = stapl::linear_view(x_vw);
  auto y_lin_vw = stapl::linear_view(y_vw);
stapl::do_once( msg_val<int>( zout, "Trace ",2 ));

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: multi-array");
  seconds(time_start);

  // METRIC: initialize containers in parallel
  size_t size = pages * rows * cols;
  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(size));
stapl::do_once( msg_val<int>( zout, "Trace ",3 ));

  stapl::map_func( init_stl_vec_wf(), a_lin_vw, len_vw );
  stapl::map_func( init_stl_vec_wf(), u_lin_vw, len_vw );
stapl::do_once( msg_val<int>( zout, "Trace ",4 ));

#ifdef STAPL_OR_TEST_BUG
  stapl::map_func( init_stl_vec_wf(), x_lin_vw, len_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: multi-array");
  seconds(time_start);

#if 0
  // METRIC: process elements in parallel yzplane major order
  atom_tp sum = stapl::map_reduce( map_sz_wf(), add_sz_wf(), a_lin_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: multi-array");
  seconds(time_start);

#if 0
  // METRIC: process elements in parallel in xyplane major order
  sum = stapl::map_reduce( map_sz_wf(), add_sz_wf(), u_lin_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("view: multi-array");
  seconds(time_start);

#if 0
  // METRIC: process elements in parallel xzplane major order
  sum = stapl::map_reduce( map_sz_wf(), add_sz_wf(), x_vw );
#endif

  seconds(time_end);
  time_delta = time_end - time_start;
  times[4] += time_delta;
  labels[4] = string("view: multi-array");

  return true;
}

///////////////////////////////////////////////////////////////////////////
// plot: multiarray of STAPL vectors
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<atom_tp> nest_tp;

typedef stapl::multiarray<3, nest_tp, trav3_yz_tp, part3_tp> ary3_yz_nest_tp;
typedef stapl::multiarray_view<ary3_yz_nest_tp>           ary3_yz_nest_vw_tp;

typedef stapl::multiarray<3, nest_tp, trav3_xy_tp, part3_tp> ary3_xy_nest_tp;
typedef stapl::multiarray_view<ary3_xy_nest_tp>           ary3_xy_nest_vw_tp;

typedef stapl::multiarray<3, nest_tp, trav3_xz_tp, part3_tp> ary3_xz_nest_tp;
typedef stapl::multiarray_view<ary3_xz_nest_tp>           ary3_xz_nest_vw_tp;

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

struct max_nest_inner_wf
{
  typedef void result_type;
  template <typename Elem1, typename Elem2, typename Elem3>
  result_type operator()(Elem1 v1, Elem2 v2, Elem3 &v3) {
    // CODE
  }
};

struct max_nest_outer_wf
{
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const v1, View2 const v2, View3 const &v3) {
    stapl::map_func(max_nest_inner_wf(), v1, v2, v3 );
  }
};


bool run_nest_test(size_t pages, size_t rows, size_t cols,
                   double *times, string *labels ) {

  double time_start, time_end, time_delta;

  // construct container
  size_t inner = 100 * (rand() % 10);
  size_t outer = pages * rows * cols;
  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::tuple<size_t,size_t,size_t> dims = stapl::make_tuple(pages,rows,cols);
  ary3_yz_nest_tp a_ct(dims), b_ct(dims), c_ct(dims);
  ary3_xy_nest_tp u_ct(dims), v_ct(dims), w_ct(dims);
  ary3_xz_nest_tp x_ct(dims), y_ct(dims), z_ct(dims);

  seconds(time_start);

  // metric: construct view over container
  ary3_yz_nest_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);
  ary3_xy_nest_vw_tp u_vw(u_ct), v_vw(v_ct), w_vw(w_ct);
  ary3_xz_nest_vw_tp x_vw(x_ct), y_vw(y_ct), z_vw(z_ct);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[0] += time_delta;
  labels[0] = string("view: multi-array");
  seconds(time_start);

  // metric: initialize containers in parallel
  stapl::map_func( fill_wf(), a_vw );

  seconds(time_end);
  time_delta = time_end - time_start;
  times[1] += time_delta;
  labels[1] = string("view: multi-array");
  seconds(time_start);

  // metric: process elements in parallel
  stapl::map_func(max_nest_outer_wf(), a_vw, b_vw, c_vw);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[2] += time_delta;
  labels[2] = string("view: multi-array");
  seconds(time_start);

  // metric: process elements in parallel in xyplane_major
  stapl::map_func(max_nest_outer_wf(), u_vw, v_vw, w_vw);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[3] += time_delta;
  labels[3] = string("view: multi-array");
  seconds(time_start);

  // metric: process elements in parallel xzplane_major
  stapl::map_func(max_nest_outer_wf(), x_vw, y_vw, z_vw);

  seconds(time_end);
  time_delta = time_end - time_start;
  times[4] += time_delta;
  labels[4] = string("view: multi-array");

  return true;
}
#endif

