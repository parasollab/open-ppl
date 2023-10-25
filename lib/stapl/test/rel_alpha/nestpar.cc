/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <cmath>
#include <string>
#include <iostream>

#include <stapl/utility/tuple.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/list_view.hpp>
#include <stapl/containers/list/list.hpp>

#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/containers/matrix/matrix.hpp>

#ifdef SET_IMPL
#include <stapl/views/set_view.hpp>
#include <stapl/containers/set/set.hpp>
#endif

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/containers/unordered_map/unordered_map.hpp>

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>


#include <stapl/views/segmented_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/runtime.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

#include "testutil.hpp"
#include "rel_alpha_data.h"

using namespace std;

//////////////////////////////////////////////////////////////////////

typedef stapl::negate<int> ineg_wf;
typedef stapl::identity<int> iid_wf;
typedef stapl::plus<int> iadd_wf;
typedef stapl::minus<int> isub_wf;
typedef stapl::multiplies<int> imul_wf;
typedef stapl::min<int> imin_wf;
typedef stapl::max<int> imax_wf;
typedef stapl::logical_or<int> ior_wf;
typedef stapl::logical_and<int> iand_wf;

template <typename Functor>
struct self_assign
{
  Functor m_func;

  self_assign(Functor const& f)
    : m_func(f)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_func);
  }

  template <typename Ref>
  void operator()(Ref val)
  {
    val = m_func(val);
  }
};

template <typename Functor>
self_assign<Functor> make_self_assign(Functor const& f)
{
  return self_assign<Functor>(f);
}

//////////////////////////////////////////////////////////////////////

// 1-level containers

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_view_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_view_tp;

typedef stapl::list<int> list_int_tp;
typedef stapl::list_view<list_int_tp> list_int_view_tp;

#ifdef SET_IMPL
typedef stapl::set<int> set_int_tp;
typedef stapl::unordered_set<int> unordered_set_int_tp;
#endif

typedef stapl::splitter_partition<vec_int_view_tp::domain_type> seg_splitter;
typedef stapl::segmented_view<vec_int_view_tp, seg_splitter> seg_vw;

// 2-level containers

typedef stapl::vector< stapl::vector<int> > vec_vec_int_tp;
typedef stapl::vector_view<vec_vec_int_tp> vec_vec_int_view_tp;

typedef stapl::vector< stapl::list<int> > vec_list_int_tp;
typedef stapl::vector_view<vec_list_int_tp> vec_list_int_view_tp;

typedef stapl::list< stapl::vector<int> > list_vec_int_tp;
typedef stapl::list_view<list_vec_int_tp> list_vec_int_view_tp;

typedef stapl::list< stapl::list<int> > list_list_int_tp;
typedef stapl::list_view<list_list_int_tp> list_list_int_view_tp;

typedef stapl::array< stapl::array<int> > ary_ary_int_tp;
typedef stapl::array_view<ary_ary_int_tp> ary_ary_int_view_tp;

typedef stapl::vector< stapl::array<int> > vec_ary_int_tp;
typedef stapl::vector_view<vec_ary_int_tp> vec_ary_int_view_tp;

typedef stapl::array< stapl::vector<int> > ary_vec_int_tp;
typedef stapl::array_view<ary_vec_int_tp> ary_vec_int_view_tp;

typedef stapl::list< stapl::map<int, int> > list_map_int_tp;
typedef stapl::list_view<list_map_int_tp> list_map_int_view_tp;

typedef stapl::static_array< stapl::static_array<int> > stary_stary_int_tp;
typedef stapl::array_view<stary_stary_int_tp> stary_stary_int_view_tp;

typedef struct {
  double x_;
  double y_;
  double z_;
} point;

typedef stapl::list<point> list_pnt_tp;
typedef stapl::list_view<list_pnt_tp> list_pnt_view_tp;

typedef stapl::vector<point> vec_pnt_tp;
typedef stapl::vector_view<vec_pnt_tp> vec_pnt_view_tp;

typedef stapl::indexed_domain<size_t>                              vec_dom_tp;
typedef stapl::balanced_partition<vec_dom_tp>                      bal_tp;
typedef stapl::row_major                                           trav_tp;
typedef stapl::nd_partition<stapl::tuple<bal_tp, bal_tp>, trav_tp> part_tp;
typedef stapl::tuple<size_t, size_t>                               mat_gid_tp;


//////////////////////////////////////////////////////////////////////

bool nestpar_01( size_t model );
bool nestpar_02( size_t model );
bool nestpar_03( size_t model );
bool nestpar_04( size_t model );
bool nestpar_05( size_t model );
bool nestpar_06( size_t model );
bool nestpar_07( size_t model );
bool nestpar_08( size_t model );
bool nestpar_09( size_t model );
bool nestpar_10( size_t model );
bool nestpar_11( size_t model );
bool nestpar_12( size_t model );
bool nestpar_13( size_t model );
bool nestpar_14( size_t model );
bool nestpar_15( size_t model );
bool nestpar_16( size_t model );
bool nestpar_17( size_t model );
bool nestpar_18( size_t model );
bool nestpar_19( size_t model );
bool nestpar_20( size_t model );
bool nestpar_21( size_t model );
bool nestpar_22( size_t model );
bool nestpar_23( size_t model );
bool nestpar_24( size_t model );
bool nestpar_25( size_t model );

//////////////////////////////////////////////////////////////////////

bool opt_list = false, opt_noref = false;
bool opt_out = false, opt_quiet = false;
int opt_test = -1;
char *opt_data = 0;
bool opt_stapl= false;
bool opt_manual= false;

stapl::exit_code stapl_main(int argc, char **argv) {

  char *temp = 0;
  for ( int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ('-' == opt[0] ) {
      switch( opt[1] ) {
      case 'h':
        cerr << "HELP\n";
        break;
      case 'd':
        opt_data = argv[argi++];
        break;
      case 'l':
        opt_list = true;
        break;
      case 'n':
        opt_noref = true;
        break;
      case 'o':
        opt_out = true;
        break;
      case 'q':
        opt_quiet = true;
        break;
      case 't':
        temp = argv[argi++];
        opt_test = atoi(temp);
        break;
      case 'v':
        temp = argv[argi++];
        opt_stapl = (0 == strcmp("stapl", temp) );
        opt_manual = (0 == strcmp("manual", temp) );
        break;
      }
    } else {
      cerr << "unknown command line argument " << opt << endl;
    }
  }
  if (!opt_stapl && !opt_manual ) {
    // default behavior
    opt_stapl = true;
    opt_manual = true;
  }

  if (opt_list ) {
    // Boost.PropertyTree will only serialize a node with unnamed subkeys
    // as an array.  Therefore we've got to build extra trees in order to
    // create the unnamed subkeys needed to form proper JSON output.
    typedef boost::property_tree::ptree ptree;
    ptree manual_tree;
    ptree stapl_tree;
    manual_tree.push_back(std::make_pair("name", ptree("stapl")));
    stapl_tree.push_back(std::make_pair("name", ptree("manual")));
    ptree version_array;
    version_array.push_back(std::make_pair("", manual_tree));
    version_array.push_back(std::make_pair("", stapl_tree));

    ptree test_array;
    for ( int i = 1; i<25; i++ ) {
      ptree test;
      char buf[8];
      sprintf(buf,"t%03d",i);
      test.push_back(std::make_pair(std::string(buf), version_array));
      test_array.push_back(std::make_pair("", test));
    }

    // The array of tests is the property of a key named "tests".
    ptree test_tree;
    test_tree.push_back(std::make_pair("tests", test_array));
    write_json(std::cout, test_tree, false);
    return EXIT_SUCCESS;
  }

  int model = -1;
  switch( opt_data[0] ) {
  case 't':
    model = 1;
    break;
  case 's':
    model = 100;
    break;
  case 'm':
    model = 10000;
    break;
  case 'b':
    model = 10000000;
    break;
  case 'h':
    model = 100000000;
    break;
  default:
    cerr << "opt_data " << opt_data << endl;
    break;
  }
  if (model == -1) {
    std::cerr << "usage: exe -data tiny/small/medium/big/huge\n";
    exit(1);
  }

  int first_test = 1;
  int last_test = 36;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  int size = 1000 * model;

  vec_int_tp a(size), b(size), c(size), d(size);
  vec_int_tp e(size), f(size), g(size), h(size);
  //size_vec_tp p(size), q(size);

  vec_int_view_tp a_vw(a), b_vw(b), c_vw(c), d_vw(d);
  vec_int_view_tp e_vw(e), f_vw(f), g_vw(g), h_vw(h);
  //size_vec_view_tp p_vw(p), q_vw(q);

  int fail_count = 0;
  bool ok = true;

  first_test = 1; last_test = 26; // EDIT
  for (int test = first_test; test <= last_test; test++ ) {
    cerr << "\n";
    switch (test) {
    case 1:
      ok = nestpar_01(model);
      break;
    case 2:
      ok = nestpar_02(model);
      break;
    case 3:
      ok = nestpar_03(model);
      break;
    case 4:
      ok = nestpar_04(model);
      break;
    case 5:
      ok = nestpar_05(model);
      break;
    case 6:
      ok = nestpar_06(model);
      break;
    case 7:
      ok = nestpar_07(model);
      break;
    case 8:
      ok = nestpar_08(model);
      break;
    case 9:
      ok = nestpar_09(model);
      break;
    case 10:
      ok = nestpar_10(model);
      break;
    case 11:
      ok = nestpar_11(model);
      break;
    case 12:
      ok = nestpar_12(model);
      break;
    case 13:
      ok = nestpar_13(model);
      break;
    case 14:
      ok = nestpar_14(model);
      break;
    case 15:
      ok = nestpar_15(model);
      break;
    case 16:
      ok = nestpar_16(model);
      break;
    case 17:
      ok = nestpar_17(model);
      break;
    case 18:
      ok = nestpar_18(model);
      break;
    case 19:
      ok = nestpar_19(model);
      break;
    case 20:
      ok = nestpar_20(model);
      break;
    case 21:
      ok = nestpar_21(model);
      break;
    case 22:
      ok = nestpar_22(model);
      break;
    case 23:
      ok = nestpar_23(model);
      break;
    case 24:
      ok = nestpar_24(model);
      break;
    case 25:
      ok = nestpar_25(model);
      break;
    }
  }
  return EXIT_SUCCESS;
}

//////////////////////////////////////////////////////////////////////

void read_map(std::multimap<int,int> graf_map, const char * name) {
  FILE *file = fopen(name,"r");
  if (NULL == file ) {
    cerr << "unable to open " << name << endl;
    exit(1);
  }

  const int buf_siz = 32768;
  char lin_buf[buf_siz];
  lin_buf[0] = '\0';
  while ( fgets(lin_buf, buf_siz, file) ) {
    lin_buf[strlen(lin_buf)-1] = '\0';
    char *token = strtok(lin_buf, ":");
    int src = atoi(token);
    while ( (token = strtok(NULL, " ")) ) {
      int dest = atoi(token);

      graf_map.insert(make_pair(src,dest));
    }
  }
  fclose(file);
}

//////////////////////////////////////////////////////////////////////
// DEFECTS
//////////////////////////////////////////////////////////////////////

// @bug: get_element doesn't compile
// @bug: dereferenced iterator on nested container doesn't compile
#define GET_ELEM 1

// @bug: can't insert or append values to nested containers
#undef PUSH_INSERT


// @bug set container is not implemented
#undef SET_IMPL

// @bug : multimap container not implemented
#undef MMAP_IMPL

// @bug : multiset container not implemented
#undef MSET_IMPL

// @bug : unordered_set container is not implemented
#undef USET_IMPL

// @bug : map container doesn't support nesting
#undef OUTER_MAP

// @bug : matrix container requires partition
#undef MATRIX_DIM

// @bug : outer multiarray needs dimensions
#undef MULTI_ARRAY

// @bug : can't add graph as a vertex of another graph
#undef GRAF_VTX

/*==========================================================================*/

//////////////////////////////////////////////////////////////////////
// nested parallelism case 1: (STL) : set(vec(list))
//
// A parallel associative class (set)
// which contains multiple nested instances of
// two parallel sequence classes (list, vector).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

#ifdef SET_IMPL
typedef stapl::set< stapl::vector< stapl::list<int> > > set_vec_list_int_tp;
typedef stapl::set_view<set_vec_list_int_tp> set_vec_list_int_view_tp;

void nestpar_01_build( size_t model, set_vec_list_int_tp outer,
                       set_vec_list_int_view_tp outer_vw ) {

  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    vec_list_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      list_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item.push_back( inner_item );
      }
      outer_item.push_back( middle_item );
    }
    outer.insert( outer_item );
  }
}
#endif

#ifdef SET_IMPL
size_t nestpar_01_visit( set_vec_list_int_tp outer,
                       set_vec_list_int_view_tp outer_vw ) {
  size_t cksum = 0;
  set_vec_list_int_tp::iterator outer_it;
  set_vec_list_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    vec_list_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    vec_list_int_tp outer_item = *outer_it;
#endif

    vec_list_int_tp::iterator middle_it;
    vec_list_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      list_int_tp inner; // = middle.get_element(middle_gid);
#else
      list_int_tp middle_item = *middle_it;
#endif

      list_int_tp::iterator inner_it;
      list_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}
#endif

bool nestpar_01( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef SET_IMPL
  set_vec_list_int_tp outer;
  set_vec_list_int_view_tp outer_vw(outer);

  nestpar_01_build( model, outer, outer_vw );

  size_t cksum = nestpar_01_visit( outer, outer_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 2: (STL) : uset(list(vec))
//
// A parallel unordered associative class (unordered_set)
// which contains multiple nested instances of
// two parallel sequence classes (list, vector).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

#ifdef USET_IMPL
typedef stapl::unordered_set<
          stapl::list< stapl::vector<int> > > uset_list_vec_int_tp;
typedef stapl::unordered_set_view<
          uset_list_vec_int_tp>               uset_list_vec_int_view_tp;

void nestpar_02_build( size_t model, uset_vec_list_int_tp outer,
                       uset_vec_list_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    list_vec_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      vec_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item.push_back( inner_item );
      }
      outer_item.push_back( middle_item );
    }
    outer.insert( outer_item );
  }
}
#endif

#ifdef USET_IMPL
size_t nestpar_02_visit( uset_vec_list_int_tp outer,
                         uset_vec_list_int_view_tp outer_vw ) {

  size_t cksum = 0;
  uset_vec_list_int_tp outer;
  uset_vec_list_int_view_tp outer_vw(outer);

  uset_list_vec_int_tp::iterator outer_it;
  uset_list_vec_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    list_vec_int_tp outer_item; // = outer.get_element(outer_gid);
#else
#endif

    list_vec_int_tp::iterator middle_it;
    list_vec_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      vec_int_tp inner; // = middle.get_element(middle_gid);
#else
#endif

      vec_int_tp::iterator inner_it;
      vec_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}
#endif

bool nestpar_02( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USET_IMPL
  uset_vec_list_int_tp outer;
  uset_vec_list_int_view_tp outer_vw(outer);

  nestpar_02_build( model, outer, outer_vw );

  size_t cksum = nestpar_02_visit( outer, outer_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 3: dygraf(vec(vec))
//
// A parallel relational class (graph)
// which contains multiple nested instances of
// two parallel sequence classes (list, vector).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

typedef stapl::dynamic_graph< stapl::UNDIRECTED, stapl::NONMULTIEDGES,
        vec_vec_int_tp, int > dygraf_vec_vec_int_tp;
typedef stapl::graph_view<dygraf_vec_vec_int_tp> dygraf_vec_vec_int_view_tp;

void nestpar_03_build( size_t model, dygraf_vec_vec_int_tp outer,
                       dygraf_vec_vec_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    vec_vec_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      vec_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item.push_back( inner_item );
      }
#ifdef PUSH_INSERT
      outer_item.push_back( middle_item );
#endif
    }
#ifdef PUSH_INSERT
    outer.insert( outer_item ); // add_vertex
#endif
  }
}

size_t nestpar_03_visit( dygraf_vec_vec_int_tp outer,
                         dygraf_vec_vec_int_view_tp outer_vw ) {

  size_t cksum = 0;
  dygraf_vec_vec_int_tp::iterator outer_it;
  dygraf_vec_vec_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    vec_vec_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    vec_vec_int_tp outer_item = *outer_it;
#endif

    vec_vec_int_tp::iterator middle_it;
    vec_vec_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      vec_int_tp inner; // = middle.get_element(middle_gid);
#else
      vec_int_tp middle_item = *middle_it;
#endif

      vec_int_tp::iterator inner_it;
      vec_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}

bool nestpar_03( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  dygraf_vec_vec_int_tp outer;
  dygraf_vec_vec_int_view_tp outer_vw(outer);

  nestpar_03_build( model, outer, outer_vw );

  size_t cksum = nestpar_03_visit( outer, outer_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 4: map(ary(vec))
//
// A parallel associative class (map)
// which contains multiple nested instances of
// two parallel indexed classes (array, static_array, matrix).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

typedef stapl::map<
          int, stapl::vector< stapl::array<int> > > map_vec_ary_int_tp;
typedef stapl::map_view<map_vec_ary_int_tp>         map_vec_ary_int_view_tp;

void nestpar_04_build( size_t model, map_vec_ary_int_tp outer,
                       map_vec_ary_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    vec_ary_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      ary_int_tp middle_item(inner_cnt);
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item[inner_ndx] = inner_item;
      }
#ifdef PUSH_INSERT
      outer_item.push_back( middle_item );
#endif
    }
#ifdef PUSH_INSERT
    outer.insert( make_pair( outer_ndx, outer_item ) );
#endif
  }
}

size_t nestpar_04_visit( map_vec_ary_int_tp outer,
                       map_vec_ary_int_view_tp outer_vw ) {
  size_t cksum = 0;

#ifdef OUTER_MAP
  map_vec_ary_int_tp::iterator outer_it;
  map_vec_ary_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    vec_ary_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    vec_ary_int_tp outer_item = *outer_it;
#endif

    vec_ary_int_tp::iterator middle_it;
    vec_ary_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      ary_int_tp inner; // = middle.get_element(middle_gid);
#else
      ary_int_tp middle_item = *middle_iter;
#endif

      ary_int_tp::iterator inner_it;
      ary_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
#endif
  return cksum;
}

bool nestpar_04( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef OUTER_MAP
  map_vec_ary_int_tp outer;
  map_vec_ary_int_view_tp outer_vw(outer);

  nestpar_04_build( model, outer, outer_vw );

  size_t cksum = nestpar_04_visit( outer, outer_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 5: umap(vec(ary))
//
// A parallel unordered associative class (unordered_map)
// which contains multiple nested instances of
// two parallel indexed class (array, static_array, matrix).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

typedef stapl::unordered_map<
          int, stapl::vector< stapl::array<int> > > umap_vec_ary_int_tp;
typedef stapl::map_view<umap_vec_ary_int_tp>        umap_vec_ary_int_view_tp;

void nestpar_05_build( size_t model, umap_vec_ary_int_tp outer,
                       umap_vec_ary_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    vec_ary_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      ary_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item[inner_ndx] = inner_item;
      }
#ifdef PUSH_INSERT
      outer_item.push_back( middle_item );
#endif
    }
#ifdef PUSH_INSERT
    outer.insert( make_pair( outer_ndx, outer_item ) );
#endif
  }
}

size_t nestpar_05_visit( umap_vec_ary_int_tp outer,
                         umap_vec_ary_int_view_tp outer_vw ) {

  size_t cksum = 0;
  umap_vec_ary_int_tp::iterator outer_it;
  umap_vec_ary_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    vec_ary_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    vec_ary_int_tp outer_item = *outer_it;
#endif

    vec_ary_int_tp::iterator middle_it;
    vec_ary_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      ary_int_tp inner; // = middle.get_element(middle_gid);
#else
      ary_int_tp middle_item = *middle_it;
#endif

      ary_int_tp::iterator inner_it;
      ary_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}

bool nestpar_05( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  umap_vec_ary_int_tp outer;
  umap_vec_ary_int_view_tp outer_vw(outer);

  nestpar_05_build( model, outer, outer_vw );

  size_t cksum =nestpar_05_visit( outer, outer_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 6: dygraf(vec(ary))
//
// A parallel relational class (dynamic_graph)
// which contains multiple nested instances of
// two parallel indexed classes (array, static_array, matrix).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

typedef stapl::dynamic_graph< stapl::UNDIRECTED, stapl::NONMULTIEDGES,
        vec_ary_int_tp, int > dygraf_vec_ary_int_tp;
typedef stapl::graph_view<dygraf_vec_ary_int_tp> dygraf_vec_ary_int_view_tp;

void nestpar_06_build( size_t model, dygraf_vec_ary_int_tp outer,
                       dygraf_vec_ary_int_view_tp outer_vw) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    vec_ary_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      ary_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item[inner_ndx] = inner_item;
      }
#ifdef PUSH_INSERT
      outer_item.push_back( middle_item );
#endif
    }
#ifdef PUSH_INSERT
    outer.insert( outer_item ); // add_vertex
#endif
  }
}

size_t nestpar_06_visit( dygraf_vec_ary_int_tp outer,
                       dygraf_vec_ary_int_view_tp outer_vw) {
  size_t cksum = 0;

  dygraf_vec_ary_int_tp::iterator outer_it;
  dygraf_vec_ary_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    vec_ary_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    vec_ary_int_tp outer_item = *outer_it;
#endif

    vec_ary_int_tp::iterator middle_it;
    vec_ary_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      ary_int_tp inner; // = middle.get_element(middle_gid);
#else
      ary_int_tp middle_item = *middle_it;
#endif

      ary_int_tp::iterator inner_it;
      ary_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}

bool nestpar_06( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  dygraf_vec_ary_int_tp outer;
  dygraf_vec_ary_int_view_tp outer_vw(outer);

  nestpar_06_build( model, outer, outer_vw );

  size_t cksum = nestpar_06_visit( outer, outer_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 7: (STL) : list(list(vec))
//
// A parallel sequence container class (list)
// which contains multiple nested instances of
// other parallel sequence container classes (list, vector).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

typedef stapl::list< stapl::list< stapl::vector<int> > > list_list_vec_int_tp;
typedef stapl::list_view<list_list_vec_int_tp> list_list_vec_int_view_tp;

void nestpar_07_build( size_t model, list_list_vec_int_tp outer,
                       list_list_vec_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    list_vec_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      vec_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item.push_back( inner_item );
      }
#ifdef PUSH_INSERT
      outer_item.push_back( middle_item );
#endif
    }
#ifdef PUSH_INSERT
    outer.push_back( outer_item );
#endif
  }
}

size_t nestpar_07_visit( list_list_vec_int_tp outer,
                         list_list_vec_int_view_tp outer_vw ) {

  size_t cksum = 0;
  list_list_vec_int_tp::iterator outer_it;
  list_list_vec_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    list_vec_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    list_vec_int_tp outer_item = *outer_it;
#endif

    list_vec_int_tp::iterator middle_it;
    list_vec_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      vec_int_tp inner; // = middle.get_element(middle_gid);
#else
      vec_int_tp middle_item = *middle_it;
#endif

      vec_int_tp::iterator inner_it;
      vec_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}

bool nestpar_07( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  list_list_vec_int_tp outer;
  list_list_vec_int_view_tp outer_vw(outer);

  nestpar_07_build( model, outer, outer_vw );

  size_t cksum = nestpar_07_visit( outer, outer_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 8: (STL) : set(set(map))
//
// A parallel associative container class (set)
// which contains multiple nested instances of
// other parallel associative container classes (set, multiset, map, multimap)
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

#ifdef SET_IMPL
typedef stapl::set< stapl::set< stapl::map<int,int> > > set_set_map_int_tp;
typedef stapl::set_view<set_set_map_int_tp> set_set_map_int_view_tp;

void nestpar_08_build( size_t model, set_set_map_int_tp outer,
                       set_set_map_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    set_map_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;
      map_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item.insert( make_pair( inner_item, inner_ndx ) );
      }
      outer_item.insert( middle_item );
    }
    outer.insert( outer_item );
  }
}
#endif

#ifdef SET_IMPL
size_t nestpar_08_visit( set_set_map_int_tp outer,
                       set_set_map_int_view_tp outer_vw ) {
  size_t cksum = 0;
  set_set_map_int_tp outer;
  set_set_map_int_view_tp outer_vw(outer);

  set_set_map_int_tp::iterator outer_it;
  set_set_map_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    set_map_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    set_map_int_tp outer_item = *outer_it;
#endif

    set_map_int_tp::iterator middle_it;
    set_map_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      map_int_tp inner; // = middle.get_element(middle_gid);
#else
      map_int_tp middle_item = *middle_it;
#endif

      map_int_tp::iterator inner_it;
      map_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}
#endif

bool nestpar_08( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef SET_IMPL
  set_set_map_int_tp outer;
  set_set_map_int_view_tp outer_vw(outer);

  nestpar_08_build( model, outer, outer_vw );

  size_t cksum = nestpar_08_visit( outer, outer_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 9: (STL) : umap(umap(uset))
//
// A parallel unordered associative container class (unordered_map)
// which contains multiple nested instances of
// other parallel unordered associative container classes (unordered_set,
// unordered_map) Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

#ifdef USET_IMPL
typedef stapl::unordered_map<
          int, stapl::unordered_map<int, stapl::unordered_set<int, int> >
        >                                       umap_umap_uset_int_tp;
typedef stapl::map_view<umap_umap_uset_int_tp>  umap_umap_uset_int_view_tp;
typedef stapl::unordered_map<
          int, stapl::unordered_set<int, int> > umap_uset_int_tp;
typedef stapl::unordered_set<int, int>          uset_int_tp;

void nestpar_09_build( size_t model, umap_umap_uset_int_tp outer,
                       umap_umap_uset_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    umap_uset_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t middle_ndx = 0; middle_ndx < mid_cnt; ++middle_ndx ) {
      size_t middle_shape = 0;
      uset_int_tp middle_item;
      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item.insert( inner_item );
      }
      outer_item.insert( make_pair( middle_ndx, middle_item ) );
    }
    outer.insert( make_pair( outer_ndx, outer_item ) );
  }
}
#endif

#ifdef USET_IMPL
size_t nestpar_09_visit( umap_umap_uset_int_tp outer,
                       umap_umap_uset_int_view_tp outer_vw ) {

  size_t cksum = 0;
  umap_umap_uset_int_tp::iterator outer_it;
  umap_umap_uset_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    umap_uset_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    umap_uset_int_tp outer_item = *outer_it;
#endif

    umap_uset_int_tp::iterator middle_it;
    umap_uset_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      uset_int_tp inner; // = middle.get_element(middle_gid);
#else
      uset_int_tp middle_item = *middle_it;
#endif

      uset_int_tp::iterator inner_it;
      uset_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }
  }
  return cksum;
}
#endif

bool nestpar_09( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USET_IMPL
  umap_umap_uset_int_tp outer;
  umap_umap_uset_int_view_tp outer_vw(outer);

  nestpar_09_build( model, outer, outer_vw );

  size_t cksum = nestpar_09_visit( outer, outer_vw );

  stapl::rmi_fence();

#endif
  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 10: ary(ary(mat))
//
// A parallel indexed container class (array)
// which contains multiple nested instances of
// other parallel indexed container classes (array, static_array, matrix,
// multiarray)
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

typedef stapl::matrix<int> mat_int_tp;
typedef stapl::array< stapl::matrix<int> > ary_mat_int_tp;
typedef stapl::array< stapl::array< stapl::matrix<int> > > ary_ary_mat_int_tp;
typedef stapl::array_view<ary_ary_mat_int_tp> ary_ary_mat_int_view_tp;

void nestpar_10_build( size_t model, ary_ary_mat_int_tp outer,
                       ary_ary_mat_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  size_t inner_cnt = 1 + (rand() % (outer_cnt * 8));
  size_t inner_rows = (size_t) log10( (double) inner_cnt );
  size_t inner_cols = inner_cnt / inner_rows;

  bal_tp part0(vec_dom_tp(0, inner_rows-1), stapl::get_num_locations());
  bal_tp part1(vec_dom_tp(0, inner_cols-1), stapl::get_num_locations());
  part_tp part(part0, part1);

  typedef mat_int_tp::size_type size_tp;
  size_tp dims = size_tp(outer_cnt, inner_cnt);

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    ary_mat_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {

#ifdef MATRIX_DIM
      mat_int_tp middle_item(dims, part);
#endif
      for (size_t inner_i=0; inner_i < inner_rows; ++inner_i ) {
        for (size_t inner_j=0; inner_j < inner_cols; ++inner_j ) {

          mat_gid_tp inner_gid(inner_i,inner_j);
          int inner_item = prime1000[rand1000_01[inner_j%1000]];
#ifdef MATRIX_DIM
          middle_item[inner_gid] = inner_item;
#endif
        }
      }
#ifdef PUSH_INSERT
      outer_item[middle_ndx] = middle_item;
#endif
    }
#ifdef PUSH_INSERT
    outer[outer_ndx] = outer_item;
#endif
  }
}

size_t nestpar_10_visit( ary_ary_mat_int_tp outer,
                        ary_ary_mat_int_view_tp outer_vw ) {

  size_t inner_rows;
  size_t inner_cols;

  size_t cksum = 0;
  ary_ary_mat_int_tp::iterator outer_it;
  ary_ary_mat_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    ary_mat_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    ary_mat_int_tp outer_item = *outer_it;
#endif

    ary_mat_int_tp::iterator middle_it;
    ary_mat_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {
#ifdef MATRIX_DIM
#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      mat_int_tp middle_item(dims, part); // = middle.get_element(middle_it);
#else
      mat_int_tp middle_item(dims, part) = *middle_it;
#endif
#endif

#ifdef MATRIX_DIM
      size_t inner_rows; // = dims[0]
      size_t inner_cols; // = dims[1]
#endif

      for (size_t inner_i=0; inner_i < inner_rows; ++inner_i ) {
        for (size_t inner_j=0; inner_j < inner_cols; ++inner_j ) {

          mat_gid_tp inner_gid(inner_i,inner_j);
#ifdef MATRIX_DIM
          inner_item = middle_item[inner_gid];
          cksum ^= (unsigned int) inner_item;
#endif
        }
      }
    }
  }
  return cksum;
}

bool nestpar_10( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_ary_mat_int_tp outer;
  ary_ary_mat_int_view_tp outer_vw(outer);

  nestpar_10_build( model, outer, outer_vw );

  size_t cksum = nestpar_10_visit( outer, outer_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 11: graf(list(map))
//
// A parallel relational container class (graph)
// which contains multiple nested instances of
// two parallel associative classes (set,map).
// Process each scalar value in the data structure.
// Save result summary.
//////////////////////////////////////////////////////////////////////

typedef stapl::graph< stapl::DIRECTED, stapl::NONMULTIEDGES,
        list_map_int_tp, int > graf_list_map_int_tp;
typedef stapl::graph_view<graf_list_map_int_tp> graf_list_map_int_view_tp;

void nestpar_11_build( size_t model, graf_list_map_int_tp outer,
                       graf_list_map_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 10;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    list_map_int_tp outer_item;
    size_t mid_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t mid_ndx = 0; mid_ndx < mid_cnt; ++mid_ndx ) {
      size_t middle_shape = 0;

      typedef stapl::map<size_t, size_t> map_int_tp;
      map_int_tp middle_item;

      size_t inner_cnt = 1 + (rand() % (mid_cnt * 2));
      for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
        int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
        middle_item.insert( make_pair(inner_ndx, inner_item) );
      }

#ifdef PUSH_INSERT
      outer_item.push_back( middle_item );
#endif
    }
#ifdef PUSH_INSERT
    outer.insert( outer_item ); // add_vertex
#endif
  }
}

size_t nestpar_11_visit( graf_list_map_int_tp outer,
                         graf_list_map_int_view_tp outer_vw ) {

  size_t cksum = 0;
  graf_list_map_int_tp::iterator outer_it;
  graf_list_map_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    list_map_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    list_map_int_tp outer_item = *outer_it;
#endif

    list_map_int_tp::iterator middle_it;
    list_map_int_tp::gid_type middle_gid;
    for (middle_it = outer_item.begin();
       middle_it != outer_item.end(); middle_it++ ) {

      typedef stapl::map<size_t, size_t> map_int_tp;
      map_int_tp middle_item;

#ifdef GET_ELEM
      middle_gid = gid_of(middle_it);
      map_int_tp inner; // = middle.get_element(middle_gid);
#else
      map_int_tp middle_item = *middle_it;
#endif

      map_int_tp::iterator inner_it;
      map_int_tp::gid_type inner_gid;
      for (inner_it = inner.begin();
         inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
        inner_gid = gid_of(inner_it);
        int inner_item; // = inner.get_element(inner_gid);
#else
        int inner_item = *inner_it;
#endif
        cksum ^= (unsigned int) inner_item;
      }
    }

  }
  return cksum;
}

bool nestpar_11( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  graf_list_map_int_tp outer;
  graf_list_map_int_view_tp outer_vw(outer);

  nestpar_11_build( model, outer, outer_vw );

  size_t cksum = nestpar_11_visit( outer, outer_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// this group of tests asks two questions:
// 1) can homogenous nested structures be constructed?
// 2) can those structures be processed with stapl algorithms?
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// nested parallelism case 12: (STL) : list(list(str))
//
// A parallel sequence container (list)
// nested within other parallel sequence container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - count_if
// modifying - merge
// removing - remove_if
// mutating - reverse
// numeric  - accumulate
//////////////////////////////////////////////////////////////////////

typedef stapl::list< stapl::list<point> > list_list_pnt_tp;
typedef stapl::list_view<list_list_pnt_tp> list_list_pnt_view_tp;

void nestpar_12_build( size_t model, list_list_pnt_tp outer,
                       list_list_pnt_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    list_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.push_back( inner_item );
    }
#ifdef PUSH_INSERT
    outer.push_back( outer_item );
#endif
  }
}

size_t nestpar_12_visit(  list_list_pnt_tp outer,
                          list_list_pnt_view_tp outer_vw ) {
  size_t cksum = 0;

  list_list_pnt_tp::iterator outer_it;
  list_list_pnt_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    list_pnt_tp outer_item; // = outer.get_element(outer_gid);
#else
    list_pnt_tp outer_item = *outer_it;
#endif

    list_pnt_tp::iterator inner_it;
    list_pnt_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      point inner_item; // = inner.get_element(inner_gid);
#else
      point inner_item = *inner_it;
#endif
      cksum ^= 0; // (unsigned int) inner_item;
    }
  }
  return cksum;
}

struct nestpar_12_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator() (Reference left, Reference result) {
  }
  template <typename Reference>
  void operator() (Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_12( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  list_list_pnt_tp outer;
  list_list_pnt_view_tp outer_vw(outer);
  list_list_pnt_tp temp;
  list_list_pnt_view_tp temp_vw(temp);

  nestpar_12_build( model, outer, outer_vw );

  size_t cksum = nestpar_12_visit( outer, outer_vw );

  stapl::rmi_fence();

  stapl::map_func( nestpar_12_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 13: (STL) : vec(vec(str))
//
// A parallel sequence container (vector)
// nested within other parallel sequence container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - find_if
// modifying - copy_backward
// removing - remove_copy_if
// mutating - next_permutation
// numeric - adjacent_difference
//////////////////////////////////////////////////////////////////////

typedef stapl::vector< stapl::vector<point> > vec_vec_pnt_tp;
typedef stapl::vector_view<vec_vec_pnt_tp> vec_vec_pnt_view_tp;

void nestpar_13_build( size_t model, vec_vec_pnt_tp outer,
                       vec_vec_pnt_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    vec_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.push_back( inner_item );
    }
#ifdef PUSH_INSERT
    outer.push_back( outer_item );
#endif
  }
}

size_t nestpar_13_visit( vec_vec_pnt_tp outer,
                         vec_vec_pnt_view_tp outer_vw ) {

  size_t cksum = 0;

  vec_vec_pnt_tp::iterator outer_it;
  vec_vec_pnt_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    vec_pnt_tp outer_item; // = outer.get_element(outer_gid);
#else
    vec_pnt_tp outer_item = *outer_it;
#endif

    vec_pnt_tp::iterator inner_it;
    vec_pnt_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      point inner_item; // = inner.get_element(inner_gid);
#else
      point inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) (inner_item.x_ + inner_item.y_);
    }
  }
  return cksum;
}

struct nestpar_13_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

#if 0
// JUNK
    //stapl::map_func( stapl::transform( in, out, ineg_wf() ) );
    //stapl::map_reduce( stapl::identity<Value>(), ineg_wf(), in, out );
    //stapl::map_func( ineg_wf(), in, out );
#endif

bool nestpar_13( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_vec_pnt_tp outer;
  vec_vec_pnt_view_tp outer_vw(outer);

  vec_vec_pnt_tp temp;
  vec_vec_pnt_view_tp temp_vw(temp);

  nestpar_13_build( model, outer, outer_vw );

  size_t cksum = nestpar_13_visit( outer, outer_vw );

  stapl::rmi_fence();

  stapl::map_func( nestpar_13_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 14: (STL) : set(set))
//
// A parallel associative container (set)
// nested within other parallel associative container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - for_each
// modifying - replace_copy
// removing - remove
//////////////////////////////////////////////////////////////////////

#ifdef SET_IMPL
typedef stapl::set< stapl::set<int> > set_set_int_tp;
typedef stapl::set_view<set_set_int_tp> set_set_int_view_tp;

void nestpar_14_build( size_t model, set_set_int_tp outer,
                       set_set_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    set_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.insert( inner_item );
    }
    outer.insert( outer_item );
  }
}
#endif

#ifdef SET_IMPL
size_t nestpar_14_visit( set_set_int_tp outer,
                       set_set_int_view_tp outer_vw ) {

  size_t cksum = 0;

  set_set_int_tp::iterator outer_it;
  set_set_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    set_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    set_inner outer_item = *outer_it;
#endif

    set_int_tp::iterator inner_it;
    set_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= 0; // (unsigned int) inner_item;
    }
  }
  return cksum;
}
#endif

struct nestpar_14_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_14( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef SET_IMPL
  set_set_int_tp outer;
  set_set_int_view_tp outer_vw(outer);

  set_set_int_tp temp;
  set_set_int_view_tp temp_vw(temp);

  nestpar_14_build( model, outer, outer_vw );

  size_t cksum = nestpar_14_visit( outer, outer_vw );

  stapl::rmi_fence();

  stapl::map_func( nestpar_14_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 15: (STL) : bag(bag)
//
// A parallel associative container (multiset)
// nested within other parallel associative container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - count
// modifying - replace_copy_if
// removing - remove_copy
//////////////////////////////////////////////////////////////////////

#ifdef MSET_IMPL
typedef stapl::multiset< stapl::multiset<int> > bag_bag_int_tp;
typedef stapl::multiset_view<bag_bag_int_tp> bag_bag_int_view_tp;

void nestpar_15_build( size_t model, bag_bag_int_tp outer,
                       bag_bag_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    bag_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.insert( inner_item );
    }
    outer.insert( outer_item );
  }
}
#endif

#ifdef MSET_IMPL
size_t nestpar_15_visit( bag_bag_int_tp outer,
                       bag_bag_int_view_tp outer_vw ) {
  size_t cksum = 0;

  bag_bag_int_tp::iterator outer_it;
  bag_bag_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    bag_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    bag_int_tp outer_item = *outer_it;
#endif

    bag_int_tp::iterator inner_it;
    bag_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) inner_item;
    }
  }
  return cksum;
}
#endif

struct nestpar_15_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_15( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef MSET_IMPL
  bag_bag_int_tp outer;
  bag_bag_int_view_tp outer_vw(outer);

  bag_bag_int_tp temp;
  bag_bag_int_view_tp temp_vw(temp);

  nestpar_15_build( model, outer, outer_vw );

  size_t cksum = nestpar_15_visit( outer, outer_vw );

  stapl::rmi_fence();

  stapl::map_func( nestpar_15_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 16: (STL) : map(map)
//
// A parallel associative container (map)
// nested within other parallel associative container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - count_if
// modifying - transform
// removing - unique
//////////////////////////////////////////////////////////////////////

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map< int, stapl::map<int,int> > map_map_int_tp;
typedef stapl::map_view<map_map_int_tp> map_map_int_view_tp;

void nestpar_16_build( size_t model, map_map_int_tp outer,
                       map_map_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
#ifdef OUTER_MAP
    map_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.insert( make_pair( inner_ndx, inner_item ) );
    }
#endif
#ifdef PUSH_INSERT
    outer.insert( make_pair( outer_ndx, outer_item ) );
#endif
  }
}

size_t nestpar_16_visit( map_map_int_tp outer,
                       map_map_int_view_tp outer_vw ) {

  size_t cksum = 0;
#ifdef OUTER_MAP
  map_map_int_tp::iterator outer_it;
  map_map_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    map_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    map_int_tp outer_item = *outer_it;
#endif

    map_int_tp::iterator inner_it;
    map_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) inner_item;
    }
  }
#endif
  return cksum;
}

struct nestpar_16_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_16( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef OUTER_MAP
  map_map_int_tp outer;
  map_map_int_view_tp outer_vw(outer);

  map_map_int_tp temp;
  map_map_int_view_tp temp_vw(temp);

  nestpar_16_build( model, outer, outer_vw );

  size_t cksum = nestpar_16_visit(outer, outer_vw );

  stapl::rmi_fence();

  stapl::map_func( nestpar_16_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 17: (STL) : mmap(mmap)
//
// A parallel associative container (multimap)
// nested within other parallel associative container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - for_each
// modifying - for_each
// removing - unique_copy
//////////////////////////////////////////////////////////////////////

#ifdef MMAP_IMPL
typedef stapl::multimap< stapl::multimap<int> > mmap_mmap_int_tp;
typedef stapl::map_view<mmap_mmap_int_tp> mmap_mmap_int_view_tp;

void nestpar_17_build( size_t model, mmap_mmap_int_tp outer,
                       mmap_mmap_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    mmap_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.insert( make_pair( inner_ndx, inner_item ) );
    }
    outer.insert( make_pair( outer_ndx, outer_item ) );
  }
}
#endif

#ifdef MMAP_IMPL
size_t nestpar_17_visit( mmap_mmap_int_tp outer,
                         mmap_mmap_int_view_tp outer_vw ) {
  size_t cksum = 0;
  mmap_mmap_int_tp::iterator outer_it;
  mmap_mmap_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    mmap_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    mmap_int_tp outer_item = *outer_it;
#endif

    mmap_int_tp::iterator inner_it;
    mmap_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) inner_item;
    }
  }
  return cksum;
}
#endif

struct nestpar_17_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_17( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef MMAP_IMPL
  mmap_mmap_int_tp outer;
  mmap_mmap_int_view_tp outer_vw(outer);

  nestpar_17_build( model, outer, outer_vw );

  size_t cksum = nestpar_17_visit( outer, outer_vw );

  stapl::rmi_fence();

  mmap_mmap_int_tp temp;
  mmap_mmap_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_17_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 18: (STL) : uset(uset)
//
// An unordered parallel associative container (unordered_set)
// nested within other parallel unordered associative container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - find_if
// modifying - transform
// removing - remove
//////////////////////////////////////////////////////////////////////

#ifdef SET_IMPL
typedef stapl::unordered_set< stapl::unordered_set<int> > uset_uset_int_tp;
typedef stapl::unordered_set_view<uset_uset_int_tp> uset_uset_int_view_tp;

void nestpar_18_build( size_t model, uset_uset_int_tp outer,
                       uset_uset_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    uset_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.insert( inner_item );
    }
    outer.insert( outer_item );
  }
}
#endif

#ifdef SET_IMPL
size_t nestpar_18_visit( uset_uset_int_tp outer,
                         uset_uset_int_view_tp outer_vw ) {

  size_t cksum = 0;
  uset_uset_int_tp::iterator outer_it;
  uset_uset_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    uset_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    uset_int_tp outer_item = *outer_it;
#endif

    uset_int_tp::iterator inner_it;
    uset_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) inner_item;
    }
  }
}
#endif

struct nestpar_18_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_18( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef SET_IMPL
  uset_uset_int_tp outer;
  uset_uset_int_view_tp outer_vw(outer);

  nestpar_18_build( model, outer, outer_vw );

  size_t cksum = nestpar_18_visit( outer, outer_vw );

  stapl::rmi_fence();

  uset_uset_int_tp temp;
  uset_uset_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_18_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 19: (STL) : umap(umap)
//
// A parallel unordered associative container (unordered_map)
// nested within other parallel unordered associative container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - find
// modifying - for_each
// removing - remove_copy
//////////////////////////////////////////////////////////////////////

typedef stapl::unordered_map<
          int, stapl::unordered_map<int,int> > umap_umap_int_tp;
typedef stapl::map_view<umap_umap_int_tp>      umap_umap_int_view_tp;
typedef stapl::unordered_map<int,int>          umap_int_tp;

void nestpar_19_build( size_t model, umap_umap_int_tp outer,
                       umap_umap_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    umap_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item.insert( make_pair(inner_ndx, inner_item) );
    }
#ifdef PUSH_INSERT
    outer.insert( make_pair( outer_ndx, outer_item ) );
#endif
  }
}

size_t nestpar_19_visit(  umap_umap_int_tp outer,
                         umap_umap_int_view_tp outer_vw ) {

  size_t cksum = 0;
  umap_umap_int_tp::iterator outer_it;
  umap_umap_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    umap_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    umap_int_tp outer_item = *outer_it;
#endif

    umap_int_tp::iterator inner_it;
    umap_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) inner_item;
    }
  }
  return cksum;
}

struct nestpar_19_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_19( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  umap_umap_int_tp outer;
  umap_umap_int_view_tp outer_vw(outer);

  nestpar_19_build( model, outer, outer_vw );

  size_t cksum = nestpar_19_visit( outer, outer_vw );

  stapl::rmi_fence();

  umap_umap_int_tp temp;
  umap_umap_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_19_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 20: ary(ary)
//
// A parallel indexed container class (array)
// nested within other parallel indexed container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - mismatch
// modifying - replace
// mutating - reverse_copy
// numeric - accumulate
//////////////////////////////////////////////////////////////////////

void nestpar_20_build( size_t model, ary_ary_int_tp outer,
                       ary_ary_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
    ary_int_tp outer_item;
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
      outer_item[inner_ndx]= inner_item;
    }
#ifdef PUSH_INSERT
    outer[outer_ndx] = outer_item;
#endif
  }
}

size_t nestpar_20_visit( ary_ary_int_tp outer,
                         ary_ary_int_view_tp outer_vw ) {
  size_t cksum = 0;

  for (size_t outer_ndx = 0; outer_ndx < outer.size(); outer_ndx++ ) {
#ifdef GET_ELEM
    ary_int_tp outer_item; // = outer[outer_ndx];
#else
    ary_int_tp outer_item = outer[outer_ndx];
#endif

    ary_int_tp::iterator inner_it;
    ary_int_tp::gid_type inner_gid;
    for (size_t inner_ndx = 0; inner_ndx < outer_item.size(); inner_ndx++ ) {
      int inner_item = outer_item[inner_ndx];
      cksum ^= (unsigned int) inner_item;
    }
  }
  return cksum;
}

struct nestpar_20_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_20( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_ary_int_tp outer;
  ary_ary_int_view_tp outer_vw(outer);

  nestpar_20_build( model, outer, outer_vw );

  size_t cksum = nestpar_20_visit( outer, outer_vw );

  stapl::rmi_fence();

  ary_ary_int_tp temp;
  ary_ary_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_20_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 21: mat(mat)
//
// A parallel indexed container class (matrix)
// nested within other parallel indexed container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying -
// for_each, count, count_if, min_element, max_element,
// find, find_if, search_n, search, find_end,
// find_first_of, adjacent_find, equal, mismatch,
// modifying - for_each, transform
// mutating - reverse
// numeric - inner_product
//////////////////////////////////////////////////////////////////////

typedef stapl::matrix<int, trav_tp, part_tp>       mat2_int_tp;
typedef stapl::matrix< mat2_int_tp >               mat_mat_int_tp;
typedef stapl::multiarray_view<mat_mat_int_tp>     mat_mat_int_view_tp;

#if 0
balanced_tp part0(vec_dom_tp(0, n-1), stapl::get_num_locations());
balanced_tp part1(vec_dom_tp(0, m-1), stapl::get_num_locations());
partition_tp part(part0, part1);
mat_int_tp c(gid_tp(n,m), part);
#endif

void nestpar_21_build( size_t model, mat_mat_int_tp outer,
                       mat_mat_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }
  size_t outer_rows = (size_t) log10( (double) outer_cnt );
  size_t outer_cols = outer_cnt / outer_rows;

  size_t inner_cnt = 1 + (rand() % (outer_cnt * 8));
  size_t inner_rows = (size_t) log10( (double) inner_cnt );
  size_t inner_cols = inner_cnt / inner_rows;

  bal_tp part0(vec_dom_tp(0, inner_rows-1), stapl::get_num_locations());
  bal_tp part1(vec_dom_tp(0, inner_cols-1), stapl::get_num_locations());
  part_tp part(part0, part1);

  for (size_t outer_i=0; outer_i < outer_rows; ++outer_i ) {
    for (size_t outer_j=0; outer_j < outer_cols; ++outer_j ) {

      mat_gid_tp outer_gid(outer_i,outer_j);

#ifdef MATRIX_DIM
     mat2_int_tp outer_item (mat_gid_tp(inner_rows,inner_cols), part);
                 // == outer[outer_gid];
#endif
      for (size_t inner_i=0; inner_i < inner_rows; ++inner_i ) {
        for (size_t inner_j=0; inner_j < inner_cols; ++inner_j ) {
          mat_gid_tp inner_gid(inner_i,inner_j);
          int inner_item = prime1000[rand1000_01[inner_j%1000]];
#ifdef MATRIX_DIM
          outer_item[inner_gid] = inner_item;
#endif
        }
      }
#ifdef PUSH_INSERT
      outer[outer_gid]= outer_item;
#endif
    }
  }
}

size_t nestpar_21_visit( mat_mat_int_tp outer,
                         mat_mat_int_view_tp outer_vw ) {

  size_t cksum = 0;

  size_t outer_cnt = outer.size();
  size_t outer_rows = (size_t) log10( (double) outer_cnt );
  size_t outer_cols = outer_cnt / outer_rows;

  size_t inner_cnt = 1 + (rand() % (outer_cnt * 8));
  size_t inner_rows = (size_t) log10( (double) inner_cnt );
  size_t inner_cols = inner_cnt / inner_rows;

  bal_tp part0(vec_dom_tp(0, inner_rows-1), stapl::get_num_locations());
  bal_tp part1(vec_dom_tp(0, inner_cols-1), stapl::get_num_locations());
  part_tp part(part0, part1);

  for (size_t outer_i=0; outer_i < outer_rows; ++outer_i ) {
    for (size_t outer_j=0; outer_j < outer_cols; ++outer_j ) {

      mat_gid_tp outer_gid(outer_i,outer_j);
#ifdef MATRIX_DIM
     mat2_int_tp outer_item (mat_gid_tp(inner_rows,inner_cols), part);
                 // == outer[outer_gid];
#endif
      for (size_t inner_i=0; inner_i < inner_rows; ++inner_i ) {
        for (size_t inner_j=0; inner_j < inner_cols; ++inner_j ) {
          mat_gid_tp inner_gid(inner_i,inner_j);
#ifdef MATRIX_DIM
          int inner_item; // = outer_item[inner_gid];
#else
          int inner_item;
#endif
          cksum ^= (unsigned int) inner_item;
        }
      }
    }
  }

  return cksum;
}

struct nestpar_21_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_21( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef MATRIX_DIM
  mat_mat_int_tp outer;
  mat_mat_int_view_tp outer_vw(outer);

  nestpar_21_build( model, outer, outer_vw );

  size_t cksum = nestpar_21_visit( outer, outer_vw );

  stapl::rmi_fence();

  mat_mat_int_tp temp;
  mat_mat_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_21_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 22: ary3(ary)
//
// A parallel indexed container class (multiarray)
// nested within other parallel indexed container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - equal
// modifying - replace_copy
// mutating - rotate_copy
// numeric - adjacent_difference
//////////////////////////////////////////////////////////////////////

typedef stapl::multiarray<3, stapl::array<int> > ary3_ary_int_tp;
typedef stapl::array_view<ary3_ary_int_tp> ary3_ary_int_view_tp;
typedef stapl::multiarray<3,int> ary3_tp;
typedef stapl::tuple<size_t,size_t,size_t> gid3_tp;

void nestpar_22_build( size_t model, ary3_ary_int_tp outer,
                       ary3_ary_int_view_tp outer_vw ) {

  size_t inner_max = 19;
  ary3_tp::size_type dims = outer.dimensions();
  for (size_t i=0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j=0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k=0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
#ifdef GET_ELEM
        ary_int_tp outer_item; // = outer[gid];
#else
        ary_int_tp outer_item = outer[gid];
#endif
        size_t inner_cnt = 1 + (rand() % inner_max);
        for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
          int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
          outer_item[inner_ndx] = inner_item;
        }
#ifdef PUSH_INSERT
        outer[gid] = outer_item;
#endif
      }
    }
  }
}

size_t nestpar_22_visit( ary3_ary_int_tp outer,
                       ary3_ary_int_view_tp outer_vw ) {

  size_t cksum = 0;
  typedef stapl::tuple<size_t,size_t,size_t> gid3_tp;
  ary3_tp::size_type dims = outer.dimensions();
  for (size_t i=0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j=0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k=0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
#ifdef GET_ELEM
        ary_int_tp inner; // = outer[gid];
#else
        ary_int_tp inner = outer[gid];
#endif

        ary_int_tp::iterator inner_it;
        ary_int_tp::gid_type inner_gid;
        for (inner_it = inner.begin();
           inner_it != inner.end(); inner_it++ ) {
#ifdef GET_ELEM
          inner_gid = gid_of(inner_it);
          int inner_item; // = inner.get_element(inner_gid);
#else
          int inner_item = *inner_it;
#endif
          cksum ^= (unsigned int) inner_item;
        }

      }
    }
  }
  return cksum;
}

struct nestpar_22_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_22( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  size_t len0 = (int) log10( (double)outer_cnt );
  size_t len1 = (int) log10( (double)len0 );

  ary3_tp::size_type dims;
#ifdef MULTI_ARRAY
  ary3_ary_int_tp outer(dims);
  ary3_ary_int_view_tp outer_vw(outer);

  nestpar_22_build( model, outer, outer_vw );

  size_t cksum = nestpar_22_visit( outer, outer_vw );

  stapl::rmi_fence();

  ary3_ary_int_tp temp(dims);
  ary3_ary_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_22_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();
#endif

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 23: ary(ary3)
//
// A parallel indexed container class (multiarray)
// nested within other parallel indexed container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - max_element
// modifying - for_each, transform
// mutating - rotate
// numeric - partial_sum
//////////////////////////////////////////////////////////////////////

typedef stapl::multiarray<3,int> ary3_int_tp;
typedef stapl::array< stapl::multiarray<3,int> > ary_ary3_int_tp;
typedef stapl::array_view<ary_ary3_int_tp> ary_ary3_int_view_tp;

void nestpar_23_build( size_t model, ary_ary3_int_tp outer,
                       ary_ary3_int_view_tp outer_vw ) {
  set_random_seed();
  size_t outer_cnt = 1;
  for (size_t i=0; i<model; i++ ) {
    outer_cnt *= 100;
  }

  ary3_tp::size_type dims;
  for (size_t outer_ndx=0; outer_ndx<outer_cnt; ++outer_ndx ) {
#ifdef MULTI_ARRAY
    ary3_int_tp outer_item(dims);
#endif
    size_t inner_cnt = 1 + (rand() % (outer_cnt * 2));
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      int inner_item = prime1000[rand1000_01[inner_ndx%1000]];
#ifdef PUSH_INSERT
      outer_item[inner_ndx] = inner_item;
#endif
    }
#ifdef PUSH_INSERT
    outer[outer_ndx] = outer_item;
#endif
  }
}

size_t nestpar_23_visit( ary_ary3_int_tp outer,
                         ary_ary3_int_view_tp outer_vw ) {
  size_t cksum = 0;

  ary_ary3_int_tp::iterator outer_it;
  ary_ary3_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef MULTI_ARRAY
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    ary3_int_tp((*outer_it).dimensions) outer_item;
    // = outer.get_element(outer_gid);
#else
    ary3_int_tp outer_item = *outer_it;
#endif

    ary3_tp::size_type dims = inner.dimensions();
    typedef stapl::tuple<size_t,size_t,size_t> gid3_tp;
    for (size_t i=0; i < stapl::get<0>(dims); i++ ) {
      for (size_t j=0; j < stapl::get<1>(dims); j++ ) {
        for (size_t k=0; k < stapl::get<2>(dims); k++ ) {
          gid3_tp gid(i,j,k);
          int inner_item = inner[gid];
          cksum ^= (unsigned int) inner_item;
        }
      }
    }
#endif
  }
  return cksum;
}

struct nestpar_23_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_23( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_ary3_int_tp outer;
  ary_ary3_int_view_tp outer_vw(outer);

  nestpar_23_build( model, outer, outer_vw );

  size_t cksum = nestpar_23_visit( outer, outer_vw );

  stapl::rmi_fence();

  ary_ary3_int_tp temp;
  ary_ary3_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_23_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

void read_map(std::multimap<int,int>, const char *);

//////////////////////////////////////////////////////////////////////
// nested parallelism case 24: dygraf(dygraf)
//
// A parallel relational container (dynamic_graph)
// nested within other parallel relational container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - for_each
// modifying - for_each, transform
//////////////////////////////////////////////////////////////////////

typedef stapl::dynamic_graph< stapl::UNDIRECTED, stapl::NONMULTIEDGES,
        int, int> dygraf_int_tp;
typedef stapl::dynamic_graph< stapl::UNDIRECTED, stapl::NONMULTIEDGES,
        stapl::dynamic_graph< stapl::UNDIRECTED, stapl::NONMULTIEDGES,
        int, int>, int> dygraf_dygraf_int_tp;

typedef stapl::graph_view<dygraf_dygraf_int_tp> dygraf_dygraf_int_view_tp;

void nestpar_24_build( size_t model, dygraf_dygraf_int_tp outer,
                       dygraf_dygraf_int_view_tp outer_vw ) {
  set_random_seed();

  std::multimap<int,int> inner_map;
  std::multimap<int,int> outer_map;
  switch( model ) {
  case 1:
    read_map(outer_map, "graf_map1.txt");
    read_map(inner_map, "graf_map0.txt");
    break;
  case 2:
    read_map(outer_map, "graf_map2.txt");
    read_map(inner_map, "graf_map1.txt");
    break;
  case 3:
    read_map(inner_map, "graf_map2.txt");
    read_map(outer_map, "graf_map3.txt");
    break;
  case 4:
    break;
  }

  // build outer vertices
  size_t outer_cnt = outer_map.size();
  for (size_t outer_ndx = 0; outer_ndx < outer_cnt; ++outer_ndx ) {

    // add inner vertices
    size_t inner_cnt = inner_map.size();
    dygraf_int_tp inner_graf;;
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      inner_graf.add_vertex(inner_ndx);
    }

    // add inner edges
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      std::multimap<int,int>::iterator pair_iter;
      for (pair_iter = inner_map.lower_bound(inner_ndx);
           pair_iter != inner_map.upper_bound(inner_ndx); ++pair_iter ) {
        inner_graf.add_edge(pair_iter->first, pair_iter->second);
      }
    }
#ifdef GRAF_VTX
    outer.add_vertex(inner_graf);
#endif
  }

  // add outer edges
  for (size_t outer_ndx = 0; outer_ndx < outer_cnt; ++outer_ndx ) {
    std::multimap<int,int>::iterator pair_iter;
    for (pair_iter = inner_map.lower_bound(outer_ndx);
         pair_iter != inner_map.upper_bound(outer_ndx); ++pair_iter ) {
      outer.add_edge(pair_iter->first, pair_iter->second);
    }
  }
}

size_t nestpar_24_visit( dygraf_dygraf_int_tp outer,
                       dygraf_dygraf_int_view_tp outer_vw ) {

  size_t cksum = 0;
  dygraf_dygraf_int_tp::iterator outer_it;
  dygraf_dygraf_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    dygraf_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    dygraf_int_tp outer_item = *outer_it;
#endif

    dygraf_int_tp::iterator inner_it;
    dygraf_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) inner_item;
    }
  }
  return cksum;
}

struct nestpar_24_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_24( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  dygraf_dygraf_int_tp outer;
  dygraf_dygraf_int_view_tp outer_vw(outer);

  nestpar_24_build( model, outer, outer_vw );

  size_t cksum = nestpar_24_visit( outer, outer_vw );

  stapl::rmi_fence();

  dygraf_dygraf_int_tp temp;
  dygraf_dygraf_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_24_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

//////////////////////////////////////////////////////////////////////
// nested parallelism case 25: digraf(digraf)
//
// A parallel relational container (dynamic_graph)
// nested within other parallel relational container classes.
// Apply one from each of the parallel algorithm classes:
// non-modifying - for_each
// modifying - for_each, transform
//////////////////////////////////////////////////////////////////////

typedef stapl::dynamic_graph< stapl::DIRECTED, stapl::NONMULTIEDGES,
        int, int> digraf_int_tp;
typedef stapl::dynamic_graph< stapl::DIRECTED, stapl::NONMULTIEDGES,
        stapl::dynamic_graph< stapl::DIRECTED, stapl::NONMULTIEDGES,
        int, int>, int> digraf_digraf_int_tp;

typedef stapl::graph_view<digraf_digraf_int_tp> digraf_digraf_int_view_tp;

void nestpar_25_build( size_t model, digraf_digraf_int_tp outer,
                       digraf_digraf_int_view_tp outer_vw ) {
  set_random_seed();

  std::multimap<int,int> inner_map;
  std::multimap<int,int> outer_map;
  switch( model ) {
  case 1:
    read_map(outer_map, "graf_map1.txt");
    read_map(inner_map, "graf_map0.txt");
    break;
  case 2:
    read_map(outer_map, "graf_map2.txt");
    read_map(inner_map, "graf_map1.txt");
    break;
  case 3:
    read_map(outer_map, "graf_map3.txt");
    read_map(inner_map, "graf_map2.txt");
    break;
  case 4:
    break;
  }

  // build outer vertices
  size_t outer_cnt = outer_map.size();
  for (size_t outer_ndx = 0; outer_ndx < outer_cnt; ++outer_ndx ) {

    // add inner vertices
    size_t inner_cnt = inner_map.size();
    digraf_int_tp inner_graf;;
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      inner_graf.add_vertex(inner_ndx);
    }

    // add inner edges
    for (size_t inner_ndx = 0; inner_ndx < inner_cnt; ++inner_ndx ) {
      std::multimap<int,int>::iterator pair_iter;
      for (pair_iter = inner_map.lower_bound(inner_ndx);
           pair_iter != inner_map.upper_bound(inner_ndx); ++pair_iter ) {
        inner_graf.add_edge(pair_iter->first, pair_iter->second);
      }
    }
#ifdef GRAF_VTX
    outer.add_vertex(inner_graf);
#endif
  }

  // add outer edges
  for (size_t outer_ndx = 0; outer_ndx < outer_cnt; ++outer_ndx ) {
    std::multimap<int,int>::iterator pair_iter;
    for (pair_iter = inner_map.lower_bound(outer_ndx);
         pair_iter != inner_map.upper_bound(outer_ndx); ++pair_iter ) {
      outer.add_edge(pair_iter->first, pair_iter->second);
    }
  }
}

size_t nestpar_25_visit( digraf_digraf_int_tp outer,
                         digraf_digraf_int_view_tp outer_vw ) {

  size_t cksum = 0;
  digraf_digraf_int_tp::iterator outer_it;
  digraf_digraf_int_tp::gid_type outer_gid;
  for (outer_it = outer.begin();
       outer_it != outer.end(); outer_it++ ) {
#ifdef GET_ELEM
    outer_gid = gid_of(outer_it);
    digraf_int_tp outer_item; // = outer.get_element(outer_gid);
#else
    digraf_int_tp outer_item = *outer_it;
#endif

    digraf_int_tp::iterator inner_it;
    digraf_int_tp::gid_type inner_gid;
    for (inner_it = outer_item.begin();
       inner_it != outer_item.end(); inner_it++ ) {
#ifdef GET_ELEM
      inner_gid = gid_of(inner_it);
      int inner_item; // = inner.get_element(inner_gid);
#else
      int inner_item = *inner_it;
#endif
      cksum ^= (unsigned int) inner_item;
    }
  }
  return cksum;
}

struct nestpar_25_algo_wf {
#if 1
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 left, View2 result) {
  }
  template <typename View1, typename View2, typename View3>
  void operator()(View1 left, View2 right, View3 result) {
  }
#else
  template <typename Reference>
  void operator()(Reference left, Reference result) {
  }
  template <typename Reference>
  void operator()(Reference left, Reference right, Reference result) {
  }
#endif
};

bool nestpar_25( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  digraf_digraf_int_tp outer;
  digraf_digraf_int_view_tp outer_vw(outer);

  nestpar_25_build( model, outer, outer_vw );

  size_t cksum = nestpar_25_visit( outer, outer_vw );

  stapl::rmi_fence();

  digraf_digraf_int_tp temp;
  digraf_digraf_int_view_tp temp_vw(temp);

  stapl::map_func( nestpar_25_algo_wf(), outer_vw, temp_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
}

