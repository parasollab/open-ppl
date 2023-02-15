/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>

#include <stapl/containers/unordered_set/unordered_set.hpp>

#include <stapl/skeletons/serial.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/runtime.hpp>

#include <stapl/stream.hpp>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

using namespace std;
#include "testutil.hpp"
#include "rel_alpha_data.h"

// #define BIG_FILES // Uncomment when big and huge files are present
// #define TRIPLE_MAP_REDUCE // Uncomment when the bug is fixed

// Disable some tests by commenting the defines below:
// 2-lvl tests:
#define TEST_SET_2LVL_01
#define TEST_SET_2LVL_02
#define TEST_SET_2LVL_03
// #define TEST_SET_2LVL_04 // COMPILES BUT DOESN'T RUN : MAP BUG
#define TEST_SET_2LVL_05
#define TEST_SET_2LVL_06

// 3-lvl tests:
#define TEST_SET_3LVL_01
// #define TEST_SET_3LVL_02 // #ifdef TRIPLE_MAP_REDUCE
// #define TEST_SET_3LVL_03 // #ifdef TRIPLE_MAP_REDUCE
// #define TEST_SET_3LVL_04 // #ifdef TRIPLE_MAP_REDUCE
// #define TEST_SET_3LVL_05 // MAP BUG
// #define TEST_SET_3LVL_06 // MAP BUG


/*=========================================================================*/
#ifdef TEST_SET_2LVL_01
size_t nestpar_unordered_set_01(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_2LVL_02
size_t nestpar_unordered_set_02(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_2LVL_03
size_t nestpar_unordered_set_03(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_2LVL_04
size_t nestpar_unordered_set_04(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_2LVL_05
size_t nestpar_unordered_set_05(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_2LVL_06
size_t nestpar_unordered_set_06(size_t,
                                stapl::stream<ifstream>&,
                                stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_3LVL_01
size_t nestpar_unordered_set_10(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_3LVL_02
size_t nestpar_unordered_set_11(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_3LVL_03
size_t nestpar_unordered_set_12(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_3LVL_04
size_t nestpar_unordered_set_13(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_3LVL_05
size_t nestpar_unordered_set_14(size_t, stapl::stream<ofstream>& );
#endif
#ifdef TEST_SET_3LVL_06
size_t nestpar_unordered_set_15(size_t, stapl::stream<ofstream>& );
#endif

/*=========================================================================*/

extern int prime100k[];
extern int rand100k[];

int fibo[20] = { 1, 2, 3, 5, 8,
                 13, 21, 34, 55, 89,
                 144, 233, 377, 610, 987,
                 1597, 2584, 4181, 6765, 10946 };

/*=========================================================================*/

typedef stapl::unordered_set<int> set_int_tp;


// -- 2-lvl set : --
typedef stapl::vector< set_int_tp > vec_set_int_tp;
typedef stapl::vector_view<vec_set_int_tp> vec_set_int_vw_tp;

typedef stapl::array< set_int_tp > ary_set_int_tp;
typedef stapl::array_view<ary_set_int_tp> ary_set_int_vw_tp;

typedef stapl::map< int, set_int_tp > map_set_int_tp;
typedef stapl::map_view<map_set_int_tp> map_set_int_vw_tp;
// 3 level :
  // vec<vec<set>>
typedef stapl::vector<vec_set_int_tp> vec_vec_set_int_tp;
typedef stapl::vector_view<vec_vec_set_int_tp> vec_vec_set_int_vw_tp;

  // ary<vec<set>>
typedef stapl::array<vec_set_int_tp> ary_vec_set_int_tp;
typedef stapl::array_view<ary_vec_set_int_tp> ary_vec_set_int_vw_tp;

  // vec<ary<set>>
typedef stapl::vector<ary_set_int_tp> vec_ary_set_int_tp;
typedef stapl::vector_view<vec_ary_set_int_tp> vec_ary_set_int_vw_tp;

  // vec<map<set>>
typedef stapl::vector<map_set_int_tp> vec_map_set_int_tp;
typedef stapl::vector_view<vec_map_set_int_tp> vec_map_set_int_vw_tp;

  // map<ary<set>>
typedef stapl::map< int, ary_set_int_tp > map_ary_set_int_tp;
typedef stapl::map_view<map_ary_set_int_tp> map_ary_set_int_vw_tp;

// -----------------

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;

/*=========================================================================*/

typedef stapl::identity<int> id_int_wf;
typedef stapl::identity<size_t> id_un_wf;
typedef stapl::negate<int> neg_int_wf;

typedef stapl::plus<int> add_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::min<int> min_int_wf;
typedef stapl::max<int> max_int_wf;

typedef stapl::bit_xor<size_t> xor_un_wf;
typedef stapl::bit_or<size_t> ior_un_wf;
typedef stapl::bit_and<size_t> and_un_wf;


/*=========================================================================*/

void open_zin(int model, int test, stapl::stream<ifstream>& zin)
{
  switch ( model ) {
  case 1:
    switch( test ) {
    case 1:
      zin.open("tiny_factors.zin");
      break;
    default:
      zin.open("tiny_primes.zin");
      break;
    }
    break;
  case 100:
    switch( test ) {
      case 1:
      zin.open("data/small_factors.zin");
      break;
    default:
      zin.open("data/small_primes.zin");
      break;
    }
    break;
  case 10000:
    switch( test ) {
      case 1:
      zin.open("data/medium_factors.zin");
      break;
    default:
      zin.open("data/medium_primes.zin");
      break;
    }
    break;
  #ifdef BIG_FILES
  case 1000000:
    switch( test ) {
    case 1:
      zin.open("data/big_factors.zin");
      break;
    default:
      zin.open("data/big_primes.zin");
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 1:
      zin.open("data/huge_factors.zin");
      break;
    default:
      zin.open("data/huge_primes.zin");
      break;
    }
    break;
  #endif
  }
}


struct put_val_wf
{
private:
  // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  put_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Ref>
  void operator()(Ref val)
  {
    m_zout << val << " ";
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }

};

struct nestpar_cksum_wf
{
  typedef size_t result_type;

  template<typename Ref1>
  size_t operator()(Ref1 v1)
  {
    return stapl::map_reduce(id_un_wf(), xor_un_wf(), v1);
  }
};

struct map_set_cksum_wf
{
  typedef int result_type;

  template<typename Element>
  int operator()(Element elem) const
  {
    return stapl::map_reduce(id_int_wf(), xor_un_wf(), elem.second);
  }
};

struct map_outer_3lvl_cksum_wf
{
  typedef int result_type;

  template<typename Element>
  int operator()(Element elem) const
  {
    return stapl::map_reduce(nestpar_cksum_wf(), xor_un_wf(), elem.second);
  }
};

struct map_middle_3lvl_cksum_wf
{
  typedef int result_type;

  template<typename Element>
  int operator()(Element elem) const
  {
    return stapl::map_reduce(map_set_cksum_wf(), xor_un_wf(), elem);
  }
};


struct inner_set_show_wf
{
private:
 // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  inner_set_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template<typename Element>
  void operator()(Element elem)
  {
    //FIXME, ss shouldn't be necessary
    stringstream ss;
    ss  << "{" << elem << "}" << endl;
    m_zout << ss.str();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }

};

struct outer_map_show_inn_set_wf
{
private:
 // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  outer_map_show_inn_set_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }


  template<typename Element>
  void operator()(Element elem)
  {
    stapl::serial_io(inner_set_show_wf(m_zout), elem.second);
  }


  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }

};


struct roll_wf
{
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 length, View2 & limit) const
  {
    length = limit;
  }
};

/*=========================================================================*/

int opt_test = -1;
char *opt_data = 0;

stapl::exit_code stapl_main(int argc, char **argv)
{

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  char *temp = 0;
  for ( int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ('-' == opt[0] ) {
      switch ( opt[1] ) {
      case 'h':
        cerr << "Specify the data size with -d $X \n"
        << "With $X = "<< endl
        << " t for tiny" << endl
        << " s for small" << endl
        << " m for medium" << endl;
        break;
      case 'd':
        opt_data = argv[argi++];
        break;
      }
    } else {
      cerr << "unknown command line argument " << opt << endl;
    }
  }

  int model = -1;
  switch ( opt_data[0] ) {
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
    std::cerr << "usage: exe -d [t|s|m]\n";
    exit(1);
  }

  int first_test = 1;
  int last_test = 36;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  bool ok = true;
  first_test = 1; last_test = 12;
  if ( stapl::get_location_id() == 0 )
        std::cout << "Nested Parallel Set " << endl;
  for ( int test=first_test; test<=last_test; test++ ) {
    if ( stapl::get_location_id() == 0 )
      std::cerr << "Test #" << test << ": ";
    size_t result = 0;
    bool disabled = false;
    switch ( test) {
    case 1:
      #ifdef TEST_SET_2LVL_01
      result = nestpar_unordered_set_01(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 2:
      #ifdef TEST_SET_2LVL_02
      result = nestpar_unordered_set_02(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 3:
      #ifdef TEST_SET_2LVL_03
      result = nestpar_unordered_set_03(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 4:
      //#ifdef TEST_SET_2LVL_04
      //result = nestpar_unordered_set_04(model, zout);
      //#else
      disabled = true;
      //#endif
      break;
    case 5:
      #ifdef TEST_SET_2LVL_05
      result = nestpar_unordered_set_05(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 6:
      #ifdef TEST_SET_2LVL_06
      result = nestpar_unordered_set_06(model, zin, zout);
      #else
      disabled = true;
      #endif
      break;
    case 7:
      #ifdef TEST_SET_3LVL_01
      result = nestpar_unordered_set_10(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 8:
      #ifdef TEST_SET_3LVL_02
      result = nestpar_unordered_set_11(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 9:
      #ifdef TEST_SET_3LVL_03
      result = nestpar_unordered_set_12(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 10:
      #ifdef TEST_SET_3LVL_04
      result = nestpar_unordered_set_13(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 11:
      #ifdef TEST_SET_3LVL_05
      result = nestpar_unordered_set_14(model, zout);
      #else
      disabled = true;
      #endif
      break;
    case 12:
      #ifdef TEST_SET_3LVL_06
      result = nestpar_unordered_set_15(model, zout);
      #else
      disabled = true;
      #endif
      break;

    default:
      std::cerr << endl
                << "-- test "
                << test
                << " not yet implemented --"
                << endl;
      break;

    }

    bool passed;
    passed = (result == 0);

    if ( stapl::get_location_id() == 0) {
      if (disabled)
        std::cerr << "Disabled" << endl;
      else
      {
        if (passed)
          std::cerr << "[PASSED] " << endl;
        else
          std::cerr << "[FAILED] " << endl;
      }
    }
  }
  return EXIT_SUCCESS;
}

/*=========================================================================
 * heterogeneous nested structures
 *=========================================================================*/

// --------------
// ---- Fill ----
// --------------

struct insert_wf
{

  typedef void result_type;
  size_t m_size;

  insert_wf(size_t sz)
    : m_size(sz)
  { }


  template <typename T,typename View>
  void operator()(T i,View& vw)
  {
    if ( m_size <= 100000 )
      vw.insert(rand_nums[i]);
    else
    {
      size_t k = i % 10000;
      vw.insert( prime_nums[k] * prime_nums[10000-k] );
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }

};


struct nestpar_inner_set_fill_wf
{

  typedef void result_type;

  size_t m_size;

  nestpar_inner_set_fill_wf(size_t sz)
    : m_size(sz)
  { }

  template <typename View1>
  void operator()(View1 vw1) //FIXME: View1 & vw1 should work.
  {
    insert_wf insertwf(m_size);
    stapl::map_func(insertwf,
                    stapl::counting_view<size_t>(m_size),
                    stapl::make_repeat_view(vw1));

  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }

};

struct nestpar_inner_set_fill_fromfile_wf
{
private:
  size_t m_size;
  // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ifstream> m_zin;
public:
  typedef void result_type;

  nestpar_inner_set_fill_fromfile_wf(
    size_t sz,stapl::stream<ifstream> const& zin)
    : m_size(sz),m_zin(zin)
  { }

  template <typename ViewOverSet>
  void operator()(ViewOverSet vw_set)//FIXME: ViewOverSet & vw_set should work.
  {
    typename ViewOverSet::value_type t;
    for ( size_t i = 0; i < m_size; i++ ) {
      m_zin >> t;
      vw_set.insert(t);
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
    t.member(m_zin);
  }

};
// --------------
// ---- Show ----
// --------------

struct nestpar_inner_set_show_wf
{
private:
  // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  nestpar_inner_set_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename View1>
  void operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }

};

struct nestpar_outer_cksum_wf
{
  typedef size_t result_type;

  template<typename Ref1>
  size_t operator()(Ref1 v1)
  {
    return stapl::map_reduce(nestpar_cksum_wf(), xor_un_wf(), v1);
  }
};

// -------------
// -- Process --
// -------------
struct nestpar_size_process_wf
{
  typedef long int result_type;

  template <typename View1>
  long int operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_map_reduce_process_wf
{
  typedef long int result_type;

  template <typename View1>
  long int operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1);
  }
};


template<typename NestedContainer,
         typename NestedContainerView,
         typename FillWF = nestpar_inner_set_fill_wf >
class two_lvl_test_wrapper
{
public:

  template <typename Filename,typename ProcessWF>
  size_t run_test(size_t model,
                  Filename output_filename,
                  ProcessWF & proc_wf,
                  stapl::stream<ofstream>& zout)
  {

    size_t size = 100 * model;
    size_t limit = 100 * model;
    set_random_seed();
    zout.open(output_filename);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();

    NestedContainer a(size);
    NestedContainerView a_vw(a);

    stapl::map_func(FillWF(model*2), a_vw );

    stapl::map_func(proc_wf, a_vw );

    stapl::serial_io(nestpar_inner_set_show_wf(zout), a_vw );

    ctr.stop();
    double time1 = ctr.value();

    zout.close();

    size_t result = stapl::map_reduce(nestpar_cksum_wf(), xor_un_wf(), a_vw);
    return result;
  }

  template <typename Filename,typename ProcessWF>
  size_t run_test(size_t model,
                  Filename output_filename,
                  ProcessWF & proc_wf,
                  stapl::stream<ifstream>& zin,
                  stapl::stream<ofstream>& zout)
  {

    size_t size = 100 * model;
    size_t limit = 100 * model;
    set_random_seed();
    zout.open(output_filename);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();

    NestedContainer a(size);
    NestedContainerView a_vw(a);

    stapl::map_func(FillWF(model*2,zin), a_vw );

    stapl::map_func(proc_wf, a_vw );

    stapl::serial_io(nestpar_inner_set_show_wf(zout), a_vw );

    ctr.stop();
    double time1 = ctr.value();

    zout.close();

    size_t result = stapl::map_reduce(nestpar_cksum_wf(), xor_un_wf(), a_vw);
    return result;
  }

};


template<typename NestedContainer, typename NestedContainerView>
class two_lvl_test_wrapper_mapreduce
{
public:

  template <typename Filename,typename ProcessWF>
  size_t run_test(size_t model,
                  Filename output_filename,
                  ProcessWF & proc_wf,
                  stapl::stream<ofstream>& zout)
  {

    size_t size = 100 * model;
    size_t limit = 100 * model;
    set_random_seed();
    zout.open(output_filename);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();

    NestedContainer a(size);
    NestedContainerView a_vw(a);

    stapl::map_func(nestpar_inner_set_fill_wf(model*2), a_vw  );

    add_int_wf::result_type res=stapl::map_reduce( proc_wf, add_int_wf(), a_vw);
    if (stapl::get_location_id() == 0)
    {
      //FIXME, ss shouldn't be necessary
      stringstream ss;
      ss << "map_reduce result (accumulate all elements from all sets):"
        << res << endl;
      zout << ss.str();
    }

    stapl::serial_io(nestpar_inner_set_show_wf(zout), a_vw );

    ctr.stop();
    double time1 = ctr.value();

    zout.close();

    return stapl::map_reduce(nestpar_cksum_wf(), xor_un_wf(), a_vw);
  }

};
// --------------------------
// - 3-level-deep functions -
// --------------------------

// --------------
// ---- Fill ----
// --------------

template <typename FillWF>
struct nestpar_outer_fill_wf
{

  typedef void result_type;

  size_t m_size;

  nestpar_outer_fill_wf(size_t sz)
    : m_size(sz)
  { }

  template <typename View1>
  void operator()(View1 const& vw1)
  {
    stapl::map_func(FillWF(m_size), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }

};

struct middle_insert_wf
{

  typedef void result_type;
  size_t m_size;

  middle_insert_wf(size_t sz)
    : m_size(sz)
  { }

  template <typename T,typename View>
  void operator()(T i,View& vw)
  {
    if ( m_size <= 100000 ) {
      for ( size_t j = 0; j < m_size; j++ ) {
        vw[ prime_nums[i] ].insert(rand_nums[j]);
      }
    } else {
      for ( size_t j = 0; j < m_size; j+=10000 ) {
        for ( size_t k = 0; k < 10000; k++ )
          vw[ prime_nums[i%10000] * prime_nums[10000-(i%10000)] ].insert(
            rand_nums[k] * rand_nums[10000-k]);
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }

};


struct nestpar_middlemap_set_fill_wf
{

  typedef void result_type;

  size_t m_size;

  nestpar_middlemap_set_fill_wf(size_t sz)
    : m_size(sz)
  { }

  template <typename View1>
  void operator()(View1 const& map_set_vw1)
  {
    middle_insert_wf insertwf(m_size);
    stapl::map_func(insertwf,
                    stapl::counting_view<size_t>(m_size),
                    stapl::make_repeat_view(map_set_vw1));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }

};

// --------------
// ---- Show ----
// --------------

struct nestpar_outer_show_wf
{

private:
  // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  nestpar_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename View1>
  void operator()(View1 const& vw1)
  {
    stapl::serial_io(nestpar_inner_set_show_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct outer_map_3lvl_show_inn_set_wf
{

private:
 // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  outer_map_3lvl_show_inn_set_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template<typename Element>
  void operator()(Element elem)
  {
    stapl::serial_io(nestpar_inner_set_show_wf(m_zout), elem.second);
  }

  void define_type(stapl::typer& t)
  {
   t.member(m_zout);
  }
};


struct nestpar_middle_map_show_wf
{
private:
 // pointer to p_object is necessary to allow work function serialization.
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  nestpar_middle_map_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename View1>
  void operator()(View1 const& vw1)
  {
    stapl::serial_io(outer_map_show_inn_set_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// -------------
// -- Process --
// -------------
template <typename InnerProcWF>
struct nestpar_outer_process_wf
{
  typedef void result_type;

  InnerProcWF m_proc;
  nestpar_outer_process_wf(InnerProcWF proc)
    : m_proc(proc)
  { }

  template <typename View1>
  void operator()(View1 const& vw1)
  {
    stapl::map_func(m_proc, vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_proc);
  }

};


struct nestpar_outerMap_process_wf
{
  typedef void result_type;

  template <typename View1>
  void operator()(View1 const& vw1)
  {
    stapl::map_func(nestpar_map_reduce_process_wf(), vw1.second);
  }
};

struct nestpar_middle_map_reduce_process_wf
{
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    return stapl::map_reduce(nestpar_map_reduce_process_wf(),
                             add_int_wf(),
                             vw1);
  }
};

struct nestpar_3lvl_cksum_wf
{
  typedef size_t result_type;

  template<typename Ref1>
  size_t operator()(Ref1 v1)
  {
    return stapl::map_reduce(nestpar_outer_cksum_wf(), xor_un_wf(), v1);
  }
};


template<typename NestedContainer,
         typename NestedContainerView,
         typename FillWF = nestpar_inner_set_fill_wf>
class three_lvl_test_wrapper
{
public:

  template <typename Filename,typename ProcessWF>
  size_t run_test(size_t model,
                  Filename output_filename,
                  ProcessWF & proc_wf,
                  stapl::stream<ofstream>& zout)
  {

    size_t size = 100 * model;
    size_t limit = 100 * model;
    set_random_seed();
    zout.open(output_filename);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    //Array which specifies the size of the 2nd-lvl container:
    ary_sz_tp len_1(model * 10);
    ary_sz_vw_tp len_1_vw(len_1);

    stapl::map_func(roll_wf(), len_1_vw, stapl::make_repeat_view(limit));

    NestedContainer a(len_1_vw);
    NestedContainerView a_vw(a);

    stapl::map_func(nestpar_outer_fill_wf<FillWF>(model*2), a_vw );

    stapl::map_func(nestpar_outer_process_wf<ProcessWF>(proc_wf), a_vw );

    stapl::serial_io(nestpar_outer_show_wf(zout), a_vw);

    ctr.stop();
    double time1 = ctr.value();

    zout.close();
    #ifdef TRIPLE_MAP_REDUCE
    size_t cksum = stapl::map_reduce(nestpar_3lvl_cksum_wf(),
                                     xor_un_wf(),
                                     a_vw);
    return cksum;
    #else
    return 0;
    #endif
  }

};

template<typename NestedContainer,
         typename NestedContainerView,
         typename FillWF = nestpar_inner_set_fill_wf>
class three_lvl_test_wrapper_mapreduce
{
public:

  template <typename Filename,typename ProcessWF>
  size_t run_test(size_t model,
                  Filename output_filename,
                  ProcessWF & proc_wf,
                  stapl::stream<ofstream>& zout)
  {

    size_t size = 100 * model;
    size_t limit = 100 * model;
    set_random_seed();
    zout.open(output_filename);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    //Array which specifies the size of the 2nd-lvl container:
    ary_sz_tp len_1(model * 10);
    ary_sz_vw_tp len_1_vw(len_1);

    stapl::map_func(roll_wf(), len_1_vw, stapl::make_repeat_view(limit));

    NestedContainer a(len_1_vw);
    NestedContainerView a_vw(a);

    stapl::map_func(nestpar_outer_fill_wf<FillWF>(model*2), a_vw );

    add_int_wf::result_type res = stapl::map_reduce(
      proc_wf,
      add_int_wf(),
      a_vw);

    stringstream ss;
    ss << "map_reduce result (accumulate all elements from all sets):"
      << res << endl;
    zout << ss.str();


    stapl::serial_io(nestpar_outer_show_wf(zout), a_vw);

    ctr.stop();
    double time1 = ctr.value();

    zout.close();
    #ifdef TRIPLE_MAP_REDUCE
    size_t cksum = stapl::map_reduce(nestpar_3lvl_cksum_wf(),
                                     xor_un_wf(),
                                     a_vw);
    return cksum;
    #else
    return 0;
    #endif
  }

};



//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(unordered_set(int))
// construct vector with generator, traverse with serial
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_2LVL_01

size_t nestpar_unordered_set_01( size_t model, stapl::stream<ofstream>& zout)
{
  two_lvl_test_wrapper <vec_set_int_tp, vec_set_int_vw_tp> test_case;
  nestpar_size_process_wf proc;

  return test_case.run_test(model, "np_unordered_set_01.zout", proc, zout);
}

#endif
//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(unordered_set(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_2LVL_02

size_t nestpar_unordered_set_02( size_t model, stapl::stream<ofstream>& zout)
{
  two_lvl_test_wrapper <vec_set_int_tp, vec_set_int_vw_tp> test_case;
  nestpar_map_reduce_process_wf proc;
  return test_case.run_test(model, "np_unordered_set_02.zout", proc, zout);
}

#endif
//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(unordered_set(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_2LVL_03
size_t nestpar_unordered_set_03( size_t model, stapl::stream<ofstream>& zout)
{
  two_lvl_test_wrapper <ary_set_int_tp, ary_set_int_vw_tp> test_case;
  nestpar_map_reduce_process_wf proc;
  return test_case.run_test(model, "np_unordered_set_03.zout", proc, zout);
}
#endif
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(unordered_set(int))
// construct map data from C arrays, traverse with mapreduce
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_2LVL_04
struct nestpar_set_04_process_wf
{
  typedef int result_type;

  template <typename Element>
  int operator()(Element elem)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_unordered_set_04( size_t model, stapl::stream<ofstream>& zout)
{

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();
  zout.open("np_unordered_set_04.zout");

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));


  ndx_dom_tp map_dom(0, 100000);
  map_set_int_tp a(map_dom);
  map_set_int_vw_tp a_vw(a);

  if ( stapl::get_location_id() == 0 ) {
    if ( size <= 100000 ) {
      for ( size_t i = 0; i < size; i++ ) {
        for ( size_t j = 0; j < limit; j++ ) {
          a[ prime_nums[i] ].insert(rand_nums[j]);
        }
      }
    } else {
      for ( size_t i = 0; i < size; i += 10000 ) {
        for ( size_t k = 0; k < 10000; k++ ) {
          for ( size_t jj = 0; jj < limit; jj += 10000 )
            for ( size_t j = 0; j < 10000; j++ ) {
              a[ prime_nums[k] * prime_nums[10000-k] ].insert(
                rand_nums[j] * rand_nums[10000-j]);
          }
        }
      }
    }
  }
  stapl::rmi_fence();

  stapl::map_func(nestpar_set_04_process_wf(), a_vw );

  stapl::serial_io(outer_map_show_inn_set_wf(zout), a_vw );

  ctr.stop();
  double time1 = ctr.value();

  zout.close();

  return stapl::map_reduce(map_set_cksum_wf(), xor_un_wf(), a_vw);
}
#endif
//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(unordered_set(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_2LVL_05
struct nestpar_set_05_process_wf
{
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    return stapl::accumulate(vw1, -2000000);
  }
};

size_t nestpar_unordered_set_05( size_t model, stapl::stream<ofstream>& zout)
{
  two_lvl_test_wrapper_mapreduce <vec_set_int_tp, vec_set_int_vw_tp> test_case;
  nestpar_set_05_process_wf proc;

  return test_case.run_test(model, "np_unordered_set_05.zout", proc, zout);
}
#endif

#ifdef TEST_SET_2LVL_06


//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
// construct vector with serial I/O,
// process with map_reduce / work function, display with counting view
//////////////////////////////////////////////////////////////////////

size_t nestpar_unordered_set_06( size_t model, stapl::stream<ifstream>& zin,
                       stapl::stream<ofstream>& zout)
{
  open_zin(model,2,zin);
  two_lvl_test_wrapper <vec_set_int_tp,
                        vec_set_int_vw_tp,
                        nestpar_inner_set_fill_fromfile_wf> test_case;
  nestpar_map_reduce_process_wf proc;
  return test_case.run_test(model, "np_unordered_set_06.zout", proc, zin, zout);
}
#endif


//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(unordered_set(int))
// construct array with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

#ifdef TEST_SET_3LVL_01
size_t nestpar_unordered_set_10( size_t model, stapl::stream<ofstream>& zout)
{
  three_lvl_test_wrapper <vec_vec_set_int_tp, vec_vec_set_int_vw_tp> test_case;
  nestpar_size_process_wf proc;
  return test_case.run_test(model, "np_unordered_set_10.zout", proc, zout );
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(unordered_set(int))
// construct vector with serial I/O,
// process with stapl algorithm, display with serial
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_3LVL_02

struct nestpar_set_11_process_wf
{
  typedef long int result_type;

  template <typename View1>
  long int operator()(View1 const &vw1)
  {
    return stapl::accumulate(vw1, -2000000);
  }
};

struct nestpar_set_11_middle_process_wf
{
  typedef long int result_type;

  template <typename View1>
  long int operator()(View1 const &vw1)
  {
    return stapl::map_reduce(nestpar_set_11_process_wf(),
                             add_int_wf(),
                             vw1);
  }
};

size_t nestpar_unordered_set_11( size_t model, stapl::stream<ofstream>& zout)
{
  three_lvl_test_wrapper_mapreduce <vec_vec_set_int_tp,
                                    vec_vec_set_int_vw_tp> test_case;
  nestpar_set_11_middle_process_wf proc;
  return test_case.run_test(model, "np_unordered_set_11.zout", proc, zout );
}

#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism of various containers
// construct vector with serial I/O,
// process with map_reduce, display with serial
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// vec<ary<unordered_set <int> > >
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_3LVL_03
size_t nestpar_unordered_set_12( size_t model, stapl::stream<ofstream>& zout )
{
  three_lvl_test_wrapper <vec_ary_set_int_tp, vec_ary_set_int_vw_tp> test_case;
  nestpar_map_reduce_process_wf proc;
  return test_case.run_test(model, "np_unordered_set_12.zout", proc, zout );
}
#endif

//////////////////////////////////////////////////////////////////////
// ary<vec<unordered_set <int> > >
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_3LVL_04
size_t nestpar_unordered_set_13( size_t model, stapl::stream<ofstream>& zout )
{
  three_lvl_test_wrapper <ary_vec_set_int_tp, ary_vec_set_int_vw_tp> test_case;
  nestpar_map_reduce_process_wf proc;
  return test_case.run_test(model, "np_unordered_set_13.zout", proc, zout );
}
#endif

//////////////////////////////////////////////////////////////////////
// vec<map<unordered_set <int> > >
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_3LVL_05

struct nestpar_set_14_process_wf
{
  typedef int result_type;

  template <typename Element>
  int operator()(Element elem)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), elem.second);
  }
};


size_t nestpar_unordered_set_14( size_t model, stapl::stream<ofstream>& zout )
{

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();
  zout.open("np_unordered_set_14.zout");

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_map_set_int_tp a(model * 10);
  vec_map_set_int_vw_tp a_vw(a);

  stapl::map_func(nestpar_middlemap_set_fill_wf(model*2), a_vw );

  nestpar_set_14_process_wf proc_wf;
  stapl::map_func(nestpar_outer_process_wf<nestpar_set_14_process_wf>(proc_wf),
                  a_vw );

  stapl::serial_io(nestpar_middle_map_show_wf(zout), a_vw);

  ctr.stop();
  double time1 = ctr.value();

  zout.close();

  return  stapl::map_reduce(map_middle_3lvl_cksum_wf(), xor_un_wf(), a_vw);

}
#endif

//////////////////////////////////////////////////////////////////////
// map<ary<unordered_set <int> > >
//////////////////////////////////////////////////////////////////////
#ifdef TEST_SET_3LVL_06

size_t nestpar_unordered_set_15( size_t model, stapl::stream<ofstream>& zout )
{

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();
  zout.open("np_unordered_set_15.zout");

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();


  ndx_dom_tp map_dom(0, 16277216);
  map_ary_set_int_tp a(map_dom);
  map_ary_set_int_vw_tp a_vw(a);

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  if ( stapl::get_location_id() == 0 ) {
    if ( size <= 100000 ) {
      for ( size_t i = 0; i < size; i++ ) {
        a[ prime_nums[i]].resize(len_vw[i]);
      }
      for ( size_t i = 0; i < size; i++ ) {
        for ( size_t j = 0; j < len_vw[i]; j++ ) {
          int val = rand_nums[j];
          a[ prime_nums[i] ][ val ].insert(rand_nums[j]);
        }
      }
    } else {
      for ( size_t i = 0; i < size; i += 10000 ) {
        a[ prime_nums[i] ].resize(len_vw[i]);
      }
      for ( size_t i = 0; i < size; i += 10000 ) {
        for ( size_t j = 0; j < len_vw[i]; j++ ) {
          for ( size_t k = 0; k < 10000; k++ ) {
            int val = rand_nums[j];
            a[ prime_nums[k] * prime_nums[10000-k] ][ val ].insert(
              rand_nums[j]);
          }
        }
      }
    }
  }
  stapl::rmi_fence();

  stapl::map_func(nestpar_outerMap_process_wf(), a_vw );
  stapl::serial_io(outer_map_3lvl_show_inn_set_wf(zout), a_vw);

  ctr.stop();
  double time1 = ctr.value();

  zout.close();

  return stapl::map_reduce(map_outer_3lvl_cksum_wf(), xor_un_wf(), a_vw);
}


#endif
