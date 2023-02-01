/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Unit test for C++ STL containers and utility classes marshaling.
//////////////////////////////////////////////////////////////////////

#define STAPL_RUNTIME_TEST_MODULE stl_containers
#include "utility.h"
#include <stapl/runtime/serialization.hpp>
#include <stapl/runtime/request/arg_storage.hpp>
#include <boost/optional.hpp>
#include <algorithm>
#include <cstdlib>
#include "test_classes.h"

#include <array>
#include <deque>
#include <functional>
#include <iterator>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <valarray>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <boost/serialization/array.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>


struct brand
{
  bool operator()(void) const
  { return ( (double(std::rand())/double(RAND_MAX)) > 0.5); }
};

using namespace stapl::runtime;

template<typename T>
void test_wrapper(T const& t)
{
  typedef arg_storage_t<T, T> storage_type;

  boost::optional<T> o = t;
  check_equal(*o, t);

  // find size
  const std::size_t static_size = sizeof(storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );
  check_equal(*o, t);

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  storage_type* const a = new(p) storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );
  check_equal(*o, t);

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  check_equal(a->get(p, size2), t);
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


const unsigned int N = 1000;


// Valarray

BOOST_AUTO_TEST_CASE( std_valarray )
{
  std::valarray<bool> b(N);
  std::generate(&b[0], &b[N], brand());
  test_wrapper(b);

  std::valarray<int> i(N);
  std::generate(&i[0], &i[N], std::rand);
  test_wrapper(i);

  std::valarray<float> f(N);
  std::generate(&f[0], &f[N], std::rand);
  test_wrapper(f);

  std::valarray<double> d(N);
  std::generate(&d[0], &d[N], std::rand);
  test_wrapper(d);

  std::valarray<dynamic_base> a(N);
  std::generate(&a[0], &a[N], std::rand);
  test_wrapper(a);
}


// Sequence containers

BOOST_AUTO_TEST_CASE( std_array )
{
  using std::array;

  array<bool, N> b;
  std::generate(b.begin(), b.end(), brand());
  test_wrapper(b);

  array<int, N> i;
  std::generate(i.begin(), i.end(), std::rand);
  test_wrapper(i);

  array<float, N> f;
  std::generate(f.begin(), f.end(), std::rand);
  test_wrapper(f);

  array<double, N> d;
  std::generate(d.begin(), d.end(), std::rand);
  test_wrapper(d);

  array<dynamic_base, N> a;
  std::generate(a.begin(), a.end(), std::rand);
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( std_deque )
{
  std::deque<bool> b(N);
  std::generate(b.begin(), b.end(), brand());
  test_wrapper(b);

  std::deque<int> i(N);
  std::generate(i.begin(), i.end(), std::rand);
  test_wrapper(i);

  std::deque<float> f(N);
  std::generate(f.begin(), f.end(), std::rand);
  test_wrapper(f);

  std::deque<double> d(N);
  std::generate(d.begin(), d.end(), std::rand);
  test_wrapper(d);

  std::deque<dynamic_base> a(N);
  std::generate(a.begin(), a.end(), std::rand);
  test_wrapper(a);
}


#if 0
#include <forward_list>

BOOST_AUTO_TEST_CASE( std_forward_list )
{
  // TODO -- std::forward_list serialization implementation not yet available
  std::forward_list<bool> b(N);
  std::generate(b.begin(), b.end(), brand());
  test_wrapper(b);

  std::forward_list<int> i(N);
  std::generate(i.begin(), i.end(), std::rand);
  test_wrapper(i);

  std::forward_list<some_type> a(N);
  std::generate(a.begin(), a.end(), std::rand);
  test_wrapper(a);
}


#include <list>
#include <boost/serialization/list.hpp>

BOOST_AUTO_TEST_CASE( std_list )
{
  // TODO -- STAPL packing not correct for std::list
  std::list<bool> b(N);
  std::generate(b.begin(), b.end(), brand());
  test_wrapper(b);

  std::list<int> i(N);
  std::generate(i.begin(), i.end(), std::rand);
  test_wrapper(i);

  std::list<some_type> a(N);
  std::generate(a.begin(), a.end(), std::rand);
  test_wrapper(a);
}
#endif


BOOST_AUTO_TEST_CASE( std_vector )
{
  std::vector<bool> b(N); // special case
  std::generate(b.begin(), b.end(), brand());
  test_wrapper(b);

  std::vector<int> i(N);
  std::generate(i.begin(), i.end(), std::rand);
  test_wrapper(i);

  std::vector<float> f(N);
  std::generate(f.begin(), f.end(), std::rand);
  test_wrapper(f);

  std::vector<double> d(N);
  std::generate(d.begin(), d.end(), std::rand);
  test_wrapper(d);

  std::vector<dynamic_base> a(N);
  std::generate(a.begin(), a.end(), std::rand);
  test_wrapper(a);
}


// Associative containers

BOOST_AUTO_TEST_CASE( std_map )
{
  std::map<int, bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b[idx] = brand()();
  }
  test_wrapper(b);

  std::map<int, int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i[idx] = idx;
  }
  test_wrapper(i);

  std::map<int, dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    a[idx] = dynamic_base(idx);
  }
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( std_multimap )
{
  std::multimap<int, bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( std::make_pair(idx, brand()()) );
    b.insert( std::make_pair(idx, brand()()) );
  }
  test_wrapper(b);

  std::multimap<int, int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i.insert( std::make_pair(idx, std::rand()) );
    i.insert( std::make_pair(idx, std::rand()) );
  }
  test_wrapper(i);

  std::multimap<int, dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    a.insert( std::make_pair(idx, dynamic_base(std::rand())) );
    a.insert( std::make_pair(idx, dynamic_base(std::rand())) );
  }
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( std_set )
{
  std::set<bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( brand()() );
  }
  test_wrapper(b);

  std::set<int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i.insert( std::rand() );
  }
  test_wrapper(i);

  std::set<dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    a.insert( dynamic_base(std::rand()) );
  }
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( std_multiset )
{
  std::multiset<bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( brand()() );
    b.insert( brand()() );
  }
  test_wrapper(b);

  std::multiset<int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    int j = std::rand();
    i.insert( j );
    i.insert( j );
  }
  test_wrapper(i);

  std::multiset<dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    dynamic_base t(std::rand());
    a.insert( t );
    a.insert( t );
  }
  test_wrapper(a);
}



// Unordered associative containers

BOOST_AUTO_TEST_CASE( std_unordered_map )
{
  std::unordered_map<int, bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b[idx] = brand()();
  }
  test_wrapper(b);

  std::unordered_map<int, int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i[idx] = idx;
  }
  test_wrapper(i);

  std::unordered_map<int, dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    a[idx] = dynamic_base(idx);
  }
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( std_unordered_multimap )
{
  std::unordered_multimap<int, bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( std::make_pair(idx, brand()()) );
    b.insert( std::make_pair(idx, brand()()) );
  }
  test_wrapper(b);

  std::unordered_multimap<int, int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i.insert( std::make_pair(idx, std::rand()) );
    i.insert( std::make_pair(idx, std::rand()) );
  }
  test_wrapper(i);

  std::unordered_multimap<int, dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    a.insert( std::make_pair(idx, dynamic_base(std::rand())) );
    a.insert( std::make_pair(idx, dynamic_base(std::rand())) );
  }
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( std_unordered_set )
{
  std::unordered_set<bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( brand()() );
  }
  test_wrapper(b);

  std::unordered_set<int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i.insert( std::rand() );
  }
  test_wrapper(i);
}


BOOST_AUTO_TEST_CASE( std_unordered_multiset )
{
  std::unordered_multiset<bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( brand()() );
    b.insert( brand()() );
  }
  test_wrapper(b);

  std::unordered_multiset<int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    int j = std::rand();
    i.insert( j );
    i.insert( j );
  }
  test_wrapper(i);
}


// Functional

BOOST_AUTO_TEST_CASE( std_reference_wrapper_fundamental )
{
  int i = std::rand();
  test_wrapper(std::cref(i));
  test_wrapper(std::ref(i));

  double d = std::rand();
  test_wrapper(std::cref(d));
  test_wrapper(std::ref(d));
}


BOOST_AUTO_TEST_CASE( std_reference_wrapper_vector )
{
  std::vector<int> v(N);
  std::generate(v.begin(), v.end(), std::rand);
  test_wrapper(std::cref(v));
  test_wrapper(std::ref(v));
}


BOOST_AUTO_TEST_CASE( std_reference_wrapper_map )
{
  std::map<int, dynamic_base> m;
  for (unsigned int idx=0; idx<N; ++idx) {
    m[idx] = dynamic_base(idx);
  }
  test_wrapper(std::cref(m));
  test_wrapper(std::ref(m));
}


// Pair

BOOST_AUTO_TEST_CASE( std_pair )
{
  std::pair<bool, bool> b{brand()(), brand()()};
  test_wrapper(b);

  std::pair<int, int> i(std::rand(), std::rand());
  test_wrapper(i);

  std::pair<float, float> f(std::rand(), std::rand());
  test_wrapper(f);

  std::pair<double, double> d(std::rand(), std::rand());
  test_wrapper(d);

  std::pair<dynamic_base, dynamic_base> a(std::rand(), std::rand());
  test_wrapper(a);
}


// Tuple

BOOST_AUTO_TEST_CASE( std_tuple )
{
  std::tuple<bool, bool, int> b{brand()(), brand()(), 42};
  test_wrapper(b);

  std::tuple<int, int, int> i(std::rand(), std::rand(), 42);
  test_wrapper(i);

  std::tuple<float, float, int> f(std::rand(), std::rand(), 42);
  test_wrapper(f);

  std::tuple<double, double, int> d(std::rand(), std::rand(), 42);
  test_wrapper(d);

  std::tuple<dynamic_object, dynamic_object, int> a(std::rand(),
                                                    std::rand(),
                                                    42);
  test_wrapper(a);

  std::tuple<std::vector<int>, std::vector<double>> v({1, 2, 3},
                                                      {42.0, 43.1, 44.2});
  test_wrapper(v);
}
