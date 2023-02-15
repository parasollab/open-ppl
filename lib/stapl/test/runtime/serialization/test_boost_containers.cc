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
/// Unit test for Boost containers marshaling.
//////////////////////////////////////////////////////////////////////

#define STAPL_RUNTIME_TEST_MODULE stl_containers
#include "utility.h"
#include <stapl/runtime/serialization.hpp>
#include <stapl/runtime/request/arg_storage.hpp>
#include <boost/optional.hpp>
#include <algorithm>
#include <cstdlib>
#include "test_classes.h"

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/multi_array.hpp>


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


// Sequence containers

BOOST_AUTO_TEST_CASE( boost_array )
{
  using boost::array;

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


BOOST_AUTO_TEST_CASE( boost_multi_array )
{
  // 3D array 3 x 4 x 2
  typedef boost::multi_array<double, 3> array_type;
  array_type a(boost::extents[30][40][20]);

  // Assign values to the elements
  typedef array_type::index index;
  int values = 0;
  for (index i = 0; i != 30; ++i)
    for (index j = 0; j != 40; ++j)
      for (index k = 0; k != 20; ++k)
        a[i][j][k] = values++;
  test_wrapper(a);
}



// Unordered associative containers

BOOST_AUTO_TEST_CASE( boost_unordered_map )
{
  boost::unordered_map<int, bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b[idx] = brand()();
  }
  test_wrapper(b);

  boost::unordered_map<int, int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i[idx] = idx;
  }
  test_wrapper(i);

  boost::unordered_map<int, dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    a[idx] = dynamic_base(idx);
  }
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( boost_unordered_multimap )
{
  boost::unordered_multimap<int, bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( std::make_pair(idx, brand()()) );
    b.insert( std::make_pair(idx, brand()()) );
  }
  test_wrapper(b);

  boost::unordered_multimap<int, int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i.insert( std::make_pair(idx, std::rand()) );
    i.insert( std::make_pair(idx, std::rand()) );
  }
  test_wrapper(i);

  boost::unordered_multimap<int, dynamic_base> a;
  for (unsigned int idx=0; idx<N; ++idx) {
    a.insert( std::make_pair(idx, dynamic_base(std::rand())) );
    a.insert( std::make_pair(idx, dynamic_base(std::rand())) );
  }
  test_wrapper(a);
}


BOOST_AUTO_TEST_CASE( boost_unordered_set )
{
  boost::unordered_set<bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( brand()() );
  }
  test_wrapper(b);

  boost::unordered_set<int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    i.insert( std::rand() );
  }
  test_wrapper(i);
}


BOOST_AUTO_TEST_CASE( boost_unordered_multiset )
{
  boost::unordered_multiset<bool> b;
  for (unsigned int idx=0; idx<N; ++idx) {
    b.insert( brand()() );
    b.insert( brand()() );
  }
  test_wrapper(b);

  boost::unordered_multiset<int> i;
  for (unsigned int idx=0; idx<N; ++idx) {
    int j = std::rand();
    i.insert( j );
    i.insert( j );
  }
  test_wrapper(i);
}
