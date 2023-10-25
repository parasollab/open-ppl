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
/// Unit test for @c std::string marshaling.
//////////////////////////////////////////////////////////////////////

#define STAPL_RUNTIME_TEST_MODULE string
#include "utility.h"
#include <stapl/runtime/request/arg_storage.hpp>
#include <algorithm>
#include <string>
#include <vector>
#include <boost/optional.hpp>

using namespace stapl::runtime;

template<typename T>
void test_wrapper(T const& t)
{
  typedef arg_storage_t<T, T> storage_type;

  boost::optional<T> o = t;

  // find size
  const std::size_t static_size = sizeof(storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );
  BOOST_REQUIRE( *o==t );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  storage_type* const a = new(p) storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );
  BOOST_REQUIRE( *o==t );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)==t );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


struct tester
{
  int                      w;
  const std::string        x;
  char                     j;
  std::string              y[3];
  int                      k;
  std::vector<std::string> z;

  tester(const std::string& str)
  : w(0),
    x(str),
    j(0),
    k(0)
  { }

  bool operator==(tester const& other) const
  {
    return (w==other.w &&
            x==other.x &&
            j==other.j &&
            std::equal(y, y+3, other.y) &&
            k==other.k &&
            z==other.z);
  }

  bool operator!=(tester const& other) const
  { return !(operator==(other)); }

  void define_type(stapl::typer& t)
  {
    t.member(w);
    t.member(x);
    t.member(j);
    t.member(y);
    t.member(k);
    t.member(z);
  }
};


BOOST_AUTO_TEST_CASE( std_string )
{
  test_wrapper(std::string{"STAPL is awesome!!"});
}


BOOST_AUTO_TEST_CASE( complex_string )
{
  const std::string str{"compare string"};
  const std::string str1{"string1"};
  const std::string str2{"this is a longer string2"};
  const std::string str3{"short string"};
  const std::string str4{"yet another longer string"};

  std::vector<std::string> vec;
  vec.push_back(str1);
  vec.push_back(str2);

  tester tst(str);
  tst.w = 5;
  tst.j = 'c';
  tst.y[0] = str2;
  tst.y[1] = str3;
  tst.y[2] = str4;
  tst.k = 3;
  tst.z = vec;

  test_wrapper(tst);
}
