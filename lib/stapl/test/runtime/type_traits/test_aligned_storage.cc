/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE aligned_storage
#include "utility.h"
#include <stapl/runtime/type_traits/aligned_storage.hpp>
#include <boost/mpl/range_c.hpp>
#include <boost/mpl/for_each.hpp>

// Returns the size of an object aligned to the word size.
#define THIS_ALIGNMENT \
  STAPL_RUNTIME_DEFAULT_ALIGNMENT
#define ALIGNED_SIZEOF(type) \
  (sizeof(type)+THIS_ALIGNMENT-((sizeof(type)-1)%THIS_ALIGNMENT + 1))
#define ALIGNED_SIZE(num) \
  ((num)+THIS_ALIGNMENT-(((num)-1)%THIS_ALIGNMENT + 1))

template<std::size_t Size>
struct array
{
  char data[Size];
};

template<>
struct array<0>
{ };

using stapl::runtime::aligned_storage_t;
using stapl::runtime::aligned_size;

struct test_wrapper
{
  template<typename T>
  void operator()(T&) const
  {
    typedef array<T::value>                       array_type;
    typedef aligned_storage_t<sizeof(array_type)> aligned_type;

    const std::size_t size = T::value;
    BOOST_CHECK_EQUAL( size, sizeof(array_type) );

    // test aligned_storage<T>
    const std::size_t aligned_type_size = sizeof(aligned_type);
    BOOST_CHECK_EQUAL( (aligned_type_size%THIS_ALIGNMENT), std::size_t(0) );
    BOOST_CHECK( aligned_type_size>=sizeof(array_type) );
    BOOST_CHECK_EQUAL( aligned_type_size, (ALIGNED_SIZE(sizeof(array_type))) );
    BOOST_CHECK_EQUAL( aligned_type_size, (ALIGNED_SIZEOF(array_type)) );
    if (size%THIS_ALIGNMENT==std::size_t(0)) {
      BOOST_CHECK_EQUAL( aligned_type_size, size );
    }

    // test aligned_size()
    const std::size_t fsz = aligned_size(sizeof(array_type));
    BOOST_CHECK_EQUAL( (fsz%THIS_ALIGNMENT), std::size_t(0) );
    BOOST_CHECK( fsz>=sizeof(array_type) );
    BOOST_CHECK_EQUAL( fsz, (ALIGNED_SIZE(sizeof(array_type))) );
    BOOST_CHECK_EQUAL( fsz, (ALIGNED_SIZEOF(array_type)) );
    if (size%THIS_ALIGNMENT==0) {
      BOOST_CHECK_EQUAL( fsz, size );
    }

    // compare aligned_storage_t with aligned_size()
    BOOST_CHECK_EQUAL(sizeof(aligned_type), aligned_size(sizeof(array_type)));
  }
};


BOOST_AUTO_TEST_CASE( test_aligned_storage )
{
  using namespace boost::mpl;

  typedef range_c<std::size_t, 1, 129> range;
  for_each<range>( test_wrapper() );
}


BOOST_AUTO_TEST_CASE( test_aligned_storage_size_t )
{
  typedef std::size_t T;

  BOOST_CHECK( sizeof(T)<=sizeof(aligned_storage_t<sizeof(T)>) );
}


BOOST_AUTO_TEST_CASE( test_aligned_storage_ptr )
{
  typedef void* T;

  BOOST_CHECK( sizeof(T)<=sizeof(aligned_storage_t<sizeof(T)>) );
}
