/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE typer_traits
#include "utility.h"
#include <stapl/runtime/serialization.hpp>
#include <cstring>
#include <memory>
#include <boost/optional.hpp>
#include "test_classes.h"

template<typename T>
void test_wrapper(T const& t)
{
  using namespace stapl;
  using namespace stapl::runtime;

  typedef typer_traits<T> traits_type;

  boost::optional<T> o = t;
  BOOST_REQUIRE( *o==t );

  // find size
  const std::size_t sz =
    (aligned_size(sizeof(T)) +
     aligned_size(traits_type::packed_size(*o)));
  const std::size_t sz2 =
    (aligned_size(sizeof(T)) +
     aligned_size(traits_type::meets_requirements(typer::COPY, *o).second));
  const std::size_t sz3 =
    (aligned_size(sizeof(T)) +
     aligned_size(traits_type::meets_requirements(typer::MOVE, *o).second));
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );
  BOOST_CHECK_EQUAL( sz, sz2 );
  BOOST_CHECK_EQUAL( sz, sz3 );
  BOOST_REQUIRE( *o==t );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();
  T& r    = *static_cast<T*>(p);

  // pack
  const std::size_t size = aligned_size(sizeof(T));
  traits_type::prepack(std::addressof(r), std::addressof(*o));
  BOOST_CHECK( !buf.overwritten() );
  const std::size_t psz =
    (size + aligned_size(traits_type::pack(r, p, size, *o)));
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( sz, psz );
  BOOST_CHECK( *o==t );

  // clear o
  o = boost::none;

  // unpack
  const std::size_t usz =
    (size + aligned_size(traits_type::unpack(r, p)));
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( sz, usz );
  BOOST_CHECK( r==t );
  BOOST_CHECK( !buf.overwritten() );

  // destroy
  traits_type::destroy(r);
  BOOST_CHECK( !buf.overwritten() );
}


template<typename U, typename T>
void test_ptr_wrapper(T const& t)
{
  using namespace stapl;
  using namespace stapl::runtime;

  typedef typer_traits<U*> traits_type;

  boost::optional<T> u = t;
  U* o = &(*u);
  BOOST_REQUIRE( *o==t );

  // find size
  const std::size_t sz =
    (aligned_size(sizeof(U*)) +
     aligned_size(traits_type::packed_size(o)));
  const std::size_t sz2 =
    (aligned_size(sizeof(U*)) +
     aligned_size(traits_type::meets_requirements(typer::COPY, o).second));
  const std::size_t sz3 =
    (aligned_size(sizeof(U*)) +
     aligned_size(traits_type::meets_requirements(typer::MOVE, o).second));
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );
  BOOST_CHECK_EQUAL( sz, sz2 );
  BOOST_CHECK_EQUAL( sz, sz3 );
  BOOST_REQUIRE( *o==t );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();
  U** pt  = static_cast<U**>(p);

  // pack
  const std::size_t size = aligned_size(sizeof(U*));
  traits_type::prepack(pt, &o);
  BOOST_CHECK( !buf.overwritten() );
  const std::size_t psz =
    (size + aligned_size(traits_type::pack(*pt, p, size, o)));
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( sz, psz );
  BOOST_CHECK( *o==t );

  // clear u
  u = boost::none;

  // unpack
  const std::size_t usz =
    (size + aligned_size(traits_type::unpack(*pt, p)));
  BOOST_CHECK( !buf.overwritten() );
  U const* r = *pt;
  BOOST_CHECK_EQUAL( sz, usz );
  BOOST_CHECK( *r==t );
  BOOST_CHECK( !buf.overwritten() );

  // destroy
  traits_type::destroy(*pt);
  BOOST_CHECK( !buf.overwritten() );
}


#define TEST(T, init) {                      \
    BOOST_TEST_MESSAGE( "Testing for: "#T ); \
    const T val((init));                     \
    test_wrapper(val);                       \
    test_ptr_wrapper<T>(val);                \
}


#define TEST_POLY(T, Base, init) {           \
    BOOST_TEST_MESSAGE( "Testing for: "#T ); \
    const T val((init));                     \
    test_ptr_wrapper<Base>(val);             \
}


// Testcases:

BOOST_AUTO_TEST_CASE( primitives )
{
  TEST(bool, true)
  TEST(char, 'A')
  TEST(short, true)
  TEST(int, true)
  TEST(float, 42.0f)
  TEST(double, 42.0)
}


BOOST_AUTO_TEST_CASE( pods )
{
  TEST(pod_stapl, 42)
  TEST(pod_boost, 42)
}


BOOST_AUTO_TEST_CASE( empty_struct )
{
  TEST(empty, empty())
}


BOOST_AUTO_TEST_CASE( transient )
{
  TEST(transient_type, 42)
}


BOOST_AUTO_TEST_CASE( stack )
{
  TEST(stack_base1, 100)
  TEST(stack_base2, 100)
  TEST(stack_object, 100)
}


BOOST_AUTO_TEST_CASE( dynamic )
{
  TEST(dynamic_base, 100)
  TEST(dynamic_object, 100)
}


BOOST_AUTO_TEST_CASE( multi_inheritance )
{
  TEST(multiple_inheritance, 100)
}


BOOST_AUTO_TEST_CASE( static_array )
{
  typedef array_stapl<int, 100> array1_type;
  TEST(array1_type, 100)
  typedef array_boost<int, 100> array2_type;
  TEST(array2_type, 100)

  typedef array_stapl<dynamic_base, 100> array3_type;
  TEST(array3_type, 100)
  typedef array_boost<dynamic_base, 100> array4_type;
  TEST(array4_type, 100)

  typedef array_stapl<dynamic_object, 100> array5_type;
  TEST(array5_type, 100)
  typedef array_boost<dynamic_object, 100> array6_type;
  TEST(array6_type, 100)
}


BOOST_AUTO_TEST_CASE( vectors )
{
  TEST(vector_stapl<int>, 100)
  TEST(vector_boost<int>, 100)

  TEST(vector_stapl<dynamic_base>, 100)
  TEST(vector_boost<dynamic_base>, 100)

  TEST(vector_stapl<dynamic_object>, 100)
  TEST(vector_boost<dynamic_object>, 100)
}


BOOST_AUTO_TEST_CASE( ptr_ptr )
{
  TEST(ptr_ptr_container<int>, 100)
  TEST(ptr_ptr_container<dynamic_object>, 100)
}


BOOST_AUTO_TEST_CASE( mem_ptrs )
{
  typedef mem_ptr_container<
            array_stapl<int, 100>, int
          > array1_type;
  TEST(array1_type, 100)

  typedef mem_ptr_container<
            array_boost<int, 100>, int
          > array2_type;
  TEST(array2_type, 100)

  typedef mem_ptr_container<
            array_stapl<dynamic_object, 100>, dynamic_object
          > array3_type;
  TEST(array3_type, 100)

  typedef mem_ptr_container<
            array_boost<dynamic_object, 100>, dynamic_object
          > array4_type;
  TEST(array4_type, 100)
}


BOOST_AUTO_TEST_CASE( cv_quals )
{
  TEST(cv_class_stapl<int>, 100)
  TEST(cv_class_boost<int>, 100)
  TEST(cv_class_stapl<pod_stapl>, 100)
  TEST(cv_class_boost<pod_stapl>, 100)
  TEST(cv_class_stapl<pod_boost>, 100)
  TEST(cv_class_boost<pod_boost>, 100)
}


BOOST_AUTO_TEST_CASE( polymorphic_1 )
{
  TEST(poly_type1, 100)
  TEST(poly_type2, 100)
  TEST_POLY(poly_type1, poly_base, 100)
  TEST_POLY(poly_type2, poly_base, 100)
  TEST(poly_test, 100)
}


BOOST_AUTO_TEST_CASE( polymorphic_2 )
{
  TEST(poly2_type1, 100)
  TEST(poly2_type2, 100)
  TEST_POLY(poly2_type1, poly2_base, 100)
  TEST_POLY(poly2_type2, poly2_base, 100)
  TEST(poly2_test, 100)
}
