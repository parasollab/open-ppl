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
/// Unit test for @ref stapl::runtime::arg_storage.
//////////////////////////////////////////////////////////////////////

#define STAPL_RUNTIME_TEST_MODULE arg_storage
#include "utility.h"
#include <stapl/runtime/request/arg_storage.hpp>
#include <stapl/runtime/type_traits/aligned_storage.hpp>
#include <cstring>
#include <type_traits>
#include <boost/optional.hpp>
#include "test_classes.h"

using namespace stapl::runtime;

template<typename Argument, typename Expected, typename T>
void tester_function(T&& init)
{
  BOOST_TEST_MESSAGE( "Test Set-up" );

  typedef typename std::remove_cv<Argument>::type stripped_argument_type;
  typedef arg_storage_t<Argument, Expected>       storage_type;

  const bool is_empty = std::is_empty<Argument>::value;

  Argument t(init);                  // argument (reference object)
  boost::optional<Argument> o(init); // object to use for packing
  Expected e = const_cast<stripped_argument_type&>(t);
  BOOST_REQUIRE( *o==t );

  BOOST_TEST_MESSAGE( "Finding size" );

  const std::size_t static_size = sizeof(storage_type);
  if (!is_empty)
    BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  else
    BOOST_CHECK_EQUAL( static_size, std::size_t(1) );
  const std::size_t sz = (static_size + storage_type::packed_size(*o));
  if (!is_empty)
    BOOST_CHECK_EQUAL( sz, aligned_size(sz) );
  else
    BOOST_CHECK_EQUAL( sz, std::size_t(1) );
  BOOST_REQUIRE( *o==t );

  BOOST_TEST_MESSAGE( "Packing" );

  buffer buf{sz};
  void* p = buf.address();
  std::size_t size = static_size;
  storage_type* const a = new(p) storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );
  BOOST_REQUIRE( *o==t );
  o = boost::none;

  BOOST_TEST_MESSAGE( "Unpacking" );

  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)==e );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  BOOST_TEST_MESSAGE( "Clean-up" );

  a->~storage_type();
  BOOST_CHECK( !buf.overwritten() );

  BOOST_TEST_MESSAGE( "Test Complete" );
}


template<typename Argument, typename Expected, typename T>
void tester_function_rvalue(T&& init)
{
  BOOST_TEST_MESSAGE( "Test Set-up" );

  typedef typename std::remove_cv<Argument>::type stripped_argument_type;
  typedef arg_storage_t<Argument, Expected&&>     storage_type;

  const bool is_empty = std::is_empty<Argument>::value;

  Argument t(init);                  // argument (reference object)
  boost::optional<Argument> o(init); // object to use for packing
  Expected e = const_cast<stripped_argument_type&>(t);
  BOOST_REQUIRE( *o==t );

  BOOST_TEST_MESSAGE( "Finding size" );

  const std::size_t static_size = sizeof(storage_type);
  if (!is_empty)
    BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  else
    BOOST_CHECK_EQUAL( static_size, std::size_t(1) );
  const std::size_t sz = (static_size +
                          storage_type::packed_size(std::move(*o)));
  BOOST_REQUIRE( *o==t );
  if (!is_empty)
    BOOST_CHECK_EQUAL( sz, aligned_size(sz) );
  else
    BOOST_CHECK_EQUAL( sz, std::size_t(1) );

  BOOST_TEST_MESSAGE( "Packing" );

  buffer buf{sz};
  void* p = buf.address();
  std::size_t size = static_size;
  storage_type* const a = new(p) storage_type{std::move(*o), p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );
  o = boost::none;

  BOOST_TEST_MESSAGE( "Unpacking" );

  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)==e );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  BOOST_TEST_MESSAGE( "Clean-up" );

  a->~storage_type();
  BOOST_CHECK( !buf.overwritten() );

  BOOST_TEST_MESSAGE( "Test Complete" );
}


template<typename Argument, typename Expected, typename T>
void tester_function_ptr(T&& init)
{
  BOOST_TEST_MESSAGE( "Test Set-up" );

  typedef typename std::remove_pointer<
            typename std::decay<Argument>::type
          >::type                           stripped_argument_type;
  typedef arg_storage_t<Argument, Expected> storage_type;

  stripped_argument_type t(init);
  boost::optional<stripped_argument_type> tt = t;
  Argument o = &(*tt);
  Expected e = &t;
  BOOST_REQUIRE( *o==t );

  BOOST_TEST_MESSAGE( "Finding size" );

  const std::size_t static_size = sizeof(storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = (static_size + storage_type::packed_size(o));
  BOOST_REQUIRE( *o==t );
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  BOOST_TEST_MESSAGE( "Packing" );

  buffer buf{sz};
  void* p = buf.address();
  std::size_t size = static_size;
  storage_type* const a = new(p) storage_type{o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );
  BOOST_REQUIRE( *o==t );
  tt = boost::none;

  BOOST_TEST_MESSAGE( "Unpacking" );

  std::size_t size2 = static_size;
  BOOST_CHECK( *(a->get(p, size2))==*e );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  BOOST_TEST_MESSAGE( "Clean-up" );

  a->~storage_type();
  BOOST_CHECK( !buf.overwritten() );

  BOOST_TEST_MESSAGE( "Test Complete" );
}


#define CALL_TEST(A, E, init)                           \
  BOOST_TEST_MESSAGE("# arg_storage_t<" #A "," #E ">"); \
  tester_function<A, E>(init);                          \
  BOOST_TEST_MESSAGE("# done arg_storage_t<" #A "," #E ">");

#define CALL_RVALUE_TEST(A, E, init)                      \
  BOOST_TEST_MESSAGE("# arg_storage_t<" #A "," #E "&&>"); \
  tester_function_rvalue<A, E>(init);                     \
  BOOST_TEST_MESSAGE("# done arg_storage_t<" #A "," #E "&&>");

#define CALL_PTR_TEST(A, E, init)                       \
  BOOST_TEST_MESSAGE("# arg_storage_t<" #A "," #E ">"); \
  tester_function_ptr<A, E>(init);                      \
  BOOST_TEST_MESSAGE("# done arg_storage_t<" #A "," #E ">");


#define TEST_BY_VALUE(A, E, init)   \
{                                   \
  CALL_TEST(A, E, init)             \
  CALL_TEST(A, E const, init)       \
  CALL_TEST(A const, E, init)       \
  CALL_TEST(A const, E const, init) \
}

#define TEST_BY_REF(A, E, init)      \
{                                    \
  CALL_TEST(A, E&, init)             \
  CALL_TEST(A, E const&, init)       \
  CALL_TEST(A const, E const&, init) \
}

#define TEST_BY_MOVE(A, E, init)     \
{                                    \
  CALL_RVALUE_TEST(A, E, init)       \
  CALL_RVALUE_TEST(A, E const, init) \
}

#define TEST_BY_POINTER(A, E, init)             \
{                                               \
  CALL_PTR_TEST(A*, E*, init)                   \
  CALL_PTR_TEST(A*, E* const, init)             \
  CALL_PTR_TEST(A*, const E*, init)             \
  CALL_PTR_TEST(A*, const E* const, init)       \
  CALL_PTR_TEST(const A*, const E*, init)       \
  CALL_PTR_TEST(const A*, const E* const, init) \
}


#define TEST(T, init)                  \
{                                      \
  BOOST_TEST_MESSAGE( "Test of: "#T ); \
  TEST_BY_VALUE(T, T, init)            \
  TEST_BY_REF(T, T, init)              \
  TEST_BY_MOVE(T, T, init)             \
  TEST_BY_POINTER(T, T, init)          \
}

#define TEST_POLY(T, Base, init)                             \
{                                                            \
  BOOST_TEST_MESSAGE( "Test of: "#T " extends from "#Base ); \
  /*const T val((init));                                     \
  test_reference<T, Base>(val);                              \
  test_pointer<T, Base>(val); */                             \
}


// Testcases:


BOOST_AUTO_TEST_CASE( cv_correctness_1 )
{
  // commented out cases should not be able to compile at all

  typedef normal_struct T;

  auto init = 42;

  CALL_TEST(T, T, init)
  CALL_TEST(T, T const, init)
  CALL_TEST(T, T volatile, init)
  CALL_TEST(T, T const volatile, init)

  CALL_TEST(T const, T, init)
  CALL_TEST(T const, T const, init)
  CALL_TEST(T const, T volatile, init)
  CALL_TEST(T const, T const volatile, init)

#if 0
  // no conversion for the following
  CALL_TEST(T volatile, T, init)
  CALL_TEST(T volatile, T const, init)
  CALL_TEST(T volatile, T volatile, init)
  CALL_TEST(T volatile, T const volatile, init)

  CALL_TEST(T const volatile, T, init)
  CALL_TEST(T const volatile, T const, init)
  CALL_TEST(T const volatile, T volatile, init)
  CALL_TEST(T const volatile, T const volatile, init)
#endif

  CALL_TEST(T, T&, init)
  CALL_TEST(T, T const&, init)
  CALL_TEST(T, T volatile&, init)
  CALL_TEST(T, T const volatile&, init)

  CALL_TEST(T const, T const&, init)
  CALL_TEST(T const, T const volatile&, init)
#if 0
  // no conversion for the following
  CALL_TEST(T const, T&, init)
  CALL_TEST(T const, T volatile&, init)
#endif

#if 0
  // no conversion for the following
  CALL_TEST(T volatile, T&, init)
  CALL_TEST(T volatile, T const&, init)
#endif
  CALL_TEST(T volatile, T volatile&, init)
  CALL_TEST(T volatile, T const volatile&, init)

#if 0
  // no conversion for the following
  CALL_TEST(T const volatile, T&, init)
  CALL_TEST(T const volatile, T const&, init)
  CALL_TEST(T const volatile, T volatile&, init)
#endif
  CALL_TEST(T const volatile, T const volatile&, init)
}


BOOST_AUTO_TEST_CASE( cv_correctness_2 )
{
  typedef cv_struct T;

  auto init = 42;

  CALL_TEST(T, T, init)
  CALL_TEST(T, T const, init)
  CALL_TEST(T, T volatile, init)
  CALL_TEST(T, T const volatile, init)

  CALL_TEST(T const, T, init)
  CALL_TEST(T const, T const, init)
  CALL_TEST(T const, T volatile, init)
  CALL_TEST(T const, T const volatile, init)

  CALL_TEST(T volatile, T, init)
  CALL_TEST(T volatile, T const, init)
  CALL_TEST(T volatile, T volatile, init)
  CALL_TEST(T volatile, T const volatile, init)

  CALL_TEST(T const volatile, T, init)
  CALL_TEST(T const volatile, T const, init)
  CALL_TEST(T const volatile, T volatile, init)
  CALL_TEST(T const volatile, T const volatile, init)

  CALL_TEST(T, T&, init)
  CALL_TEST(T, T const&, init)
  CALL_TEST(T, T volatile&, init)
  CALL_TEST(T, T const volatile&, init)

  CALL_TEST(T const, T const&, init)
  CALL_TEST(T const, T const volatile&, init)
#if 0
  // no conversion for the following
  CALL_TEST(T const, T&, init)
  CALL_TEST(T const, T volatile&, init)
#endif

#if 0
  // no conversion for the following
  CALL_TEST(T volatile, T&, init)
  CALL_TEST(T volatile, T const&, init)
#endif
  CALL_TEST(T volatile, T volatile&, init)
  CALL_TEST(T volatile, T const volatile&, init)

#if 0
  // no conversion for the following
  CALL_TEST(T const volatile, T&, init)
  CALL_TEST(T const volatile, T const&, init)
  CALL_TEST(T const volatile, T volatile&, init)
#endif
  CALL_TEST(T const volatile, T const volatile&, init)
}


BOOST_AUTO_TEST_CASE( primitives )
{
  TEST(bool, true)
  TEST(char, 'A')
  TEST(short, 42)
  TEST(int, 42)
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
