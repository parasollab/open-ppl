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
/// Unit test for @c std::shared_ptr and @c boost::shared_ptr marshaling.
//////////////////////////////////////////////////////////////////////

#define STAPL_RUNTIME_TEST_MODULE smart_ptr
#include "utility.h"
#include <stapl/runtime/request/arg_storage.hpp>
#include <stapl/runtime/serialization.hpp>
#include <functional>
#include <boost/optional.hpp>
#include "test_classes.h"
#include <memory>               // std::shared_ptr
#include <boost/shared_ptr.hpp> // boost::shared_ptr

using namespace stapl::runtime;

template<typename T>
void check_equal(std::shared_ptr<T> const& t1,
                 std::shared_ptr<T> const& t2) noexcept
{
  if (t1.get()==nullptr) {
    BOOST_CHECK_EQUAL(t1.get(), t2.get());
    return;
  }
  check_equal(*t1, *t2);
}

template<typename T>
void check_equal(boost::shared_ptr<T> const& t1,
                 boost::shared_ptr<T> const& t2) noexcept
{
  if (t1.get()==nullptr) {
    BOOST_CHECK_EQUAL(t1.get(), t2.get());
    return;
  }
  check_equal(*t1, *t2);
}


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

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  storage_type* const a = new(p) storage_type(*o, p, size);
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

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

#define TEST(T, init)                  \
{                                      \
  BOOST_TEST_MESSAGE( "Test of: "#T ); \
  const T val{(init)};                 \
  test_wrapper(val);                   \
}


#define TEST_NO_INIT(T)                \
{                                      \
  BOOST_TEST_MESSAGE( "Test of: "#T ); \
  const T val;                         \
  test_wrapper(val);                   \
}


BOOST_AUTO_TEST_CASE( primitives_std )
{
  TEST(std::shared_ptr<bool>, new bool(true))
  TEST(std::shared_ptr<bool>, static_cast<bool*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<bool>)

  TEST(std::shared_ptr<char>, new char('A'))
  TEST(std::shared_ptr<char>, static_cast<char*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<char>)

  TEST(std::shared_ptr<short>, new short(42))
  TEST(std::shared_ptr<short>, static_cast<short*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<short>)

  TEST(std::shared_ptr<int>, new int(42))
  TEST(std::shared_ptr<int>, static_cast<int*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<int>)

  TEST(std::shared_ptr<float>, new float(42.0f))
  TEST(std::shared_ptr<float>, static_cast<float*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<float>)

  TEST(std::shared_ptr<double>, new double(42.0))
  TEST(std::shared_ptr<double>, static_cast<double*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<double>)
}


BOOST_AUTO_TEST_CASE( pods_std )
{
  TEST(std::shared_ptr<pod_stapl>, new pod_stapl(42))
  TEST(std::shared_ptr<pod_stapl>, static_cast<pod_stapl*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<pod_stapl>)

  TEST(std::shared_ptr<pod_boost>, new pod_boost(42))
  TEST(std::shared_ptr<pod_boost>, static_cast<pod_boost*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<pod_boost>)
}


BOOST_AUTO_TEST_CASE( empty_std )
{
  TEST(std::shared_ptr<empty>, new empty)
  TEST(std::shared_ptr<empty>, static_cast<empty*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<empty>)
}


BOOST_AUTO_TEST_CASE( transient_std )
{
  TEST(std::shared_ptr<transient_type>, new transient_type(42))
  TEST(std::shared_ptr<transient_type>, static_cast<transient_type*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<transient_type>)
}


BOOST_AUTO_TEST_CASE( stack_std )
{
  TEST(std::shared_ptr<stack_base1>, new stack_base1(100))
  TEST(std::shared_ptr<stack_base1>, static_cast<stack_base1*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<stack_base1>)

  TEST(std::shared_ptr<stack_base2>, new stack_base2(100))
  TEST(std::shared_ptr<stack_base2>, static_cast<stack_base2*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<stack_base2>)

  TEST(std::shared_ptr<stack_object>, new stack_object(100))
  TEST(std::shared_ptr<stack_object>, static_cast<stack_object*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<stack_object>)
}


BOOST_AUTO_TEST_CASE( dynamic_std )
{
  TEST(std::shared_ptr<dynamic_base>, new dynamic_base(100))
  TEST(std::shared_ptr<dynamic_base>, static_cast<dynamic_base*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<dynamic_base>)

  TEST(std::shared_ptr<dynamic_object>, new dynamic_object(100))
  TEST(std::shared_ptr<dynamic_object>, static_cast<dynamic_object*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<dynamic_object>)
}


BOOST_AUTO_TEST_CASE( multi_inheritance_std )
{
  TEST(std::shared_ptr<multiple_inheritance>, new multiple_inheritance(100))
  TEST(std::shared_ptr<multiple_inheritance>,
       static_cast<multiple_inheritance*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<multiple_inheritance>)
}


BOOST_AUTO_TEST_CASE( static_array_std )
{
  typedef array_stapl<int, 100> array1_type;
  TEST(std::shared_ptr<array1_type>, new array1_type(100))
  TEST(std::shared_ptr<array1_type>, static_cast<array1_type*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<array1_type>)

  typedef array_boost<int, 100> array2_type;
  TEST(std::shared_ptr<array2_type>, new array2_type(100))
  TEST(std::shared_ptr<array2_type>, static_cast<array2_type*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<array2_type>)

  typedef array_stapl<dynamic_object, 100> array3_type;
  TEST(std::shared_ptr<array3_type>, new array3_type(100))
  TEST(std::shared_ptr<array3_type>, static_cast<array3_type*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<array3_type>)

  typedef array_boost<dynamic_object, 100> array4_type;
  TEST(std::shared_ptr<array4_type>, new array4_type(100))
  TEST(std::shared_ptr<array4_type>, static_cast<array4_type*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<array4_type>)
}


BOOST_AUTO_TEST_CASE( vectors_std )
{
  TEST(std::shared_ptr<vector_stapl<int>>, new vector_stapl<int>(100))
  TEST(std::shared_ptr<vector_stapl<int>>,
       static_cast<vector_stapl<int>*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<vector_stapl<int>>)

  TEST(std::shared_ptr<vector_boost<int>>, new vector_boost<int>(100))
  TEST(std::shared_ptr<vector_boost<int>>,
       static_cast<vector_boost<int>*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<vector_boost<int>>)

  TEST(std::shared_ptr<vector_stapl<dynamic_object>>,
       new vector_stapl<dynamic_object>(100))
  TEST(std::shared_ptr<vector_stapl<dynamic_object>>,
       static_cast<vector_stapl<dynamic_object>*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<vector_stapl<dynamic_object>>)

  TEST(std::shared_ptr<vector_boost<dynamic_object>>,
       new vector_boost<dynamic_object>(100))
  TEST(std::shared_ptr<vector_boost<dynamic_object>>,
       static_cast<vector_boost<dynamic_object>*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<vector_boost<dynamic_object>>)
}


BOOST_AUTO_TEST_CASE( ptr_ptr_std )
{
  TEST(std::shared_ptr<ptr_ptr_container<int>>,
       new ptr_ptr_container<int>(100))
  TEST(std::shared_ptr<ptr_ptr_container<int>>,
       static_cast<ptr_ptr_container<int>*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<ptr_ptr_container<int>>)

  TEST(std::shared_ptr<ptr_ptr_container<dynamic_object>>,
       new ptr_ptr_container<dynamic_object>(100))
  TEST(std::shared_ptr<ptr_ptr_container<dynamic_object>>,
       static_cast<ptr_ptr_container<dynamic_object>*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<ptr_ptr_container<dynamic_object> >)
}


BOOST_AUTO_TEST_CASE( poly_1_std )
{
  TEST(std::shared_ptr<poly_type1>, new poly_type1(100))
  TEST(std::shared_ptr<poly_type1>, static_cast<poly_type1*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<poly_type1>)

  TEST(std::shared_ptr<poly_type2>, new poly_type2(100))
  TEST(std::shared_ptr<poly_type2>, static_cast<poly_type2*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<poly_type2>)

  TEST(std::shared_ptr<poly_test>, new poly_test(100))
  TEST(std::shared_ptr<poly_test>, static_cast<poly_test*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<poly_test>)
}


BOOST_AUTO_TEST_CASE( poly_2_std )
{
  TEST(std::shared_ptr<poly2_type1>, new poly2_type1(100))
  TEST(std::shared_ptr<poly2_type1>, static_cast<poly2_type1*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<poly2_type1>)

  TEST(std::shared_ptr<poly2_type2>, new poly2_type2(100))
  TEST(std::shared_ptr<poly2_type2>, static_cast<poly2_type2*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<poly2_type2>)

  TEST(std::shared_ptr<poly2_test>, new poly2_test(100))
  TEST(std::shared_ptr<poly2_test>, static_cast<poly2_test*>(nullptr))
  TEST_NO_INIT(std::shared_ptr<poly2_test>)
}


BOOST_AUTO_TEST_CASE( primitives_boost )
{
  TEST(boost::shared_ptr<bool>, new bool(true))
  TEST(boost::shared_ptr<bool>, static_cast<bool*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<bool>)

  TEST(boost::shared_ptr<char>, new char('A'))
  TEST(boost::shared_ptr<char>, static_cast<char*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<char>)

  TEST(boost::shared_ptr<short>, new short(42))
  TEST(boost::shared_ptr<short>, static_cast<short*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<short>)

  TEST(boost::shared_ptr<int>, new int(42))
  TEST(boost::shared_ptr<int>, static_cast<int*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<int>)

  TEST(boost::shared_ptr<float>, new float(42.0f))
  TEST(boost::shared_ptr<float>, static_cast<float*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<float>)

  TEST(boost::shared_ptr<double>, new double(42.0))
  TEST(boost::shared_ptr<double>, static_cast<double*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<double>)
}


BOOST_AUTO_TEST_CASE( pods_boost )
{
  TEST(boost::shared_ptr<pod_stapl>, new pod_stapl(42))
  TEST(boost::shared_ptr<pod_stapl>, static_cast<pod_stapl*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<pod_stapl>)

  TEST(boost::shared_ptr<pod_boost>, new pod_boost(42))
  TEST(boost::shared_ptr<pod_boost>, static_cast<pod_boost*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<pod_boost>)
}


BOOST_AUTO_TEST_CASE( empty_boost )
{
  TEST(boost::shared_ptr<empty>, new empty)
  TEST(boost::shared_ptr<empty>, static_cast<empty*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<empty>)
}


BOOST_AUTO_TEST_CASE( transient_boost )
{
  TEST(boost::shared_ptr<transient_type>, new transient_type(42))
  TEST(boost::shared_ptr<transient_type>, static_cast<transient_type*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<transient_type>)
}


BOOST_AUTO_TEST_CASE( stack_boost )
{
  TEST(boost::shared_ptr<stack_base1>, new stack_base1(100))
  TEST(boost::shared_ptr<stack_base1>, static_cast<stack_base1*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<stack_base1>)

  TEST(boost::shared_ptr<stack_base2>, new stack_base2(100))
  TEST(boost::shared_ptr<stack_base2>, static_cast<stack_base2*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<stack_base2>)

  TEST(boost::shared_ptr<stack_object>, new stack_object(100))
  TEST(boost::shared_ptr<stack_object>, static_cast<stack_object*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<stack_object>)
}


BOOST_AUTO_TEST_CASE( dynamic_boost )
{
  TEST(boost::shared_ptr<dynamic_base>, new dynamic_base(100))
  TEST(boost::shared_ptr<dynamic_base>, static_cast<dynamic_base*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<dynamic_base>)

  TEST(boost::shared_ptr<dynamic_object>, new dynamic_object(100))
  TEST(boost::shared_ptr<dynamic_object>, static_cast<dynamic_object*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<dynamic_object>)
}


BOOST_AUTO_TEST_CASE( multi_inheritance_boost )
{
  TEST(boost::shared_ptr<multiple_inheritance>, new multiple_inheritance(100))
  TEST(boost::shared_ptr<multiple_inheritance>,
       static_cast<multiple_inheritance*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<multiple_inheritance>)
}


BOOST_AUTO_TEST_CASE( static_array_boost )
{
  typedef array_stapl<int, 100> array1_type;
  TEST(boost::shared_ptr<array1_type>, new array1_type(100))
  TEST(boost::shared_ptr<array1_type>, static_cast<array1_type*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<array1_type>)

  typedef array_boost<int, 100> array2_type;
  TEST(boost::shared_ptr<array2_type>, new array2_type(100))
  TEST(boost::shared_ptr<array2_type>, static_cast<array2_type*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<array2_type>)

  typedef array_stapl<dynamic_object, 100> array3_type;
  TEST(boost::shared_ptr<array3_type>, new array3_type(100))
  TEST(boost::shared_ptr<array3_type>, static_cast<array3_type*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<array3_type>)

  typedef array_boost<dynamic_object, 100> array4_type;
  TEST(boost::shared_ptr<array4_type>, new array4_type(100))
  TEST(boost::shared_ptr<array4_type>, static_cast<array4_type*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<array4_type>)
}


BOOST_AUTO_TEST_CASE( vectors_boost )
{
  TEST(boost::shared_ptr<vector_stapl<int>>, new vector_stapl<int>(100))
  TEST(boost::shared_ptr<vector_stapl<int>>,
       static_cast<vector_stapl<int>*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<vector_stapl<int>>)

  TEST(boost::shared_ptr<vector_boost<int>>, new vector_boost<int>(100))
  TEST(boost::shared_ptr<vector_boost<int>>,
       static_cast<vector_boost<int>*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<vector_boost<int>>)

  TEST(boost::shared_ptr<vector_stapl<dynamic_object>>,
       new vector_stapl<dynamic_object>(100))
  TEST(boost::shared_ptr<vector_stapl<dynamic_object>>,
       static_cast<vector_stapl<dynamic_object>*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<vector_stapl<dynamic_object>>)

  TEST(boost::shared_ptr<vector_boost<dynamic_object>>,
       new vector_boost<dynamic_object>(100))
  TEST(boost::shared_ptr<vector_boost<dynamic_object>>,
       static_cast<vector_boost<dynamic_object>*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<vector_boost<dynamic_object>>)
}


BOOST_AUTO_TEST_CASE( ptr_ptr_boost )
{
  TEST(boost::shared_ptr<ptr_ptr_container<int>>,
       new ptr_ptr_container<int>(100))
  TEST(boost::shared_ptr<ptr_ptr_container<int>>,
       static_cast<ptr_ptr_container<int>*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<ptr_ptr_container<int>>)

  TEST(boost::shared_ptr<ptr_ptr_container<dynamic_object>>,
       new ptr_ptr_container<dynamic_object>(100))
  TEST(boost::shared_ptr<ptr_ptr_container<dynamic_object>>,
       static_cast<ptr_ptr_container<dynamic_object>*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<ptr_ptr_container<dynamic_object> >)
}


BOOST_AUTO_TEST_CASE( poly_1_boost )
{
  TEST(boost::shared_ptr<poly_type1>, new poly_type1(100))
  TEST(boost::shared_ptr<poly_type1>, static_cast<poly_type1*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<poly_type1>)

  TEST(boost::shared_ptr<poly_type2>, new poly_type2(100))
  TEST(boost::shared_ptr<poly_type2>, static_cast<poly_type2*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<poly_type2>)

  TEST(boost::shared_ptr<poly_test>, new poly_test(100))
  TEST(boost::shared_ptr<poly_test>, static_cast<poly_test*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<poly_test>)
}


BOOST_AUTO_TEST_CASE( poly_2_boost )
{
  TEST(boost::shared_ptr<poly2_type1>, new poly2_type1(100))
  TEST(boost::shared_ptr<poly2_type1>, static_cast<poly2_type1*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<poly2_type1>)

  TEST(boost::shared_ptr<poly2_type2>, new poly2_type2(100))
  TEST(boost::shared_ptr<poly2_type2>, static_cast<poly2_type2*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<poly2_type2>)

  TEST(boost::shared_ptr<poly2_test>, new poly2_test(100))
  TEST(boost::shared_ptr<poly2_test>, static_cast<poly2_test*>(nullptr))
  TEST_NO_INIT(boost::shared_ptr<poly2_test>)
}
