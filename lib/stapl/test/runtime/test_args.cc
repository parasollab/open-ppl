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
/// Test which stresses all combinations of basic/pointer/reference and
/// non-const/const data types as arguments to RMI calls, specified via
/// stapl::typer. It also stresses a variety of fundamental types, and
/// user-defined objects that use local and dynamic variables.
///
/// The types are passed as arguments to @ref stapl::async_rmi() and as return
/// values from @ref stapl::sync_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>
#include "test_containers.h"
#include "test_utils.h"

using namespace stapl;

class A
{
public:
  virtual ~A(void)
  { }

  STAPL_POLYMORPHIC_TYPE()

  virtual void foo(void) = 0;

  void define_type(typer&)
  { }
};

class B
: public A
{
private:
  int a, b, c;
  int *d;

public:
  B(void)
  : a(1), b(2), c(3), d(new int(4))
  { }

  B(B const& other)
  : a(other.a), b(other.b), c(other.c), d(new int(*other.d))
  { }

  virtual ~B(void)
  { delete d; }

  virtual void foo(void)
  { STAPL_RUNTIME_TEST_REQUIRE((a == 1 && b == 2 && c == 3 && *d == 4)); }

  STAPL_POLYMORPHIC_TYPE()

  void define_type(typer& t)
  {
    t.base<A>(*this);
    t.member(a);
    t.member(b);
    t.member(c);
    t.member(d);
  }
};

class C
: public A
{
private:
  int a, b, c, d;
  std::vector<int> j;

public:
  C(void)
  : a(1), b(2), c(3), d(4)
  { j.push_back(5); }

  virtual void foo(void)
  {
    STAPL_RUNTIME_TEST_REQUIRE(
        (a == 1 && b == 2 && c == 3 && d == 4 && j[0] == 5));
  }

  STAPL_POLYMORPHIC_TYPE()

  void define_type(typer& t)
  {
    t.base<A>(*this);
    t.member(a);
    t.member(b);
    t.member(c);
    t.member(d);
    t.member(j);
  }
};

namespace stapl {

template<>
struct derived_types<A>
{
  typedef std::tuple<B, C> typelist_type;
};

} // namespace stapl


// complex_object combines all the test containers available, stressing all the
// typer mechanisms at once!
struct complex_object
{
  test_vector<stack_object> v;
  test_list<dynamic_object> l;
  test_vector<stack_object>* vP;
  test_list<stack_object>* lP;

  complex_object(const int size)
  : v(size),
    vP(new test_vector<stack_object>(size)),
    lP(new test_list<stack_object>())
  { }

  complex_object(complex_object const& c)
  : v(c.v),
    l(c.l),
    vP(new test_vector<stack_object>(*c.vP)),
    lP(new test_list<stack_object>(*c.lP))
  { }

  ~complex_object(void)
  {
    delete vP;
    delete lP;
  }

  bool operator==(complex_object const& c) const
  {
    return (c.v == v) && (c.l == l) && (*c.vP == *vP) && (*c.lP == *lP);
  }

  bool operator!=(complex_object const& c) const
  { return !(operator==(c)); }

  void define_type(typer& t)
  {
    t.member(v);
    t.member(l);
    t.member(vP);
    t.member(lP);
  }
};


// Class that returns the location id with an implicit conversion operator
class deferred_get_location_id
{
public:
  operator unsigned int(void) const
  { return stapl::get_location_id(); }
};


// Class that returns a pointer to an object with an implicit conversion
// operator
template<typename T>
class to_ptr
{
private:
  T m_t;

public:
  to_ptr(void)
  : m_t()
  { }

  to_ptr(T const& t)
  : m_t(t)
  { }

  operator T*(void)
  { return &m_t; }

  void define_type(typer& t)
  { t.member(m_t); }
};


// Class that returns a pointer to an object with an implicit conversion
// operator
template<typename T>
class to_const_ptr
{
private:
  T m_t;

public:
  to_const_ptr(void)
  : m_t()
  { }

  to_const_ptr(T const& t)
  : m_t(t)
  { }

  operator const T*(void) const
  { return &m_t; }

  void define_type(typer& t)
  { t.member(m_t); }
};


struct p_test
: public p_object
{
  unsigned int left, right;  // neighbor id's

  p_test(void)
  {
    const unsigned int id = this->get_location_id();
    right = (id == this->get_num_locations() - 1) ? 0 : id + 1;
    left = (id == 0) ? this->get_num_locations() - 1 : id - 1;
    this->advance_epoch();
  }

  template<typename T>
  T identity(T t)
  { return t; }

  void stack_call_v(const stack_object lt)
  {
    STAPL_RUNTIME_TEST_CHECK(lt, stack_object(left));
  }

  void stack_call_p(const stack_object* const lt)
  {
    STAPL_RUNTIME_TEST_CHECK(*lt, stack_object(left));
  }

  void stack_call_r(stack_object& lt)
  {
    STAPL_RUNTIME_TEST_CHECK(lt, stack_object(left));
  }

  stack_object stack_return0(void)
  {
    return stack_object(this->get_location_id());
  }

  stack_object stack_return(stack_object* const rt)
  {
    STAPL_RUNTIME_TEST_CHECK(*rt, stack_object(right));
    return *rt;
  }

  void dynamic_call_v(dynamic_object rt)
  {
    STAPL_RUNTIME_TEST_CHECK(rt, dynamic_object(right));
  }

  void dynamic_call_p(const dynamic_object* const rt)
  {
    STAPL_RUNTIME_TEST_CHECK(*rt, dynamic_object(right));
  }

  void dynamic_call_r(dynamic_object& rt)
  {
    STAPL_RUNTIME_TEST_CHECK(rt, dynamic_object(right));
  }

  dynamic_object dynamic_return0(void)
  {
    return dynamic_object(this->get_location_id());
  }

  dynamic_object dynamic_return(dynamic_object* const lt)
  {
    STAPL_RUNTIME_TEST_CHECK(*lt, dynamic_object(left));
    return *lt;
  }

  void offset_pointers(const int size, test_vector2<test_vector2<double> >& v)
  {
    STAPL_RUNTIME_TEST_REQUIRE(v.check(size));
    for (int i = 0; i < size; ++i) {
      STAPL_RUNTIME_TEST_REQUIRE(v[i].check(i));
      for (int j = 0; j < i; ++j)
        STAPL_RUNTIME_TEST_REQUIRE(
            (v[i][j] - left < std::numeric_limits<double>::epsilon()));
    }
  }

  void dynamic_types(const int size, test_list<int> const& l)
  {
    int count = l.size();
    STAPL_RUNTIME_TEST_CHECK(size, count);
    for (test_list<int>::const_iterator i = l.begin(); i != l.end(); ++i)
      STAPL_RUNTIME_TEST_CHECK(*i, --count);
  }

  complex_object complex_type(complex_object& c)
  {
    return c;
  }

  void bvector_type(std::vector<bool>& v, std::size_t sz)
  {
    STAPL_RUNTIME_TEST_CHECK(v.size(), sz);
    bool b = true;
    for (std::vector<bool>::const_iterator it=v.begin(); it!=v.end(); ++it) {
      STAPL_RUNTIME_TEST_CHECK(*it, b);
      b = !b;
    }
  }

  std::vector<bool> bvector_sync_type(void)
  {
    std::vector<bool> vb;
    bool b = true;
    int i = 0;
    while (i < 100) {
      vb.push_back(b);
      b = !b;
      ++i;
    }
    return vb;
  }

  void dvector_type(std::vector<double>& v, std::size_t sz)
  {
    STAPL_RUNTIME_TEST_CHECK(v.size(), sz);
    std::size_t i = 0;
    while (i < v.size()) {
      STAPL_RUNTIME_TEST_REQUIRE(
          (v[i] - i < std::numeric_limits<double>::epsilon()));
      ++i;
    }
  }

  void tuple_type(const std::tuple<int, int, int>& t)
  {
    STAPL_RUNTIME_TEST_CHECK(t, std::make_tuple(1, 2, 3));
  }

  void tuple_type_ptr(const std::tuple<int*, int*, int*>& t)
  {
    STAPL_RUNTIME_TEST_CHECK(*std::get<0>(t), 1);
    STAPL_RUNTIME_TEST_CHECK(*std::get<1>(t), 2);
    STAPL_RUNTIME_TEST_CHECK(*std::get<2>(t), 3);
  }

  void poly_type(A* const a_ptr)
  {
    a_ptr->foo();
  }

  void poly_tuple_type(const std::tuple<A*, A*>& t)
  {
    std::get<0>(t)->foo();
    std::get<1>(t)->foo();
  }

  std::vector<double> dvector_sync_type(void)
  {
    std::vector<double> vb;
    int i = 0;
    while (i < 100) {
      vb.push_back(i);
      ++i;
    }
    return vb;
  }

  void vector_move(std::vector<int>&& v)
  {
    const std::vector<int> u(10, 42.0);
    STAPL_RUNTIME_TEST_RANGE(v, u);
  }

  void consume_vec_shared_ptr(
    std::vector<std::shared_ptr<std::vector<int>>> const& v,
    int val)
  {
    int t = 0;
    for (auto const& p : v)
      for (auto const& i : *p)
        t += i;
    STAPL_RUNTIME_TEST_CHECK(t, val);
  }

  void copy_consume_vec_shared_ptr(
    std::vector<std::shared_ptr<std::vector<int>>> v,
    int val)
  {
    int t = 0;
    for (auto const& p : v)
      for (auto const& i : *p)
        t += i;
    STAPL_RUNTIME_TEST_CHECK(t, val);
  }

  void move_consume_vec_shared_ptr(
    std::vector<std::shared_ptr<std::vector<int>>>&& v,
    int val)
  {
    int t = 0;
    for (auto const& p : v)
      for (auto const& i : *p)
        t += i;
    STAPL_RUNTIME_TEST_CHECK(t, val);
  }

  void deferred_check(const unsigned int lid)
  {
    const auto here = stapl::get_location_id();
    STAPL_RUNTIME_TEST_CHECK(lid, here);
  }

  template<typename T>
  void ptr_check(T* t1, T const& t2)
  {
    STAPL_RUNTIME_TEST_CHECK(*t1, t2);
  }

  template<typename T>
  void const_ptr_check(T const* t1, T const& t2)
  {
    STAPL_RUNTIME_TEST_CHECK(*t1, t2);
  }

  void test_fundamental(void)
  {
    STAPL_RUNTIME_TEST_CHECK(
      false,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<bool>, false));

    STAPL_RUNTIME_TEST_CHECK(
      '1',
      sync_rmi(right, get_rmi_handle(), &p_test::identity<char>, '1'));
    STAPL_RUNTIME_TEST_CHECK(
      '2',
      sync_rmi(right, get_rmi_handle(), &p_test::identity<signed char>, '2'));
    STAPL_RUNTIME_TEST_CHECK(
      '3',
      sync_rmi(right, get_rmi_handle(), &p_test::identity<unsigned char>, '3'));

    STAPL_RUNTIME_TEST_CHECK(
      4,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<short>, 4));
    STAPL_RUNTIME_TEST_CHECK(
      5,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<signed short>, 5));
    STAPL_RUNTIME_TEST_CHECK(
      6,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<unsigned short>, 6));

    STAPL_RUNTIME_TEST_CHECK(
      7,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<int>, 7));
    STAPL_RUNTIME_TEST_CHECK(
      8,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<signed int>, 8));
    STAPL_RUNTIME_TEST_CHECK(
      9,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<unsigned int>, 9));

    STAPL_RUNTIME_TEST_CHECK(
      10,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<long>, 10));
    STAPL_RUNTIME_TEST_CHECK(
      11,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<signed long>, 11));
    STAPL_RUNTIME_TEST_CHECK(
      12,
      sync_rmi(right, get_rmi_handle(), &p_test::identity<unsigned long>, 12));

    STAPL_RUNTIME_TEST_REQUIRE(
      (1.0 - sync_rmi(right, get_rmi_handle(), &p_test::identity<float>, 1.0f))
        < std::numeric_limits<float>::epsilon());
    STAPL_RUNTIME_TEST_REQUIRE(
      (1.1 - sync_rmi(right, get_rmi_handle(), &p_test::identity<double>, 1.1))
        < std::numeric_limits<double>::epsilon());
    STAPL_RUNTIME_TEST_REQUIRE(
      (1.2 -
        sync_rmi(right, get_rmi_handle(), &p_test::identity<long double>, 1.2))
          < std::numeric_limits<long double>::epsilon());

    rmi_fence(); // quiescence before next test
  }

  void test_stack_dynamic_object(void)
  {
    const unsigned int myid = this->get_location_id();
    stack_object oL(myid);
    dynamic_object oD(myid);

    // Test if user-defined types are really copy-by-value (i.e., argument
    // modifications within the RMI function aren't seen after the RMI).
    stack_object testL(myid);
    async_rmi(right, get_rmi_handle(), &p_test::stack_call_p, &oL);
    async_rmi(right, get_rmi_handle(), &p_test::stack_call_r, oL);
    async_rmi(right, get_rmi_handle(), &p_test::stack_call_v, oL);
    rmi_fence(); // wait for async_rmi calls
    STAPL_RUNTIME_TEST_CHECK(oL, testL);

    dynamic_object testD(myid);
    async_rmi(left, get_rmi_handle(), &p_test::dynamic_call_r, oD);
    async_rmi(left, get_rmi_handle(), &p_test::dynamic_call_p, &oD);
    async_rmi(left, get_rmi_handle(), &p_test::dynamic_call_v, oD);
    rmi_fence(); // wait for async_rmi calls
    STAPL_RUNTIME_TEST_CHECK(oD, testD);

    // Test if return values are transferred correctly
    {
      const dynamic_object d =
        sync_rmi(right, get_rmi_handle(), &p_test::dynamic_return, &oD);
      STAPL_RUNTIME_TEST_CHECK(d, testD);
    }

    rmi_fence(); // quiescence before next test

    {
      const stack_object s =
        sync_rmi(left, get_rmi_handle(), &p_test::stack_return, &oL);
      STAPL_RUNTIME_TEST_CHECK(s, testL);
    }

    rmi_fence(); // quiescence before next test

    {
      const stack_object ts(left);
      const stack_object s =
        sync_rmi(left, get_rmi_handle(), &p_test::stack_return0);
      STAPL_RUNTIME_TEST_CHECK(s, ts);
    }

    rmi_fence(); // quiescence before next test

    {
      const dynamic_object td(right);
      const dynamic_object d =
        sync_rmi(right, get_rmi_handle(), &p_test::dynamic_return0);
      STAPL_RUNTIME_TEST_CHECK(d, td);
    }

    rmi_fence(); // quiescence before next test

    // Test complex aggregate types
    if (myid % 2 == 0) {
      const int cSize = myid + 1;
      complex_object oC(cSize);
      for (int i = 0; i < cSize; ++i) {
        oC.l.insert_front(oD);
        oC.lP->insert_front(oL);
      }
      const complex_object t13 =
        sync_rmi(right, get_rmi_handle(), &p_test::complex_type, oC);
      STAPL_RUNTIME_TEST_CHECK(t13, oC);
    }

    rmi_fence(); // quiescence before next test
  }

  void test_offset_pointers(void)
  {
    const unsigned int myid = this->get_location_id();
    if (myid % 2 == 0) {
      const int vSize = 100;
      test_vector2<test_vector2<double> > v(vSize);
      for (int i = 1; i < vSize; ++i) {
        v[i].resize(i);
        for (int j = 0; j < i; ++j)
          v[i][j] = myid;
      }
      async_rmi(right, get_rmi_handle(), &p_test::offset_pointers, vSize, v);
    }

    rmi_fence(); // quiescence before next test
  }

  void test_dynamically_linked(void)
  {
    const unsigned int myid = this->get_location_id();
    if (myid % 2 == 0) {
      const int lSize = 10;
      test_list<int> l;
      for (int i = 0; i < lSize; i++)
        l.insert_front(i);
      async_rmi(left, get_rmi_handle(), &p_test::dynamic_types, lSize,
                       l);
      l.insert_front(lSize);
      l.insert_front(lSize + 1);
      l.insert_front(lSize + 2);
      async_rmi(right, get_rmi_handle(), &p_test::dynamic_types, lSize + 3, l);
    }

    rmi_fence(); // quiescence before next test
  }

  void test_tuple(void)
  {
    std::tuple<int, int, int> t(1, 2, 3);
    async_rmi(left, get_rmi_handle(), &p_test::tuple_type, t);

    rmi_fence(); // quiescence before next test

    int x = 1;
    int y = 2;
    int z = 3;
    std::tuple<int*, int*, int*> tup1(&x, &y, &z);
    async_rmi(left, get_rmi_handle(), &p_test::tuple_type_ptr, tup1);

    rmi_fence(); // quiescence before next test
  }

  void test_vector(void)
  {
    const unsigned int myid = this->get_location_id();
    if (myid > 0) {
      std::vector<double> vb;
      unsigned int i = 0;

      while (i < 100) {
        vb.push_back(double(i));
        async_rmi(left, get_rmi_handle(), &p_test::dvector_type, vb,
                         i + 1);
        ++i;
      }

      vb = sync_rmi(left, get_rmi_handle(), &p_test::dvector_sync_type);
      STAPL_RUNTIME_TEST_CHECK(vb.size(), 100);
      i = 0;
      while (i < vb.size()) {
        STAPL_RUNTIME_TEST_REQUIRE(
          vb[i] - i < std::numeric_limits<double>::epsilon());
        ++i;
      }
    }

    rmi_fence(); // quiescence before next test
  }

  void test_bitvector(void)
  {
    const unsigned int myid = this->get_location_id();
    if (myid > 0) {
      std::vector<bool> vb;
      bool b = true;
      unsigned int i = 0;
      while (i < 100) {
        vb.push_back(b);
        async_rmi(left, get_rmi_handle(), &p_test::bvector_type, vb, i + 1);
        b = !b;
        ++i;
      }
      vb = sync_rmi(left, get_rmi_handle(), &p_test::bvector_sync_type);
      STAPL_RUNTIME_TEST_CHECK(vb.size(), 100);
      i = 0;
      b = true;
      while (i < vb.size()) {
        STAPL_RUNTIME_TEST_CHECK(vb[i], b);
        b = !b;
        ++i;
      }
    }

    rmi_fence(); // quiescence before next test
  }

  void test_polymorphic(void)
  {
    // Test pointer to base pointer with polymorphic behaviour
    A* a_ptr1 = new B();
    async_rmi(left, get_rmi_handle(), &p_test::poly_type, a_ptr1);

    rmi_fence(); // quiescence before next test

    // Test polymorphic with combining...
    A* a_ptr2 = new C();
    async_rmi(left, get_rmi_handle(), &p_test::poly_type, a_ptr2);
    async_rmi(left, get_rmi_handle(), &p_test::poly_type, a_ptr1);
    async_rmi(left, get_rmi_handle(), &p_test::poly_type, a_ptr2);
    async_rmi(left, get_rmi_handle(), &p_test::poly_type, a_ptr2);
    async_rmi(left, get_rmi_handle(), &p_test::poly_type, a_ptr1);

    rmi_fence(); // quiescence before next test

    std::tuple<A*, A*> tup2(a_ptr1, a_ptr2);
    async_rmi(left, get_rmi_handle(), &p_test::poly_tuple_type, tup2);

    rmi_fence(); // quiescence before next test

    delete a_ptr1;
    delete a_ptr2;

    rmi_fence(); // quiescence before next test
  }

  void test_move(void)
  {
    std::vector<int> v(10, 42.0);
    async_rmi(left, get_rmi_handle(), &p_test::vector_move, std::move(v));

    rmi_fence(); // quiescence before next test
  }

  void test_vector_shared_ptr(void)
  {
    std::shared_ptr<std::vector<int>> p{new std::vector<int>(1024 * 1024, 42)};
    std::vector<std::shared_ptr<std::vector<int>>> v{20, p};

    int val = 0;
    for (auto const& p : v)
      for (auto const& i : *p)
        val += i;

    async_rmi(right, this->get_rmi_handle(),
              &p_test::consume_vec_shared_ptr, v, val);
    async_rmi(right, this->get_rmi_handle(),
              &p_test::copy_consume_vec_shared_ptr, v, val);
    async_rmi(right, this->get_rmi_handle(),
              &p_test::move_consume_vec_shared_ptr, std::move(v), val);

    rmi_fence(); // quiescence before next test
  }

  void test_deferred(void)
  {
    deferred_get_location_id d;
    async_rmi(left, get_rmi_handle(), &p_test::deferred_check, d);

    rmi_fence(); // quiescence before next test
  }

  void test_ptr_conversion(void)
  {
    int i = 42;
    async_rmi(left, get_rmi_handle(),
              &p_test::ptr_check<int>, to_ptr<int>{i}, i);

    async_rmi(left, get_rmi_handle(),
              &p_test::const_ptr_check<int>, to_const_ptr<int>{i}, i);

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_fundamental();
    test_stack_dynamic_object();
    test_offset_pointers();
    test_dynamically_linked();
    test_tuple();
    test_vector();
    test_bitvector();
    test_polymorphic();
    test_move();
    test_vector_shared_ptr();
    test_deferred();
    test_ptr_conversion();
  }
};


exit_code stapl_main(int, char*[])
{
  p_test pt;
  pt.execute();
#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}
