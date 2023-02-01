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
/// Test transporting of @ref p_object references.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"


using namespace stapl;


class simple_p_object
: public p_object
{
private:
  unsigned int m_neighbor; // left neighbor

public:
  simple_p_object(void)
  : m_neighbor((get_location_id()==0) ? get_num_locations() - 1
                                      : get_location_id() - 1)
  { this->advance_epoch(); }

  unsigned int neighbor(void) const
  { return m_neighbor; }

  int test(void) const
  {
    STAPL_RUNTIME_TEST_CHECK(stapl::get_location_id(), this->get_location_id());
    return 0;
  }
};


template<int N>
class v_p_object
: public p_object
{
private:
  unsigned int m_neighbor; // left neighbor

public:
  v_p_object(void)
  : m_neighbor((get_location_id()==0) ? get_num_locations() - 1
                                      : get_location_id() - 1)
  { this->advance_epoch(); }

  unsigned int neighbor(void) const
  { return m_neighbor; }

  virtual int get(void) const
  { return N; }

  int test(void) const
  {
    STAPL_RUNTIME_TEST_CHECK(stapl::get_location_id(), this->get_location_id());
    return get();
  }
};


template<typename T>
class intermediate_p_object
: public virtual v_p_object<42>
{
#if (__GNUC__ == 4) && (__GNUC_MINOR__ == 8) && (__GNUC_PATCHLEVEL__ < 3)
  // work-around for gcc 4.8.1/4.8.2 bug
  // more on: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=57319
  intermediate_p_object& operator=(intermediate_p_object const&) = delete;
#endif
};


class complex_p_object
: public intermediate_p_object<int>, intermediate_p_object<double>
{
public:
  typedef void virtual_base;

private:
  unsigned int m_neighbor; // left neighbor

public:
  complex_p_object(void)
  : m_neighbor((get_location_id()==0) ? get_num_locations() - 1
                                      : get_location_id() - 1)
  { }

  unsigned int neighbor(void) const
  { return m_neighbor; }

  int test(void) const
  {
    STAPL_RUNTIME_TEST_CHECK(stapl::get_location_id(), this->get_location_id());
    return get();
  }
};


template<typename T>
class ptr_to_p_object
{
private:
  int          m_magic1;
  T*           m_p;
  int          m_magic2;
  unsigned int m_id;
  int          m_magic3;
  int          m_i;
  int          m_magic4;

public:
  ptr_to_p_object(T* p, unsigned int id, int i)
  : m_magic1(7070),
    m_p(p),
    m_magic2(7070),
    m_id(id),
    m_magic3(7070),
    m_i(i),
    m_magic4(7070)
  { }

  void test(void) const
  {
    STAPL_RUNTIME_TEST_CHECK(m_magic1, 7070);
    STAPL_RUNTIME_TEST_CHECK(m_magic2, 7070);
    STAPL_RUNTIME_TEST_CHECK(m_magic3, 7070);
    STAPL_RUNTIME_TEST_CHECK(m_magic4, 7070);
    STAPL_RUNTIME_TEST_CHECK(m_p->test(), m_i);
    STAPL_RUNTIME_TEST_CHECK(m_id, m_p->get_location_id());
  }

  void define_type(typer& t)
  {
    t.member(m_magic1);
    t.member(m_p);
    t.member(m_magic2);
    t.member(m_id);
    t.member(m_magic3);
    t.member(m_i);
    t.member(m_magic4);
  }
};


#if 0
// TODO - not yet supported in typer
class ref_to_p_object
{
private:
  unsigned int     sender_id;
  int              magic;
  simple_p_object& p;
  unsigned int     id;

public:
  ref_to_p_object(unsigned int sender, simple_p_object& o, unsigned int i)
  : sender_id(sender),
    magic(7070),
    p(o),
    id(i)
  { }

  void test(void)
  {
    p.test();
    STAPL_RUNTIME_TEST_CHECK((sender_id!=p->get()), true);
    STAPL_RUNTIME_TEST_CHECK(id, p->get());
    STAPL_RUNTIME_TEST_CHECK(magic, 7070);
  }

  void define_type(typer& t)
  {
    t.member(sender_id);
    t.member(magic);
    t.member(p);
    t.member(id);
  }
};
#endif


class p_test
: public p_object
{
private:
  unsigned int m_neighbor; // right neighbor

public:
  p_test(void)
  : m_neighbor((get_location_id()==0) ? get_num_locations() - 1
                                      : get_location_id() - 1)
  { this->advance_epoch(); }

  p_test* get_ptr(void)
  { return this; }

  // this is a runtime error, p_object copying is not allowed for now
  template<typename T>
  void test_b(T)
  { }

  template<typename T>
  void test_r(T& t, int i)
  {
    STAPL_RUNTIME_TEST_CHECK(t.test(), i);
  }

  template<typename T>
  void test_p(T* p, int i)
  {
    if (p)
      STAPL_RUNTIME_TEST_CHECK(p->test(), i);
  }

  template<typename T>
  void test_pc(T* const p, int i)
  {
    if (p)
      STAPL_RUNTIME_TEST_CHECK(p->test(), i);
  }

  void test_get_ptr(void)
  {
    p_test* p = sync_rmi(m_neighbor, this->get_rmi_handle(), &p_test::get_ptr);
    STAPL_RUNTIME_TEST_CHECK(p, this);

    rmi_fence(); // quiescence before next test
  }

  // tests getting the object from the handle locally
  template<typename T>
  void test_local(void)
  {
    T t;
    t.test();

    rmi_handle::reference r = t.get_rmi_handle();
    T& tr = get_p_object<T>(r);
    STAPL_RUNTIME_TEST_CHECK(&tr, &t);

    rmi_handle::light_reference lr = t.get_rmi_handle();
    T& tlr = get_p_object<T>(lr);
    STAPL_RUNTIME_TEST_CHECK(&tlr, &t);

    rmi_fence(); // quiescence before next test
  }

  // test passing directly a p_object to a member function
  template<typename T>
  void test_direct(void)
  {
    T t;
    int v = t.test();
    rmi_fence(); // wait for all locations to finish calling test()

#if 0
    // cannot send p_object by value
    // TODO - not supported in typer
    for (int i=0; i<10; ++i) {
      async_rmi(neighbor, this->get_rmi_handle(), &p_test::test_b, o);
    }
#endif
    for (int i=0; i<10; ++i) {
      async_rmi(m_neighbor, this->get_rmi_handle(),
                &p_test::template test_p<T>, &t, v);
      async_rmi(m_neighbor, this->get_rmi_handle(),
                &p_test::template test_p<T>, nullptr, v);
      async_rmi(m_neighbor, this->get_rmi_handle(),
                &p_test::template test_pc<T>, &t, v);
      async_rmi(m_neighbor, this->get_rmi_handle(),
                &p_test::template test_pc<T>, nullptr, v);
    }
    for (int i=0; i<10; ++i) {
      async_rmi(m_neighbor, this->get_rmi_handle(),
                &p_test::template test_r<T>, t, v);
    }

    rmi_fence(); // quiescence before next test
  }

  template<typename T>
  void test_ptr_b(T t)
  { t.test();  }

  template<typename T>
  void test_ptr_r(T& t)
  { t.test();  }

  // Using a p_object* from an ordinary object
  template<typename T>
  void test_ptr(void)
  {
    T o;
    ptr_to_p_object<T> b(&o, m_neighbor, o.test());
    rmi_fence(); // wait for all locations to call test()

    for (int i=0; i<10; ++i) {
      async_rmi(m_neighbor, this->get_rmi_handle(),
                &p_test::template test_ptr_b<ptr_to_p_object<T>>, b);
    }
    for (int i=0; i<10; ++i) {
      async_rmi(m_neighbor, this->get_rmi_handle(),
                &p_test::template test_ptr_r<ptr_to_p_object<T>>, b);
    }

    rmi_fence(); // quiescence before next test
  }

  template<typename T>
  void test_ref(void)
  {
    T o;

    rmi_handle::reference r = o.get_rmi_handle();
    STAPL_RUNTIME_TEST_CHECK(r, o.get_rmi_handle());
    STAPL_RUNTIME_TEST_CHECK(o.get_rmi_handle(), r);
    STAPL_RUNTIME_TEST_REQUIRE(this->get_rmi_handle()!=r);
    STAPL_RUNTIME_TEST_REQUIRE(r!=this->get_rmi_handle());

    rmi_handle::const_reference cr = o.get_rmi_handle();
    STAPL_RUNTIME_TEST_CHECK(cr, o.get_rmi_handle());
    STAPL_RUNTIME_TEST_CHECK(o.get_rmi_handle(), cr);
    STAPL_RUNTIME_TEST_REQUIRE(this->get_rmi_handle()!=cr);
    STAPL_RUNTIME_TEST_REQUIRE(cr!=this->get_rmi_handle());

    rmi_handle::light_reference l = o.get_rmi_handle();
    STAPL_RUNTIME_TEST_CHECK(l, o.get_rmi_handle());
    STAPL_RUNTIME_TEST_CHECK(o.get_rmi_handle(), l);
    STAPL_RUNTIME_TEST_REQUIRE(this->get_rmi_handle()!=l);
    STAPL_RUNTIME_TEST_REQUIRE(l!=this->get_rmi_handle());

    rmi_handle::const_light_reference cl = o.get_rmi_handle();
    STAPL_RUNTIME_TEST_CHECK(cl, o.get_rmi_handle());
    STAPL_RUNTIME_TEST_CHECK(o.get_rmi_handle(), cl);
    STAPL_RUNTIME_TEST_REQUIRE(this->get_rmi_handle()!=cl);
    STAPL_RUNTIME_TEST_REQUIRE(cl!=this->get_rmi_handle());

    STAPL_RUNTIME_TEST_CHECK(r, r);
    STAPL_RUNTIME_TEST_CHECK(r, cr);
    STAPL_RUNTIME_TEST_CHECK(r, l);
    STAPL_RUNTIME_TEST_CHECK(r, cl);

    STAPL_RUNTIME_TEST_CHECK(cr, r);
    STAPL_RUNTIME_TEST_CHECK(cr, cr);
    STAPL_RUNTIME_TEST_CHECK(cr, l);
    STAPL_RUNTIME_TEST_CHECK(cr, cl);

    STAPL_RUNTIME_TEST_CHECK(l, r);
    STAPL_RUNTIME_TEST_CHECK(l, cr);
    STAPL_RUNTIME_TEST_CHECK(l, l);
    STAPL_RUNTIME_TEST_CHECK(l, cl);

    STAPL_RUNTIME_TEST_CHECK(cl, r);
    STAPL_RUNTIME_TEST_CHECK(cl, cr);
    STAPL_RUNTIME_TEST_CHECK(cl, l);
    STAPL_RUNTIME_TEST_CHECK(cl, cl);

    rmi_fence(); // quiescence before next test
  }

#if 0
  // Using a p_object& from an ordinary object
  // TODO - not supported in typer
  void test2_b(ref_to_p_object t)
  { t.test();  }

  void test2_r(ref_to_p_object& t)
  { t.test(); }

  void test2(void)
  {
    simple_p_object o;
    ref_to_p_object b(id, o, rt);

    async_rmi(rt, this->get_rmi_handle(), &p_test::test2_b, b);
    async_rmi(rt, this->get_rmi_handle(), &p_test::test2_r, b);

    rmi_fence(); // quiescence before next test
  }
#endif

  void execute(void)
  {
    test_get_ptr();

    test_local<simple_p_object>();
    test_local<v_p_object<1> >();
    test_local<v_p_object<2> >();
    test_local<complex_p_object>();

    test_direct<simple_p_object>();
    test_direct<v_p_object<1> >();
    test_direct<v_p_object<2> >();
    test_direct<complex_p_object>();

    test_ptr<simple_p_object>();
    test_ptr<v_p_object<1> >();
    test_ptr<v_p_object<2> >();
    test_ptr<complex_p_object>();

    test_ref<simple_p_object>();
    test_ref<v_p_object<1> >();
    test_ref<v_p_object<2> >();
    test_ref<complex_p_object>();

#if 0
    test2();
#endif
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
