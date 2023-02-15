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
/// This test verifies the transmission of @c std::bind() expressions by binding
/// member function pointers with 0-8 arguments. It stresses both @c std::bind()
/// and regular function object packing.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"
#include <functional>
#include <boost/bind.hpp>


using namespace stapl;

struct fu
{
  int test0(void)
  {
    return 0;
  }

  int test1(int a1)
  {
    STAPL_RUNTIME_TEST_CHECK(a1, 1);
    return 1;
  }

  int test2(int a1, int a2)
  {
    STAPL_RUNTIME_TEST_CHECK((a1==1 && a2==2), true);
    return 2;
  }

  int test3(int a1, int a2, int a3)
  {
    STAPL_RUNTIME_TEST_CHECK((a1==1 && a2==2 && a3 == 3), true);
    return 3;
  }

  int test4(int a1, int a2, int a3, int a4)
  {
    STAPL_RUNTIME_TEST_CHECK((a1==1 && a2==2 && a3 == 3 && a4==4), true);
    return 4;
  }

  int test5(int a1, int a2, int a3, int a4, int a5)
  {
    STAPL_RUNTIME_TEST_CHECK((a1==1 && a2==2 && a3 == 3 && a4==4 && a5==5),
                             true);
    return 5;
  }

  int test6(int a1, int a2, int a3, int a4, int a5, int a6)
  {
    STAPL_RUNTIME_TEST_CHECK((a1==1 && a2==2 && a3 == 3 && a4==4 && a5==5 &&
                              a6==6),
                             true);
    return 6;
  }

  int test7(int a1, int a2, int a3, int a4, int a5, int a6, int a7)
  {
    STAPL_RUNTIME_TEST_CHECK((a1==1 && a2==2 && a3 == 3 && a4==4 && a5==5 &&
                              a6==6 && a7==7),
                             true);
    return 7;
  }

  int test8(int a1, int a2, int a3, int a4, int a5, int a6, int a7, int a8)
  {
    STAPL_RUNTIME_TEST_CHECK((a1==1 && a2==2 && a3 == 3 && a4==4 && a5==5 &&
                              a6==6 && a7==7 && a8==8),
                             true);
    return 8;
  }
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

  template<typename Functor>
  void bind_async(Functor const& f)
  {
    fu tmp;
    f(tmp);
  }

  template<typename Functor>
  int bind_sync(Functor const& f)
  {
    fu tmp;
    return f(tmp);
  }

  template<typename Functor>
  void async_exec(Functor const& f)
  {
    async_rmi(right, get_rmi_handle(), &p_test::bind_async<Functor>, f);
  }

  template<typename Functor>
  int sync_exec(Functor const& f)
  {
    return sync_rmi(right, get_rmi_handle(), &p_test::bind_sync<Functor>,
                           f);
  }

  void execute(void)
  {
    using std::bind;
    using std::placeholders::_1;

    const int a1 = 1;
    const int a2 = 2;
    const int a3 = 3;
    const int a4 = 4;
    const int a5 = 5;
    const int a6 = 6;
    const int a7 = 7;
    const int a8 = 8;

    async_exec(bind(&fu::test0, _1));
    async_exec(bind(&fu::test1, _1, a1));
    async_exec(bind(&fu::test2, _1, a1, a2));
    async_exec(bind(&fu::test3, _1, a1, a2, a3));
    async_exec(bind(&fu::test4, _1, a1, a2, a3, a4));
    async_exec(bind(&fu::test5, _1, a1, a2, a3, a4, a5));
    async_exec(bind(&fu::test6, _1, a1, a2, a3, a4, a5, a6));
    async_exec(bind(&fu::test7, _1, a1, a2, a3, a4, a5, a6, a7));
    async_exec(bind(&fu::test8, _1, a1, a2, a3, a4, a5, a6, a7, a8));

    STAPL_RUNTIME_TEST_CHECK(0, sync_exec(bind(&fu::test0, _1)));
    STAPL_RUNTIME_TEST_CHECK(1, sync_exec(bind(&fu::test1, _1, a1)));
    STAPL_RUNTIME_TEST_CHECK(2, sync_exec(bind(&fu::test2, _1, a1, a2)));
    STAPL_RUNTIME_TEST_CHECK(3, sync_exec(bind(&fu::test3, _1,
                                               a1, a2, a3)));
    STAPL_RUNTIME_TEST_CHECK(4, sync_exec(bind(&fu::test4, _1,
                                               a1, a2, a3, a4)));
    STAPL_RUNTIME_TEST_CHECK(5, sync_exec(bind(&fu::test5, _1,
                                               a1, a2, a3, a4, a5)));
    STAPL_RUNTIME_TEST_CHECK(6, sync_exec(bind(&fu::test6, _1,
                                               a1, a2, a3, a4, a5, a6)));
    STAPL_RUNTIME_TEST_CHECK(7, sync_exec(bind(&fu::test7, _1,
                                               a1, a2, a3, a4, a5, a6, a7)));
    STAPL_RUNTIME_TEST_CHECK(8,
                             sync_exec(bind(&fu::test8, _1,
                                            a1, a2, a3, a4, a5, a6, a7, a8)));

    rmi_fence(); // wait for all RMIs to finish before exiting
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
