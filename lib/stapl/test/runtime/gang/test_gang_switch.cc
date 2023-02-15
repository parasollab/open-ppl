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
/// Test switching between gangs.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/utility/functional.hpp>
#include <iostream>
#include "../test_utils.h"

using namespace stapl;

struct simple_object
: public p_object
{
  void restore_gang(void)
  {
    gang g{*this};
    STAPL_RUNTIME_TEST_REQUIRE(
      this->get_num_locations()==stapl::get_num_locations());
  }
};

struct p_test
: public p_test_object
{
  // creates a new gang and an object into it and switches back to p_test's gang
  void test_one_switch(void)
  {
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==this->get_location_md());

    gang g;

    simple_object o;
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==o.get_location_md());

    gang gs{*this};
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=o.get_location_md());
  }

  // creates a new gang and an object into it and switches back and forth
  // between gangs
  void test_two_switches(void)
  {
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==this->get_location_md());

    gang g;

    simple_object o;
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==o.get_location_md());

    gang gs1{*this};
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=o.get_location_md());

    gang gs2{o};
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==o.get_location_md());
  }

  // creates a new gang and an object into it and switches back and forth
  // between gangs
  void test_leave(void)
  {
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==this->get_location_md());

    gang g;

    simple_object o;
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==o.get_location_md());

    g.leave();

    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=o.get_location_md());

    gang gs{o};
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()!=this->get_location_md());
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==o.get_location_md());
  }

  // creates a gang over all locations, makes it inactive and switches into it
  void test_inactive_switch(void)
  {
    const unsigned int lid      = stapl::get_location_id();
    const unsigned int num_locs = stapl::get_num_locations();

    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md()==this->get_location_md());

    simple_object* o = nullptr;

    // create gang
    {
      typedef runtime::identity<unsigned int, unsigned int> map_fun_type;
      gang g{num_locs, map_fun_type{}, map_fun_type{}};
      STAPL_RUNTIME_TEST_CHECK(num_locs, stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(lid, stapl::get_location_id());
      o = new simple_object;
    }

    STAPL_RUNTIME_TEST_CHECK(num_locs, o->get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(lid, o->get_location_id());

    // switch into
    {
      gang g{*o};
      STAPL_RUNTIME_TEST_CHECK(num_locs, stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(lid, stapl::get_location_id());
    }

    delete o;
  }

  void execute(void)
  {
    test_one_switch();
    test_two_switches();
    test_leave();
    test_inactive_switch();
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
