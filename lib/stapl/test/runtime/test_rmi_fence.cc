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
/// Test @ref stapl::rmi_fence().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"
#include <boost/lexical_cast.hpp>

using namespace stapl;

struct p_test
: public p_object
{
  unsigned int m_fence_count;
  // source and  destination must not be directly linked in the spanning tree of
  // the fence
  unsigned int m_src;
  unsigned int m_dst;

  p_test(const unsigned int src = 0,
         const unsigned int dst = (stapl::get_num_locations()-1))
  : m_fence_count(0),
    m_src(src % this->get_num_locations()),
    m_dst(dst % this->get_num_locations())
  {
    rmi_fence(); // initialize p_object and do the first fence
    ++m_fence_count;
  }

  void check_count(const unsigned int count)
  {
    // problem: the initial message was sent in the previous fence. As this is
    // the answer to the initiating  message, it results from the activity
    // initiated by the call to fence_bust(). It must be received before the
    // fence can pass.
    STAPL_RUNTIME_TEST_CHECK(m_fence_count, count);
  }

  void check_count2(const unsigned int src, const unsigned int count)
  {
    check_count(count);
    for (unsigned int i=0; i<2; ++i) {
      async_rmi(src, this->get_rmi_handle(),
                       &p_test::check_count, count);
      rmi_flush();
      delay(1);
    }
  }

  void fence_bust(void)
  {
    if (this->get_location_id()==m_src) {
      async_rmi(m_dst, this->get_rmi_handle(),
                &p_test::check_count2, m_src, m_fence_count);
      rmi_flush();
    }
    rmi_fence(); // first fence
    ++m_fence_count;
    rmi_fence(); // check if RMIs leaked from the first fence
    ++m_fence_count;
  }

  void execute(void)
  {
    fence_bust();
  }
};


exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    p_test pt;
    pt.execute();
  }
  else {
    p_test pt(boost::lexical_cast<unsigned int>(argv[1]),
              boost::lexical_cast<unsigned int>(argv[2]));
    pt.execute();
  }
#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}
