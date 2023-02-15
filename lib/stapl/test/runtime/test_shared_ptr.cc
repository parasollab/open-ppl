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
/// Test @c std::shared_ptr with @ref p_object.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <iostream>
#include <memory>
#include "test_utils.h"

using namespace stapl;

class dummy_base
: public p_object,
  public std::enable_shared_from_this<dummy_base>
{ };

class dummy
: public dummy_base
{
public:
  std::shared_ptr<dummy> shared_from_this(void)
  { return std::static_pointer_cast<dummy>(dummy_base::shared_from_this()); }
};


class p_test
: public p_object
{
private:
  unsigned int           m_right;
  std::shared_ptr<dummy> m_p;

public:
  p_test(void)
  : m_right(this->get_location_id()==(this->get_num_locations()-1)
            ? 0
            : this->get_location_id()+1)
  { this->advance_epoch(); }

  void recv_shared_ptr(std::shared_ptr<dummy> p)
  {
    STAPL_RUNTIME_TEST_CHECK(p.get(), m_p.get());
  }

  void execute(void)
  {
    m_p = std::make_shared<dummy>();

    rmi_fence();

    async_rmi(m_right, this->get_rmi_handle(), &p_test::recv_shared_ptr, m_p);

    rmi_fence();
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
