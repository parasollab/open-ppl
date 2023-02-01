/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/random.hpp>

#include "../test_report.hpp"

class random_object
 : public stapl::p_object
{
  std::mt19937 m_std_gen;
  boost::random::mt19937 m_boost_gen;

public:
  random_object(void)
    : m_std_gen{this->get_location_id()}, m_boost_gen{this->get_location_id()}
  { }

  void send_twisters()
  {
    const auto loc = this->get_location_id();

    for (std::size_t l = 0; l < this->get_num_locations(); ++l)
    {
      stapl::async_rmi(
        l, this->get_rmi_handle(),
        &random_object::receive_twister<std::mt19937>, m_std_gen, loc
      );

      stapl::async_rmi(
        l, this->get_rmi_handle(),
        &random_object::receive_twister<boost::random::mt19937>, m_boost_gen,
        loc
      );
    }
  }

  template<typename Twister>
  void receive_twister(Twister gen, stapl::location_type source)
  {
    using fn_t = void (random_object::*)(Twister);
    fn_t fn = &random_object::compare_twister;

    stapl::async_rmi(source, this->get_rmi_handle(), fn, gen);
  }

  void compare_twister(std::mt19937 gen)
  {
    if (gen != m_std_gen)
      stapl::abort("std generator has differing internal state");
  }

  void compare_twister(boost::random::mt19937 gen)
  {
    if (gen != m_boost_gen)
      stapl::abort("Boost generator has differing internal state");
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  random_object r;
  r.send_twisters();

  STAPL_TEST_REPORT(true, "Generators serialized");

  return EXIT_SUCCESS;
}
