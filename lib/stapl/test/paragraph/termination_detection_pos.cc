/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <stapl/skeletons/map_sched.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/array_ro_view.hpp>

using namespace stapl;

template <typename C = view_impl::counting_container<std::size_t, 1,
  view_impl::default_container>,
          typename Dom = typename container_traits<C>::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>>
struct tester_view
  : array_ro_view<C, Dom, MapFunc>
{
  typedef C                                          view_container_type;
  typedef Dom                                        domain_type;
  typedef MapFunc                                    map_func_type;
  typedef array_ro_view<C, Dom, MapFunc>             base_type;

  tester_view() = default;

  tester_view(std::size_t num_components)
    : base_type(new view_container_type(
                num_components*get_num_locations(), (std::size_t) 0))
  { }

  tester_view(view_container_type const& vcont, domain_type const& dom,
              map_func_type mfunc=map_func_type(),
              tester_view const& other=tester_view<C>())
    : base_type(vcont, dom, mfunc)
  { }
};


struct empty_wf
{
  typedef void result_type;

  template<typename Reference>
  void operator()(Reference) const
  {
    std::this_thread::sleep_for(std::chrono::microseconds{rand() % 10000});
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (stapl::get_location_id() == 0)
  {
    std::cout << "paragraph termination detection positive with "
              << stapl::get_num_locations() << " locations... ";
    std::srand(std::time(NULL));
  }

  const int nelems = stapl::get_num_locations()*1000;

  stapl::rmi_fence();

  map_func_sched(stapl::default_scheduler(), empty_wf(), tester_view<>(nelems));

  // if we reach here then it means that the test passed
  if (stapl::get_location_id()==0)
    std::cout << "Passed" << std::endl;

  return EXIT_SUCCESS;
}
