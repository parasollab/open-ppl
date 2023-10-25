/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cmath>
#include <iostream>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/utility/do_once.hpp>

struct empty_wf
{
  using result_type = void;

  result_type operator()(void) const
  { }
};


struct viewless_factory
  : public stapl::task_factory_base
{
  using result_type = void;

  template<typename TGV>
  void operator()(TGV const& tgv) const
  {
    tgv.add_task(tgv.graph().get_location_id(), empty_wf(), 0);
  }
};


stapl::exit_code stapl_main(int, char*[])
{
  for (int i =0; i<32; ++i)
  {
    auto pg = make_paragraph(viewless_factory());

    stapl::counter<stapl::default_timer> timer;

    timer.reset();
    timer.start();

    pg();

    double t = timer.stop();

    stapl::do_once([t, i] {
      std::cout << "Iteration " << i << " ==> " << t << "\n";
    });
  }

  return EXIT_SUCCESS;
}
