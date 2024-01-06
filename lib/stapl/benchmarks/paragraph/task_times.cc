/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <boost/program_options.hpp>
#include <test/algorithms/test_utils.h>
#include "../utilities/confint.hpp"

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/utility/do_once.hpp>

using namespace stapl;

struct my_wf
{
  typedef void result_type;

  result_type operator()(void) const
  { }
};


class my_task_factory
  : public task_factory_base
{
private:
  size_t m_num_local_tasks;

public:
  using result_type = void;

  my_task_factory(size_t num_local_tasks)
    : m_num_local_tasks(num_local_tasks)
  { }

  template <typename TGV>
  void operator()(TGV const& tgv)
  {
    for (size_t idx = 0; idx < m_num_local_tasks; ++idx)
      tgv.add_task(my_wf());

    this->m_finished = true;
  }

};


void estimate_resolution(void)
{
  confidence_interval_controller iter_control(32, 250, 0.025);

  counter_t timer;

  while (iter_control.iterate()) {
    timer.reset();
    timer.start();
    iter_control.push_back(timer.stop());
  }

  do_once([&](void) {
    std::cout << "Timer Result = " << iter_control.average() * 1e9
              << " +/- " << iter_control.confidence_interval() * 1e9
              << "\n";
  });
}


confidence_interval_controller
task_test(int num_tasks, int min_iter, int max_iter)
{
  typedef paragraph<default_scheduler, my_task_factory> paragraph_t;

  counter_t timer;

  confidence_interval_controller iter_control(min_iter, max_iter, 0.025);

  while (iter_control.iterate())
  {
    paragraph_t pg((my_task_factory(num_tasks)));

    rmi_fence();

    timer.reset();
    timer.start();

    pg();

    iter_control.push_back(timer.stop());
  }

  return iter_control;
}


exit_code stapl_main(int argc, char* argv[])
{
  std::cout << std::fixed;
  std::cout.precision(4);

  estimate_resolution();

  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Print this help message")
    ("min_iterations",
     boost::program_options::value<int>(),
     "Minimum iterations of tests to run")
    ("max_iterations",
     boost::program_options::value<int>(),
     "Maximum iterations of tests to run")
  ;

  boost::program_options::variables_map vm;
  boost::program_options::store(
    boost::program_options::command_line_parser(argc, argv).options(desc)
    .style(boost::program_options::command_line_style::default_style |
           boost::program_options::command_line_style::allow_long_disguise)
    .run(), vm);
  boost::program_options::notify(vm);

  const int min_iter =
    vm.count("min_iterations") ? vm["min_iterations"].as<int>() : 32;

  const size_t max_iter =
    vm.count("max_iterations") ? vm["max_iterations"].as<int>() : 200;

  const std::vector<int> task_counts =
    {{1, 2, 5, 10, 100, 1000, 10000, 25000, 50000, 100000}};

  std::vector<confidence_interval_controller> results;

  confidence_interval_controller zero_result = task_test(0, min_iter, max_iter);

  zero_result.report("empty paragraph");

  for (int n : task_counts)
    results.push_back(task_test(n, min_iter, max_iter));

  do_once([&](void) {
    std::cout << std::endl;
    std::cout << "=========================================================\n";
    std::cout << "Task Count | Average Task Time (ns) | CI\n";
    std::cout << "=========================================================\n";

    for (size_t i=0; i<task_counts.size(); ++i)
    {
      double avg_time = results[i].average() - zero_result.average();
      avg_time       *= 1e9;
      avg_time       /= task_counts[i];

      double avg_ci   = results[i].confidence_interval() / task_counts[i];
      avg_ci         *= 1e9;

      std::cout << std::setw(10) << task_counts[i]
                << std::setw(24) << avg_time
                << "            +/-" << std::setw(8) << avg_ci;

      if (results[i].iterations_run() == max_iter)
        std::cout << "  ***MAX_ITER REACHED***";

      std::cout << std::endl;
    }
  });

  return EXIT_SUCCESS;
}
