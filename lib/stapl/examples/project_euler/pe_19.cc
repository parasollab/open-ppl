/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/views/counting_view.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <iostream>
#include <stdlib.h>
#include "../../test/confint.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;

class sundays_in_month
{
  const int year;

  //dayofweek posted by Tomohiko Sakamoto
  //on the comp.lang.c Usenet newsgroup in 1993
  // 1 <= m <= 12,  y > 1752 (in the U.K.)
  int dayofweek(int y, int m, int d) const
  {
    static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    y -= m < 3;
    //returns 0 = Sunday, 1 = Monday, ..
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
  }

public:
  typedef int result_type;

  //year is mandatory
  sundays_in_month() = delete;

  sundays_in_month(int y)
    : year(y)
  {}

  int operator()(int month) const
  {
    //we're only testing the first Sunday
    if(dayofweek(year, month, 1) == 0)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  //needed for STAPL to be able to move this job to different processors
  void define_type(stapl::typer& t)
  {
   //year is data member
   t.member(year);
  }

};

struct sundays_in_year
{
  typedef int result_type;

  int operator() (int year) const
  {
    auto months = stapl::counting_view(12, 1);
    return stapl::map_reduce(sundays_in_month(year),
             stapl::plus<int>(), months);
  }
};


stapl::exit_code stapl_main(int argc, char **argv)
{
  const int samples = 64;
  int start_year = 1901;
  int end_year = 2000;
  confidence_interval_controller controller(samples, samples, 0.05);
  counter_t count;
  std::stringstream report;
  report << "Test: Euler19\n"
    << "Version: STAPL\n";
  while(controller.iterate())
  {
    if(argc == 3)
    {
      start_year = atoi(argv[1]);
      end_year = atoi(argv[2]);
      if(start_year < 1752 || end_year < start_year)
      {
        stapl::do_once([&]()
        {
          std::cerr<<"Date input error. You entered start_year: "
                    << start_year << " and end_year: " << end_year <<std::endl;
        });
        return EXIT_SUCCESS;
      }
    }
    else
    {
      stapl::do_once([&]()
      {
          std::cout<<"Using default value start_year: "
                    << start_year << " and end_year: " << end_year <<std::endl;
      });
    }
    count.reset();
    count.start();
    auto years = stapl::counting_view((end_year-start_year) + 1, start_year);
    int total = stapl::map_reduce(sundays_in_year(), stapl::plus<int>(), years);
    count.stop();
    controller.push_back(count.value());
    stapl::do_once([&]()
    {
      std::cout << "Total number of sundays is: " << total << std::endl;
    });
  }
  stapl::do_once([&]()
  {
    controller.report(report);
    std::cout << report.str();
  });
  return EXIT_SUCCESS;
}
