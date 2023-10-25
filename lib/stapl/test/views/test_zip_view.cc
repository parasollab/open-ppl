/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/zip_view.hpp>

#include "../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  int n = atoi(argv[1]);

  typedef result_of::counting_view<int>::type ciview_t;
  typedef result_of::counting_view<double>::type cdview_t;
  typedef zip_view<ciview_t,cdview_t>     zview_t;

  zview_t zv(counting_view<int>(n),counting_view<double>(n,100));

  const double epsilon = 0.000001;

  std::vector<double> res2(2,0.0);
  std::vector<double> expected2(2,0.0);
  expected2[0] = static_cast<double>(n-1)*n*(2*n-1)/6;
  expected2[1] = static_cast<double>(n-1)*n*(2*n-1)/6 + 100*(n-1)*n/2;

  res2 = inner_product(counting_view<double>(n),zv,res2);

  bool correct2 =
         (res2[0] - expected2[0] < epsilon) &&
         (res2[1] - expected2[1] < epsilon);

  std::vector<double> res4(4,0.0);
  std::vector<double> expected4(4,0.0);
  expected4[0] = static_cast<double>(n-1)*n*(2*n-1)/6;
  expected4[1] = static_cast<double>(n-1)*n*(2*n-1)/6 + 10*(n-1)*n/2;
  expected4[2] = static_cast<double>(n-1)*n*(2*n-1)/6 + 50*(n-1)*n/2;
  expected4[3] = static_cast<double>(n-1)*n*(2*n-1)/6 + 100*(n-1)*n/2;

  res4 = inner_product(counting_view<double>(n),
                                       zip(counting_view<double>(n),
                                           counting_view<double>(n,10),
                                           counting_view<double>(n,50),
                                           counting_view<double>(n,100)),
                                       res4);
  bool correct4 =
         (res4[0] - expected4[0] < epsilon) &&
         (res4[1] - expected4[1] < epsilon) &&
         (res4[2] - expected4[2] < epsilon) &&
         (res4[3] - expected4[3] < epsilon);

  STAPL_TEST_REPORT(correct2 && correct4,"Testing zip_view with inner_product");
  if (!correct2)
  {
    printf("res2 == %f %f, expected == %f %f\n", res2[0], res2[1],
           expected2[0], expected2[1]);
  }
  if (!correct4)
  {
    printf("res4 == %f %f %f %f, expected == %f %f %f %f\n", res4[0], res4[1],
           res4[2], res4[3], expected4[0], expected4[1], expected4[2],
           expected4[3]);
  }

  return EXIT_SUCCESS;
}
