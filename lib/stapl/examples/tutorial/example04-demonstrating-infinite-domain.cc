/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <utility>
#include <iostream>
#include <stapl/utility/do_once.hpp>
#include <stapl/domains/continuous.hpp>

stapl::exit_code stapl_main(int argc, char **argv)
{

  // Creating an infinite string domain, two initialized continuous
  // double domains and one uninitialized continuous domain.
  stapl::continuous_domain<std::string> infinite_string("a", "z");
  stapl::continuous_domain<double> continuous_double(0.0, 1.0);
  stapl::continuous_domain<double> continuous_double_2(0.25, 1.75);
  stapl::continuous_domain<double> intersection;

  // Intersection of continuous_double and continuous_double_2 domains.
  // Note the "&" operator for intersection of domains.
  intersection = continuous_double & continuous_double_2;

  // Print domains information. The use of "&" in the lambda function
  // captures all variables used in the lambda body by reference.
  // With this, it is not necessary to add the variables in stapl::do_once.
  stapl::do_once(
    [&]{

      // Print the domain. Note the overloaded operator "<<" for
      // domains.
      std::cout << "infinite_string domain: " << infinite_string
                << std::endl;
      std::cout << "continuous_double domain: " << continuous_double
                << std::endl;

      // Print sizes of infinite_string and continuous_double domains.
      std::cout << "size of infinite_string domain: "
                << infinite_string.size() << std::endl;
      std::cout << "size of continuous_double domain: "
                << continuous_double.size() << std::endl;

      // Print some examples of domain elements.
      std::cout << "infinite_string contains \"ab\"? "
                << infinite_string.contains("ab") << std::endl;
      std::cout << "infinite_string contains \"any word\"? "
                << infinite_string.contains("any word") << std::endl;
      std::cout << "infinite_string contains "
                << "\"Honorificabilitudinitatibus\"? "
                << infinite_string.contains("Honorificabilitudinitatibus")
                << std::endl;
      std::cout << "infinite_string contains \"@#?%!\"? "
                << infinite_string.contains("@#?%!") << std::endl;
      std::cout << "continuous_double contains 0.00324? "
                << continuous_double.contains(0.00324) << std::endl;
      std::cout << "continuous_double contains 1.00324? "
                << continuous_double.contains(1.00324) << std::endl;
      std::cout << "intersection of " << continuous_double << " and "
                << continuous_double_2 << " is: " << intersection
                << std::endl;
    }
  );

  return EXIT_SUCCESS;
}
