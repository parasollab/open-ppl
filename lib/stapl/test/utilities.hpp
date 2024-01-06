/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_TEST_UTILITIES_HPP
#define STAPL_TEST_UTILITIES_HPP

#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include "./algorithms/test_report.h"

//////////////////////////////////////////////////////////////////////
/// @brief Parse the options on the command line and populate a map of
/// the values provided for each option.
/// @return variables_map is a map from the option (e.g., -t) to the
/// value provided after it (if any)
//////////////////////////////////////////////////////////////////////
boost::program_options::variables_map
get_opts(int argc, char **argv)
{
  try {
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
      ("help,h", "Print this help message")
      ("data,d",
       boost::program_options::value<std::vector<std::string> >(),
       "Data set")
      ("list,l", "Print tests provided")
      ("noref", "Disable verification with reference")
      ("out,o", "Output file name")
      ("test,t", boost::program_options::value<std::vector<std::string> >(),
       "Test to run")
      ("version,v", boost::program_options::value<std::vector<std::string> >(),
       "Test class to run")
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(
      boost::program_options::command_line_parser(argc, argv).options(desc)
      .style(boost::program_options::command_line_style::default_style |
             boost::program_options::command_line_style::allow_long_disguise)
      .run(), vm);
    boost::program_options::notify(vm);

    return vm;
  }
  catch(boost::program_options::unknown_option opt) {
    test_error(argv[0], opt.what());
  }
  catch(...) {
    test_error(argv[0], ": exception thrown");
  }
}
#endif
