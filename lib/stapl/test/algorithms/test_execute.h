/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_ALGORITHMS_TEST_EXECUTE_H
#define STAPL_ALGORITHMS_TEST_EXECUTE_H

#include <cstdlib>
#include <cstring>
#include <string>
#include <functional>
#include <vector>
#if defined(TEST_P_MATRIX)
# include <cmath>
#endif
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wtautological-constant-out-of-range-compare"
#endif

#include <boost/property_tree/json_parser.hpp>

#ifdef __clang__
#pragma clang diagnostic pop
#endif

template<typename T>
struct test_pair
{
  char const* name;
  T test;
};

template<typename InputIterator>
void test_execute(int argc, char **argv,
                  InputIterator first, InputIterator last)
{
  using std::bind;
  using std::placeholders::_1;

  try {
    unsigned long int N = TEST_DEFAULT_CONTAINER_SIZE;
    unsigned int I = TEST_DEFAULT_ITERATION_COUNT;
    bool test_run = false;
    std::vector<std::string> tests;
    std::vector<traits_type::ct_t>
      cct(3, std::bind(allocate_balanced_container, _1));
    std::vector<traits_type::vw_t> cvt(3, std::bind(allocate_aligned_view, _1));

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
      ("I", boost::program_options::value<unsigned int>(),
       "Number of times test is repeated in timed section")
      ("N", boost::program_options::value<unsigned long int>(),
       "Number of elements")
      ("T", boost::program_options::value<std::vector<std::string> >(),
       "Test to run")
      ("pg", "Enable parallel input generation")
      ("po", "Run STAPL test only")
      ("cpart",
       boost::program_options::value<std::vector<std::string> >()->multitoken(),
       "Container partitions")
      ("cpart1",
       boost::program_options::value<std::vector<std::string> >()->multitoken(),
       "First container partition")
      ("cpart2",
       boost::program_options::value<std::vector<std::string> >()->multitoken(),
       "Second container partition")
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(
      boost::program_options::command_line_parser(argc, argv).options(desc)
      .style(boost::program_options::command_line_style::default_style |
             boost::program_options::command_line_style::allow_long_disguise)
      .run(), vm);
    boost::program_options::notify(vm);


    if (vm.count("help"))
    {
      std::cout << desc << "\n";
      return;
    }

    if (vm.count("list"))
    {
      // Boost.PropertyTree will only serialize a node with unnamed subkeys
      // as an array.  Therefore we've got to build extra trees in order to
      // create the unnamed subkeys needed to form proper JSON output.
      typedef boost::property_tree::ptree ptree;
      ptree stl_tree;
      ptree stapl_tree;
      stl_tree.push_back(std::make_pair("name", ptree("stapl")));
      stapl_tree.push_back(std::make_pair("name", ptree("stl")));
      ptree version_array;
      version_array.push_back(std::make_pair("", stl_tree));
      version_array.push_back(std::make_pair("", stapl_tree));

      ptree test_array;
      for (InputIterator it = first; it!=last; ++it)
      {
        ptree test;
        test.push_back(std::make_pair(std::string(it->name), version_array));
        test_array.push_back(std::make_pair("", test));
      }

      // The array of tests is the property of a key named "tests".
      ptree test_tree;
      test_tree.push_back(std::make_pair("tests", test_array));
      write_json(std::cout, test_tree, false);
      return;
    }

    std::vector<std::string> data_sizes;
    if (vm.count("data"))
      data_sizes = vm["data"].as<std::vector<std::string> >();

    if (vm.count("N"))
      N = vm["N"].as<unsigned long int>();

    if (vm.count("test"))
    {
      tests = vm["test"].as<std::vector<std::string> >();
      test_run = true;
    }
    if (tests.empty() && vm.count("T"))
    {
      tests = vm["T"].as<std::vector<std::string> >();
      test_run = true;
    }
    else if (!tests.empty() && vm.count("T"))
      test_error("-T and -test used together, but must be mutually exclusive");

    if (vm.count("version"))
    {
      std::vector<std::string> versions =
        vm["version"].as<std::vector<std::string> >();
      disable_parallel_report();
      disable_sequential_report();
      bool stapl_set(false), stl_set(false);
      for (std::vector<std::string>::iterator v  = versions.begin();
                                             v != versions.end();
                                           ++v)
      {
        if (*v == "stapl")
        {
          stapl_set = true;
          enable_parallel_report();
        }
        else if (*v == "stl")
        {
          stl_set = true;
          enable_sequential_report();
        }
        else
          test_error("Unrecognized version passed using -v");
      }
      if (!stapl_set || !stl_set)
      {
        test_disable_checking();
        if (!stl_set)
        {
          disable_sequential_execution();
          disable_sequential_generation();
        }
      }
    }

    if (vm.count("noref"))
    {
      disable_sequential_execution();
      disable_sequential_generation();
      test_disable_checking();
    }

    if (vm.count("pg"))
      disable_sequential_generation();

    if (vm.count("po"))
    {
      disable_sequential_execution();
      disable_sequential_generation();
      test_disable_checking();
    }

    if (vm.count("I"))
      I = vm["I"].as<unsigned int>();
    if (I == 0)
      test_error("-I: iteration count cannot be 0");

    if (vm.count("cpart"))
    {
      std::vector<std::string> part_spec
        = vm["cpart"].as<std::vector<std::string> >();
      if (part_spec[0] == "balanced")
        cct[0] = std::bind(allocate_balanced_container, _1);
      else if (part_spec[0] == "block")
      {
        cct[0] = std::bind(allocate_blocked_container, _1,
                      std::atoi(part_spec[1].c_str()));
      }
      else if (part_spec[0] == "block_cyclic")
      {
        cct[0] = std::bind(allocate_block_cyclic_container, _1,
                      std::atoi(part_spec[1].c_str()));
      }
      cct[1] = cct[2] = cct[0];
    }
    else
    {
      if (vm.count("cpart1"))
      {
        std::vector<std::string> part_spec
          = vm["cpart1"].as<std::vector<std::string> >();
        if (part_spec[0] == "balanced")
          cct[1] = std::bind(allocate_balanced_container, _1);
        else if (part_spec[0] == "block")
        {
          cct[1] = std::bind(allocate_blocked_container, _1,
                        std::atoi(part_spec[1].c_str()));
        }
        else if (part_spec[0] == "block_cyclic")
        {
          cct[1] = std::bind(allocate_block_cyclic_container, _1,
                        std::atoi(part_spec[1].c_str()));
        }
      }
      if (vm.count("cpart2"))
      {
        std::vector<std::string> part_spec
          = vm["cpart2"].as<std::vector<std::string> >();
        if (part_spec[0] == "balanced")
          cct[2] = std::bind(allocate_balanced_container, _1);
        else if (part_spec[0] == "block")
        {
          cct[2] = std::bind(allocate_blocked_container, _1,
                        std::atoi(part_spec[1].c_str()));
        }
        else if (part_spec[0] == "block_cyclic")
        {
          cct[2] = std::bind(allocate_block_cyclic_container, _1,
                        std::atoi(part_spec[1].c_str()));
        }
      }
    }
    traits_type t(cct, cvt);

#if defined(TEST_P_MATRIX)
    //pmatrix requirements for correctness
    int n = int(std::sqrt(float(N)));
    N = n * n;
#endif

    if (!test_run) { // run all tests since no specific was defined
        for (InputIterator it = first; it!=last; ++it) {
          if (data_sizes.empty())
            it->test(N, I, t, it->name, "none");
          else
          {
            for (std::vector<std::string>::iterator ds  = data_sizes.begin();
                                                    ds != data_sizes.end();
                                                  ++ds)
            { it->test(N, I, t, it->name, *ds); }
          }
        }
    } else {
      for (std::vector<std::string>::iterator test  = tests.begin();
                                              test != tests.end();
                                            ++test)
      {
        for (InputIterator it = first; it!=last; ++it)
        {
          if (strcmp(it->name,(*test).c_str()) == 0)
          {
            if (data_sizes.empty())
              it->test(N, I, t, it->name, "none");
            else
            {
              for (std::vector<std::string>::iterator ds  = data_sizes.begin();
                                                      ds != data_sizes.end();
                                                    ++ds)
              { it->test(N, I, t, it->name, *ds); }
            }
            break;
          }
        }
      }
    }
  }
  catch(boost::program_options::unknown_option const& opt) {
    test_error(argv[0], opt.what());
  }
  catch(...) {
    test_error(argv[0], ": exception thrown");
  }
}

#endif
