/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime.hpp>
#include <stapl/utility/type_printer.hpp>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include "../test_report.hpp"


template<typename T>
std::string get_type_name(void)
{
  std::ostringstream os;
  os << stapl::type_printer<T>();
  return os.str();
}


template<typename... T>
std::string get_type_names(void)
{
  std::ostringstream os;
  os << stapl::typelist_printer<T...>();
  return os.str();
}


template<typename... T>
std::string get_type_names(T... t)
{
  std::ostringstream os;
  os << stapl::object_type_printer(t...);
  return os.str();
}


void test_type_printer(void)
{
  // primitive types
  STAPL_TEST_REPORT(
    (std::string("int")==get_type_name<int>()),
     "type_printer<int>");
  STAPL_TEST_REPORT(
    (std::string("double")==get_type_name<double>()),
     "type_printer<double>");
  STAPL_TEST_REPORT(
    (std::string("long")==get_type_name<long>()),
     "type_printer<long>");

  // pair
  STAPL_TEST_REPORT(
    (std::string("std::pair<int, int>")==get_type_name<std::pair<int,int>>()),
    "type_printer<std::pair<int,int>>");

  // tuple
  STAPL_TEST_REPORT(
    (std::string("std::tuple<int, int>")==get_type_name<std::tuple<int,int>>()),
    "type_printer<std::tuple<int,int>>");
  auto t = std::forward_as_tuple(42, 42);
  const std::string s = get_type_name<decltype(t)>();
  STAPL_TEST_REPORT(
    (std::string("std::tuple<int&&, int&&>")==s),
    "type_printer<std::forward_as_tuple(42, 42)>");

  // references
  STAPL_TEST_REPORT(
    (std::string("double const&")==get_type_name<double const&>()),
     "type_printer<double const&>");
  STAPL_TEST_REPORT(
    (std::string("double&")==get_type_name<double&>()),
     "type_printer<double&>");
  STAPL_TEST_REPORT(
    (std::string("double&&")==get_type_name<double&&>()),
     "type_printer<double&&>");
}


void test_typelist_printer(void)
{
  STAPL_TEST_REPORT(
    (std::string("int, float const&, double&, char&&") ==
     get_type_names<int, float const&, double&, char&&>()),
     "typelist_printer<int, float const&, double&, char&&>");

  STAPL_TEST_REPORT(
    (std::string("") == get_type_names<>()),
     "typelist_printer<>");
}


void test_object_type_printer(void)
{
  int i { };
  double j { };
  std::tuple<double, bool> k { };

  STAPL_TEST_REPORT(
    (std::string("int, double, std::tuple<double, bool>") ==
     get_type_names(i, j, k)),
     "object_type_printer<int, double, std::tuple<double, bool>>");
}


stapl::exit_code stapl_main(int, char*[])
{
  test_type_printer();
  test_typelist_printer();
  test_object_type_printer();
  return EXIT_SUCCESS;
}
