/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <cstdlib>
#include <iostream>
#include <iterator>
#include <string>

#include <stapl/utility/do_once.hpp>

//
// Sends the supplied object to std::cerr
//
template<typename T>
class test_report_func1
{
private:
  T const& m_o;

public:
  typedef void result_type;

  test_report_func1(T const& o)
    : m_o(o)
  { }

  void operator()(void) const
  {
    std::cerr << m_o;
  }
};


template<typename T>
void test_report(T const& o)
{
  stapl::do_once(test_report_func1<T>(o));
}


//
// Prints the given view to std::cout
//
template<typename View>
class test_print_view_func
{
private:
  const char*  m_s;
  View const&  m_view;

public:
  typedef void result_type;

  test_print_view_func(const char *s, View const& view)
    : m_s(s), m_view(view)
  { }

  void operator()(void) const
  {
    std::cout << m_s << "\n";

    std::ostream_iterator<int> os(std::cout, " ");

    std::copy(m_view.begin(), m_view.end(), os);

    std::cout << "\n";
  }
};


template<typename View>
void test_print_view(const char *s, View const& v)
{
  stapl::do_once(test_print_view_func<View>(s,v));
}


//
// Prints an error message and exits
//
class test_error_func1
{
private:
  const char* m_s;

public:
  typedef void result_type;

  test_error_func1(const char* s)
    : m_s(s)
  { }

  void operator()(void) const
  {
    std::cerr << m_s << "\n";
  }
};


void test_error(const char *s)
{
  stapl::do_once(test_error_func1(s));

  std::exit(EXIT_FAILURE);
}


//
//  Prints an error message and exits.
//
class test_error_func2
{
private:
  const char* m_s1;
  const char* m_s2;

public:
  typedef void result_type;

  test_error_func2(const char* s1, const char* s2)
    : m_s1(s1), m_s2(s2)
  { }

  void operator()(void) const
  {
    std::cerr << m_s1 << " " << m_s2 << "\n";
  }
};


void test_error(const char *s1, const char *s2)
{
  stapl::do_once(test_error_func2(s1, s2));

  std::exit(EXIT_FAILURE);
}


//
// Prints an error message and exits.
//
class test_error_func3
{
private:
   std::string const& m_s;

public:
  typedef void result_type;

  test_error_func3(std::string const& s)
    : m_s(s)
  { }

  void operator()(void) const
  {
    std::cerr << m_s << "\n";
  }
};


void test_error(std::string const& s)
{
  stapl::do_once(test_error_func3(s));

  std::exit(EXIT_FAILURE);
}

#endif
