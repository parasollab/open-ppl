/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/



#ifndef STAPL_RUNTIME_TEST_TEST_UTILS_H
#define STAPL_RUNTIME_TEST_TEST_UTILS_H

#include <stapl/runtime.hpp>
#include "serialization/test_classes.h"
#include <algorithm>
#include <chrono>
#include <map>
#include <thread>
#include <vector>
#include <boost/current_function.hpp>

#define STAPL_RUNTIME_TEST_CHECK(x, y) {                                   \
  if ((x)!=(y))                                                            \
    stapl::runtime::assert_fail("Check (" #x "==" #y ") failed", __FILE__, \
                                __LINE__, BOOST_CURRENT_FUNCTION); }

#define STAPL_RUNTIME_TEST_REQUIRE(x) {                                      \
  if (!(x))                                                                  \
    stapl::runtime::assert_fail("Check (" #x ") failed", __FILE__, __LINE__, \
                                BOOST_CURRENT_FUNCTION); }

#define STAPL_RUNTIME_TEST_RANGE(x, y) {                                     \
  if ((x).size()!=(y).size())                                                \
    stapl::runtime::assert_fail("Range size check (" #x "==" #y ") failed",  \
                                __FILE__, __LINE__, BOOST_CURRENT_FUNCTION); \
  if (!std::equal(std::begin(x), std::end(x), std::begin(y)))                \
    stapl::runtime::assert_fail("Range check (" #x "==" #y ") failed",       \
                                __FILE__, __LINE__, BOOST_CURRENT_FUNCTION); \
}


class p_test_object
: public stapl::p_object
{
private:
  unsigned int       m_lt;
  unsigned int       m_rt;
  std::map<int,int>  m_values;
  std::map<int,bool> m_sync;

public:
  p_test_object(const unsigned int flags = 0)
  : stapl::p_object(flags),
    m_lt((this->get_location_id()==0)
           ? (this->get_num_locations()-1)
           : (this->get_location_id() - 1)),
    m_rt((this->get_location_id()==this->get_num_locations()-1)
           ? 0
           : (this->get_location_id() + 1))
  { this->advance_epoch(); }

  unsigned int get_left_neighbor(void) const
  { return m_lt; }

  unsigned int get_right_neighbor(void) const
  { return m_rt; }

  void reset(void)
  {
    m_values.clear();
    m_sync.clear();
  }

  template<typename T>
  T get_arg(T const& t)
  { return t; }

  template<typename T>
  std::vector<T> get_vector(void) const
  {
    std::vector<T> v;
    v.reserve(this->get_location_id());
    for (unsigned int i=0; i<this->get_location_id(); ++i) {
      v.push_back(T(this->get_location_id()));
    }
    return v;
  }

  void set(const int i, const int v)
  { m_values[i] = v; }

  void test(const int i, const int t)
  {
    std::map<int,int>::iterator it = m_values.find(i);
    STAPL_RUNTIME_TEST_REQUIRE( (it!=m_values.end()) );
    STAPL_RUNTIME_TEST_CHECK( it->second,t );
  }

  void test_n_set(const int i, const int t, const int v)
  {
    std::map<int,int>::iterator it = m_values.find(i);
    STAPL_RUNTIME_TEST_REQUIRE( (it!=m_values.end()) );
    STAPL_RUNTIME_TEST_CHECK( it->second,t );
    it->second = v;
  }

  void add(const int i, const int v)
  { m_values[i] += v; }

  int get(const int i) const
  {
    std::map<int,int>::const_iterator it = m_values.find(i);
    STAPL_RUNTIME_TEST_REQUIRE( (it!=m_values.end()) );
    return it->second;
  }

  bool exists(const int i) const
  { return (m_values.count(i)>0); }

  void set_sync(const int i)
  { m_sync[i] = true; }

  bool get_sync(const int i)
  {
    if (!m_sync[i]) return false;
    m_sync[i] = false;
    return true;
  }
};


inline void delay(unsigned int sec)
{
  stapl::rmi_flush();
  std::this_thread::sleep_for(std::chrono::seconds{sec});
  stapl::rmi_poll();
}


template<typename T>
struct raw_ptr
{
  T* m_p;

  constexpr raw_ptr(T* t) noexcept
  : m_p(t)
  { }

  constexpr operator T*(void) const noexcept
  { return m_p; }

  void define_type(stapl::typer& t)
  { t.member(stapl::bitwise(m_p)); }
};

#endif
