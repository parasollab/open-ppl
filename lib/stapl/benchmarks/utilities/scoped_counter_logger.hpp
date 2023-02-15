/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_SCOPED_COUNTER_LOGGER_HPP
#define STAPL_BENCHMARKS_GAP_SCOPED_COUNTER_LOGGER_HPP

#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief A scoped object that uses RAII to start a counter when being
///        initialized and print the value of the counter when the scope
///        ends.
///
///        This could use std::make_scope_exit if it makes it to C++17.
///
/// @tparam T The type of the counter
//////////////////////////////////////////////////////////////////////
template<typename T>
struct scoped_counter_logger
{
private:
  using counter_type = stapl::counter<T>;

  std::string m_message;
  counter_type m_counter;

public:
  scoped_counter_logger(std::string message)
    : m_message(std::move(message)), m_counter{}
  {
    m_counter.start();
  }

  ~scoped_counter_logger(void)
  {
    const auto count = m_counter.stop();
    stapl::do_once([&]() {
      std::cout << m_message << '\t' << count << std::endl;
    });
  }
};

#endif
