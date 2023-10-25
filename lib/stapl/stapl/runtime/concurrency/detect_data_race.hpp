/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_DETECT_DATA_RACE_HPP
#define STAPL_RUNTIME_CONCURRENCY_DETECT_DATA_RACE_HPP

#include "../exception.hpp"
#include <atomic>
#include <boost/current_function.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class for detecting possible data races in a class static or
///        stand-alone function.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
struct data_race_helper
{
  const char*        m_file;
  const unsigned int m_line;
  const char*        m_function;
  std::atomic<int>&  m_i;

  data_race_helper(const char* file,
                   unsigned int line,
                   const char* function,
                   std::atomic<int>& i)
  : m_file(file),
    m_line(line),
    m_function(function),
    m_i(i)
  {
    if (++m_i!=1)
      assert_fail("Data race detected at entry", m_file, m_line, m_function);
  }

  ~data_race_helper(void)
  {
    if (--m_i!=0)
      assert_fail("Data race detected at exit", m_file, m_line, m_function);
  }
};

} // namespace runtime

} // namespace stapl


//////////////////////////////////////////////////////////////////////
/// @brief Detects data races in a class static or stand-alone function.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_DETECT_DATA_RACE()                   \
  static std::atomic<int> drd ## __LINE__(0);              \
  stapl::runtime::data_race_helper drd_helper ## __LINE__( \
    __FILE__, __LINE__, BOOST_CURRENT_FUNCTION, drd ## __LINE__);

#endif
