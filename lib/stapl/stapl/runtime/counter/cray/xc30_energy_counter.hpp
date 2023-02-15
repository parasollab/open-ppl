/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_CRAY_XC30_ENERGY_COUNTER_HPP
#define STAPL_RUNTIME_COUNTER_CRAY_XC30_ENERGY_COUNTER_HPP

#include "../../config/platform.hpp"
#include <cstddef>
#include <fstream>

#ifndef STAPL_RUNTIME_CRAY_XC30_TARGET
# error "xc30_energy_counter is only valid for Cray XC30 machines."
#endif

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Counter that measures energy consumption in Joules for the Cray XC30
///        platform.
///
/// The counter reads the <tt>/sys/cray/pm_counters/energy</tt> file by default
/// for energy consumption information per node.
///
/// @ingroup counters
/////////////////////////////////////////////////////////////////////
class xc30_energy_counter
{
public:
  typedef std::size_t raw_value_type;
  typedef std::size_t value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "cray_xc_30_energy_counter()"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value to Joules.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type normalize(const raw_value_type v) noexcept
  { return value_type(v); }

private:
  std::ifstream  m_is;
  raw_value_type m_v;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref xc30_energy_counter that reads from
  ///        @p filename.
  //////////////////////////////////////////////////////////////////////
  explicit
  xc30_energy_counter(const char* filename = "/sys/cray/pm_counters/energy")
  : m_is(filename, std::ios::in),
    m_v(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  raw_value_type read(void)
  {
    m_is.seekg(0);
    raw_value_type joules = 0;
    m_is >> joules;
    return joules;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts the counter.
  //////////////////////////////////////////////////////////////////////
  void start(void)
  { m_v = read(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the counter and returns the difference from @ref start() in
  ///        Joules.
  //////////////////////////////////////////////////////////////////////
  value_type stop(void)
  { return normalize(read() - m_v); }
};

} // namespace stapl

#endif
