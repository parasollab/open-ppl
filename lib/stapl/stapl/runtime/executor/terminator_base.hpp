/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_TERMINATOR_BASE_HPP
#define STAPL_RUNTIME_EXECUTOR_TERMINATOR_BASE_HPP

#include <utility>
#include <boost/function.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base class for termination detection.
///
/// All termination detectors must inherit from this class.
///
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
class terminator_base
{
public:
  using size_type = std::size_t;
private:
  // Using boost::function instead of std::function to avoid mallocs.
  using notifier_type = boost::function<void(void)>;

  notifier_type m_notifier;

public:
  virtual ~terminator_base(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts this terminator if required.
  //////////////////////////////////////////////////////////////////////
  virtual void operator()(void) = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the function to be called when termination is detected.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void set_notifier(Notifier&& notifier)
  { m_notifier = std::forward<Notifier>(notifier); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of times the terminator has iterated.
  //////////////////////////////////////////////////////////////////////
  virtual size_type iterations(void) const noexcept = 0;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Calls the registered notifier when the terminator has finished.
  //////////////////////////////////////////////////////////////////////
  void call_notifier(void) const
  { m_notifier(); }
};

} // namespace stapl

#endif
