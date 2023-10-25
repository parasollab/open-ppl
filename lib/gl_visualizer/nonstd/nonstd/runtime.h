#ifndef NONSTD_RUNTIME_H_
#define NONSTD_RUNTIME_H_

#include <string>

namespace nonstd {

  ///@name Runtime Helpers
  ///@{

  /// Exit with a custom message.
  /// @param[in] _message The message to display.
  void exit_msg(const std::string& _message) noexcept;


  /// Assert a condition, and exit with a custom message on failure.
  /// @param[in] _condition The condition to assert.
  /// @param[in] _message   The message to display on failure.
  void assert_msg(const bool _condition, const std::string& _message) noexcept;

  ///@}

}

#endif
