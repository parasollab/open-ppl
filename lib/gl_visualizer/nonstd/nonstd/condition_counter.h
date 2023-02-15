#ifndef NONSTD_CONDITION_COUNTER_H
#define NONSTD_CONDITION_COUNTER_H

#include <cstddef>

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// Counts the number of true evaluations, and returns true after some
  /// threshold count is reached. Both consecutive and non-consecutive counts
  /// are supported.
  //////////////////////////////////////////////////////////////////////////////
  class condition_counter
  {

    ///@name Internal State
    ///@{

    size_t       m_counter{0};  ///< Current count.
    const size_t m_threshold;   ///< The minimum count needed to return true.
    const bool   m_consecutive; ///< Require consecutive true evaluations.

    ///@}

    public:

      ///@name Construction
      ///@{

      /// @param[in] _threshold   The counter threshold.
      /// @param[in] _consecutive Require consecutive true evaluations?
      condition_counter(size_t _threshold, bool _consecutive = true);

      ///@}
      ///@name Interface
      ///@}

      /// Check a condition, and increment the counter if it's true. For
      /// consecutive counters, the counter is reset otherwise.
      /// @param[in] _condition The condition to check.
      /// @return A bool indicating whether the threshold has been reached.
      bool operator()(const bool _condition);

      void reset(); ///< Reset the counter to its original state.

      ///@}
  };

}

#endif
