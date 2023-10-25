#ifndef NONSTD_TIMER_H_
#define NONSTD_TIMER_H_

#include <chrono>

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// A stopwatch-like wrapper around chrono's high-resolution clock. The
  /// durations are internally measured in nanoseconds, and reported as seconds
  /// by default.
  ///
  /// @WARNING This object is not re-entrant: it should be called from only one
  ///          thread at a time.
  //////////////////////////////////////////////////////////////////////////////
  class timer
  {

    ///@name Local Types
    ///@{

    typedef std::chrono::high_resolution_clock           clock;
    typedef std::chrono::duration<double, std::nano>     nano_seconds;
    typedef std::chrono::time_point<clock, nano_seconds> time_stamp;

    ///@}
    ///@name Internal State
    ///@{

    time_stamp   m_last;           ///< The last measured time point.
    nano_seconds m_total;          ///< The total measured duration.
    bool         m_running{false}; ///< Is the timer running?

    ///@}

    public:

      ///@name Construction
      ///@{

      timer();
      virtual ~timer() = default;

      ///@}
      ///@name Interface
      ///@{

      void start() noexcept;   ///< Start the timer.
      void stop() noexcept;    ///< Pause the timer and update total duration.
      void reset() noexcept;   ///< Reset to initial state.
      void restart() noexcept; ///< Reset and start the timer.

      double elapsed() const noexcept; ///< Get the total elapsed time in seconds.

      ///@}
  };
}

#endif
