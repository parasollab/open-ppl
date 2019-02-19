#ifndef PMPL_CLOCK_CLASS_H_
#define PMPL_CLOCK_CLASS_H_

#include <iostream>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// This class is used to measure the running time between @c StartClock and
/// @c StopClock. Client side could provide clock name.
///
/// @ingroup MetricUtils
////////////////////////////////////////////////////////////////////////////////
class ClockClass {

  public:

    ClockClass();

    /// Set the clock name.
    /// @param _name The name to use.
    void SetName(const std::string& _name);

    /// Reset the clock.
    void ClearClock();

    /// Start the clock and the name is identity of this clock.
    void StartClock();

    /// Stop the clock and calculate the total running time.
    void StopClock();

    /// Call StopClock and PrintClock.
    void StopPrintClock(std::ostream& _os);

    /// Print clock name and time in seconds to an outstream.
    /// @param _os The outstream to print to.
    void PrintClock(std::ostream& _os);

    /// Get the recorded time in seconds.
    double GetSeconds() const;

    /// Get the recorded time in microseconds.
    int GetUSeconds() const;

  private:

    ///@name Internal State
    ///@{

    int m_sTime, m_uTime;
    int m_suTime, m_uuTime;

    std::string m_clockName;  ///< The clock label.

    ///@}

};

#endif
