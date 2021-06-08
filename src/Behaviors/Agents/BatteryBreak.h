#ifndef BATTERY_BREAK_H_
#define BATTERY_BREAK_H_

#include <vector>

#include "ConfigurationSpace/Cfg.h"

////////////////////////////////////////////////////////////////////////////////
/// Stores the furthest time and place that an agent can reach on its path
/// before being required to turn back and go to a charger.
////////////////////////////////////////////////////////////////////////////////
class BatteryBreak {

  public:

    ///@name Construction
    ///@{

    BatteryBreak();

    /// Construct a battery break.
    /// @param _cfg place at which the battery break occurs.
    /// @param _time time at which the battery break occurs.
    BatteryBreak(Cfg _cfg, double _time);

    BatteryBreak(const BatteryBreak& _break);

    BatteryBreak& operator=(const BatteryBreak& _break);

    ~BatteryBreak()=default;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the cfg place of the battery break.
    Cfg GetPlace() const noexcept;

    /// Get the time of the battery break.
    double GetTime() const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    Cfg m_place;        ///< Location of the battery break.
    double m_time{0.0}; ///< Time of the battery break.

    ///@}

};

#endif
