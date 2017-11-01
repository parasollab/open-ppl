#ifndef BATTERY_H_
#define BATTERY_H__

#include <iostream>


////////////////////////////////////////////////////////////////////////////////
/// Models a battery with a finite amount of charge.
////////////////////////////////////////////////////////////////////////////////
class Battery {

  public:

    ///@name Construction
    ///@{

    Battery();

    ///@}
    ///@name Battery Interface
    ///@{
    /// @TODO Document these functions.

    void SetValues(double, double);
    void SetToMax();
    void Print();
    double GetCurLevel();
    double GetMaxLevel();
    void Charge(double _increaseRate);
    void UpdateValue(double _depletionRateBase, double _depletionRateMoving);
    void UpdateValue(double _depletionRateBase);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_maxLevel;  ///< Maximum charge the battery can hold.
    double m_curLevel;  ///< Current charge in the battery.

    ///@}

};

#endif
