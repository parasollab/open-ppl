#ifndef BATTERY_H_
#define BATTERY_H__

#include <iostream>


////////////////////////////////////////////////////////////////////////////////
/// Models a simulated battery with a finite amount of charge.
///
/// @TODO Unify with other hardware code. We want a general 'battery' base class
///       which could represent either a real or simulated battery. This should
///       be derived to describe either option.
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
