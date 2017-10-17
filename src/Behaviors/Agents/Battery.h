#ifndef BATTERY_H_
#define BATTERY_H__

#include <iostream>

using namespace std;

class Battery {
  public:
    Battery(); 

    void SetValues(double, double);
    void SetToMax();
    void Print();
    double GetCurLevel();
    double GetMaxLevel();
    void Charge(double _increaseRate);
    void UpdateValue(double _depletionRateBase, double _depletionRateMoving);
    void UpdateValue(double _depletionRateBase);

  protected:
    double m_MaxLevel;
    double m_CurLevel;

};

class BatteryBattery: public Battery {
  public:
    using Battery::UpdateValue;
    BatteryBattery() : Battery() {}
    void UpdateValue(double _depletionRateBase, double _depletionRateMoving) {
      m_CurLevel -= _depletionRateBase + _depletionRateMoving;
      if( m_CurLevel < 0 ) m_CurLevel = 0;
    }
};

class WaterBattery: public Battery {
  public:
    using Battery::UpdateValue;
    void UpdateValue(double _depletionRateBase) {
      m_CurLevel -= _depletionRateBase;
      if( m_CurLevel < 0 ) m_CurLevel = 0;
    }
};

#endif
