#include "Battery.h"

Battery::
Battery() {
  SetValues(30,30);
}

void 
Battery::
SetValues(double _max, double _cur) {
  m_MaxLevel = _max;
  m_CurLevel = _cur;
}

void 
Battery::
SetToMax() {
  m_CurLevel = m_MaxLevel; 
}

void 
Battery::
Print() { 
  cout << " current battery level: " << m_CurLevel << " max level: " << m_MaxLevel << endl; 
}

double
Battery::
GetCurLevel() { 
  return m_CurLevel; 
}

double 
Battery::
GetMaxLevel() { 
  return m_MaxLevel; 
}

void 
Battery::
Charge(double _increaseRate) {
  m_CurLevel += _increaseRate;
  if( m_CurLevel > m_MaxLevel ) 
    m_CurLevel = m_MaxLevel;
}

void 
Battery::
UpdateValue(double _depletionRateBase, double _depletionRateMoving) {
  m_CurLevel -= _depletionRateBase + _depletionRateMoving;
  if( m_CurLevel < 0 ) 
    m_CurLevel = 0;
}

void 
Battery::
UpdateValue(double _depletionRateBase){
  m_CurLevel -= _depletionRateBase;
  if( m_CurLevel < 0 ) 
    m_CurLevel = 0;
}

