#ifndef CONTROL_H_
#define CONTROL_H_

#include "Utilities/IOUtils.h"

class MultiBody;

class Control {
  public:

    Control(MultiBody* _owner);

    const vector<double>& GetControl() const {return m_control;}

    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const Control& _c);

  private:
    MultiBody* m_owner;
    vector<double> m_control;
};

#endif
