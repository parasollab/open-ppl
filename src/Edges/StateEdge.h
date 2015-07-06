#ifndef STATE_EDGE_H_
#define STATE_EDGE_H_

#include "Cfg/State.h"
#include "MPProblem/Weight.h"

class StateEdge : public DefaultWeight<State> {
  public:
    StateEdge(string _lpLabel="", double _w=1, const vector<State>& _intermediates = vector<State>());
    StateEdge(const DefaultWeight<State>& _other);

    const vector<double>& GetControl() const {return m_control;}
    void SetControl(const vector<double>& _v) {m_control = _v;}

  private:
    vector<double> m_control;
};

#endif
