#ifndef STATE_EDGE_H_
#define STATE_EDGE_H_

#include "MPProblem/Weight.h"

template<class StateType>
class StateEdge : public DefaultWeight<StateType> {
  public:
    StateEdge(string _lpLabel="", double _w=1,
        const vector<StateType>& _intermediates = vector<StateType>());
    StateEdge(const DefaultWeight<StateType>& _other);

    const vector<double>& GetControl() const {return m_control;}
    void SetControl(const vector<double>& _v) {m_control = _v;}

    double GetTimeStep() const {return m_timeStep;}
    void SetTimeStep(double _t) {m_timeStep = _t;}

    virtual void Read(istream& _os);
    virtual void Write(ostream& _is) const;

  protected:
    vector<double> m_control;
    double m_timeStep;
};

template<class StateType>
StateEdge<StateType>::
StateEdge(string _lpLabel, double _w, const vector<StateType>& _intermediates) :
  DefaultWeight<StateType>(_lpLabel, _w, _intermediates) {
  }

template<class StateType>
StateEdge<StateType>::
StateEdge(const DefaultWeight<StateType>& _other) :
  DefaultWeight<StateType>(_other) {
  }

template<class StateType>
void
StateEdge<StateType>::
Read(istream& _is) {
  _is >> this->m_weight;
  size_t controlSize;
  _is >> controlSize;
  double tmp;
  m_control.clear();
  for(size_t i = 0; i < controlSize; ++i) {
    _is >> tmp;
    m_control.push_back(tmp);
  }
  _is >> m_timeStep;
}

template<class StateType>
void
StateEdge<StateType>::
Write(ostream& _os) const {
  _os << this->m_weight;
  _os << " " << m_control.size() << " ";
  for(auto& i : m_control)
    _os << i << " ";
  _os << m_timeStep;
}

#endif
