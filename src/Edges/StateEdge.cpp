#include "StateEdge.h"

StateEdge::
StateEdge(string _lpLabel, double _w, const vector<State>& _intermediates) :
  DefaultWeight<State>(_lpLabel, _w, _intermediates) {
  }

StateEdge::
StateEdge(const DefaultWeight<State>& _other) :
  DefaultWeight<State>(_other) {
  }
