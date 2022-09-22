#include "SIPPMethod.h"

std::istream& operator>>(std::istream& _is, SIPPVertex& _vertex) {
  return _is;
}

std::ostream& operator<<(std::ostream& _os, const SIPPVertex& _vertex) {
  return _os;
}

std::istream& operator>>(std::istream& _is, SIPPEdge& _edge) {
  return _is;
}

std::ostream& operator<<(std::ostream& _os, const SIPPEdge& _edge) {
  return _os;
}

