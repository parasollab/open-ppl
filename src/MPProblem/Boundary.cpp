#include "Boundary.h"

ostream&
operator<<(ostream& _os, const Boundary& _b) {
  _b.Write(_os);
  return _os;
}

