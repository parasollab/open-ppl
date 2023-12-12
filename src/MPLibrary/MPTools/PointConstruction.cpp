#include "PointConstruction.h"

#include <vector>
#include <cstdlib>


PointConstruction::
PointConstruction() {
  this->SetName("PointConstruction");
}


PointConstruction::
PointConstruction(XMLNode& _node) : MPBaseObject(_node) {
  this->SetName("PointConstruction");
}


void
PointConstruction::
Initialize() {}
