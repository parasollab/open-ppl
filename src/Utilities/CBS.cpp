#include "CBS.h"

CBSEarlyTerminationFunction
CBSDefaultEarlyTermination() {
  return [](const size_t& _numNodes) {
    return false;
  };
}
