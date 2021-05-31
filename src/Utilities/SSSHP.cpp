#include "SSSHP.h"


SSSHPTerminationCriterion
SSSHPDefaultTermination() {
  return [](size_t&, const MBTOutput&) {
    return SSSHPTermination::Continue;
  };
}
