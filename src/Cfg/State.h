#ifndef STATE_H_
#define STATE_H_

#include "Cfg.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Cfgs
/// @brief Default state space definition.
///
/// State defines a configuration, velocity, and accelleration for each of the
/// degrees of freedom of a robot. This abstraction represents State space.
////////////////////////////////////////////////////////////////////////////////
class State : public Cfg {
  public:
    State(size_t _index = 0);
    State(const Cfg& _other);
    virtual ~State() {};
};

#endif
