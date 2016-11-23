#include "NullAgent.h"

/*------------------------------ Construction --------------------------------*/

NullAgent::
NullAgent(Robot* const _r) : Agent(_r) { }


NullAgent::
~NullAgent() = default;

/*---------------------------- Simulation Interface --------------------------*/

void
NullAgent::
Initialize() {
  // Guard against multiple init.
  if(m_initialized)
    return;
  m_initialized = true;

  // Do nothing.
}


void
NullAgent::
Step() {
  // Null agent does nothing on every step.
}


void
NullAgent::
Uninitialize() {
  // Guard against multiple uninit.
  if(!m_initialized)
    return;
  m_initialized = false;

  // Do nothing.
}

/*----------------------------------------------------------------------------*/
