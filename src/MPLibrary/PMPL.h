#ifndef PMPL_H_
#define PMPL_H_

////////////////////////////////////////////////////////////////////////////////
/// Define the configuration-space model (traits) based on the compile options.
////////////////////////////////////////////////////////////////////////////////

#ifdef PMPCfg
#include "ConfigurationSpace/Cfg.h"
#include "Traits/CfgTraits.h"
typedef MPTraits<Cfg> PMPLTraits;

#elif (defined(PMPState))
#include "Traits/StateTraits.h"
typedef StateTraits PMPLTraits;

#else
#error "Error, must define a RobotType for PMPL application"
#endif

////////////////////////////////////////////////////////////////////////////////
/// Set the templated types using the chosen traits.
////////////////////////////////////////////////////////////////////////////////

typedef PMPLTraits::CfgType     CfgType;
typedef PMPLTraits::WeightType  WeightType;
typedef PMPLTraits::RoadmapType RoadmapType;
typedef PMPLTraits::Path        Path;
typedef PMPLTraits::MPLibrary   MPLibrary;
typedef PMPLTraits::MPSolution  MPSolution;

#endif
