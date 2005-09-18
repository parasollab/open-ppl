#ifndef CfgTypes_h
#define CfgTypes_h

#include "Cfg_free.h"
#include "Cfg_2D.h"
#include "Cfg_free_tree.h"
#include "Cfg_fixed_tree.h"
#include "Cfg_fixed_PRR.h"


#include "Weight.h"

#ifdef PMPRigid
typedef Cfg_free CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPSerial
typedef Cfg_free_tree CFG;
typedef DefaultWeight WEIGHT;
#endif

#endif
