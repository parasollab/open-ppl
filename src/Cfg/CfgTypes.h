#ifndef CfgTypes_h
#define CfgTypes_h

#include "Cfg_free.h"
#include "Cfg_2D.h"
#include "Cfg_free_tree.h"
#include "Cfg_fixed_tree.h"
#include "Cfg_fixed_PRR.h"
#include "Cfg_reach_cc.h"
#include "Cfg_reach_cc_fixed.h"

#include "Weight.h"

#ifdef PMPRigid
typedef Cfg_free CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPSerial
typedef Cfg_free_tree CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPFixedTree
typedef Cfg_fixed_tree CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPCfg2D
typedef Cfg_2D CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPReachDistCC
typedef Cfg_reach_cc CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPReachDistCCFixed
typedef Cfg_reach_cc_fixed CfgType;
typedef DefaultWeight WeightType;
#endif

#endif
