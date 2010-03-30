#ifndef CfgTypes_h
#define CfgTypes_h

#include "Weight.h"

#ifdef PMPRigid
#include "Cfg_free.h"
typedef Cfg_free CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPSerial
#include "Cfg_free_tree.h"
typedef Cfg_free_tree CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPFixedTree
#include "Cfg_fixed_tree.h"
typedef Cfg_fixed_tree CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPCfg2D
#include "Cfg_2D.h"
typedef Cfg_2D CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPReachDistCC
#include "Cfg_reach_cc.h"
typedef Cfg_reach_cc CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPReachDistCCFixed
#include "Cfg_reach_cc_fixed.h"
typedef Cfg_reach_cc_fixed CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPSerial2DOF
#include "Cfg_free_tree_2dof.h"
typedef Cfg_free_tree_2dof CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPRigidMulti
#include "Cfg_free_multi.h"
typedef Cfg_free_multi CfgType;
typedef DefaultWeight WeightType;
#endif

#ifdef PMPProtein
#include "CfgProtein.h"
#include "BioWeight.h"
typedef CfgProtein CfgType;
typedef BioWeight WeightType;
#endif

#endif
