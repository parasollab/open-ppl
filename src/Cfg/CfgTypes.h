#ifndef CfgTypes_h
#define CfgTypes_h

#ifdef PMPRigid
#include "Cfg_free.h"
typedef Cfg_free CfgType;
#endif

#ifdef PMPSerial
#include "Cfg_free_tree.h"
typedef Cfg_free_tree CfgType;
#endif

#ifdef PMPFixedTree
#include "Cfg_fixed_tree.h"
typedef Cfg_fixed_tree CfgType;
#endif

#ifdef PMPCfg2D
#include "Cfg_2D.h"
typedef Cfg_2D CfgType;
#endif

#ifdef PMPCfg2DWithRot
#include "Cfg_2D_withRot.h"
typedef Cfg_2D_withRot CfgType;
#endif

#ifdef PMPReachDistCC
#include "Cfg_reach_cc.h"
typedef Cfg_reach_cc CfgType;
#endif

#ifdef PMPReachDistCCFixed
#include "Cfg_reach_cc_fixed.h"
typedef Cfg_reach_cc_fixed CfgType;
#endif

#ifdef PMPSerial2DOF
#include "Cfg_free_tree_2dof.h"
typedef Cfg_free_tree_2dof CfgType;
#endif

#ifdef PMPRigidMulti
#include "Cfg_free_multi.h"
typedef Cfg_free_multi CfgType;
#endif

#ifdef PMPProtein
#include "CfgProtein.h"
#include "BioWeight.h"
typedef CfgProtein CfgType;
#endif

#ifdef PMPProtein
#include "BioWeight.h"
typedef BioWeight WeightType;
#else
#include "Weight.h"
typedef DefaultWeight WeightType;
#endif

#endif
