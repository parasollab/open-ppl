// $Id$
/////////////////////////////////////////////////////////////////////
//
//  CfgLigand.c
//
//  General Description
//	A derived template class from CfgManager. It provides some 
//	specific implementation directly related to a multiple joints
//	serial robot.
//
//  Created
//	08/31/99	Guang Song
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////


#include "CfgLigand.h"
#include "Environment.h"
#include "BioPotentials.h"



CfgLigand::CfgLigand(int _numofJoints) : Cfg_free_serial(_numofJoints) {}

CfgLigand::~CfgLigand() {}



bool CfgLigand::isCollision(const Cfg &c, Environment *env, CollisionDetection *cd, 
                             SID _cdsetid, CDInfo& _cdInfo) {

	return BioPotentials::isInPotentialRangeOfNodeGeneration(c, env);
}
