#ifndef _BASIC_MAPRM_H_
#define _BASIC_MAPRM_H_

#include "GenerateMapNodes.h"
class CBasicMAPRM {

public:
	static void BasicMAPRM
	(Environment *_env, CollisionDetection* cd, DistanceMetric *dm,GN& _gn, GNInfo &_info);

	/*
	 * nnode is number of nodes that will be generated by MAPRM
	 */
	static bool ReadParameters(int & start, int end, char ** argv, num_param<int>& nnode);

private:
	static void MoveOutObstacle(Cfg & cfg, Environment *_env, CollisionDetection* cd, GNInfo &_info);
	static void MoveOutObstacle(Cfg & cfg, Vector3D & dir,Environment *_env, CollisionDetection* cd, GNInfo &_info);

	static void MoveToMedialAxis
	(Cfg &cfg, vector<Cfg> *path, Environment *_env, CollisionDetection* cd, DistanceMetric *dm, 
	 GN& _gn, GNInfo &_info, int l=0);

	static bool	getCollisionInfo
	( Cfg & cfg, Environment * _env, CollisionDetection* cd, SID cdsetid, CDInfo& cdInfo);

	static void BuildVCLIP( Environment * env );
	
	static num_param<bool> m_bApprox; //using approximation or exact computation
	static num_param<int> m_bRays; //number of shooting rays for approximation penetration.
};

#endif
