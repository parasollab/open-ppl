#include "BasicMAPRM.h"
#include "MultiBody.h"
#include "util.h"

#define EXPANSION_FACTOR 100

num_param<bool> CBasicMAPRM::m_bApprox("approx",true,false,true);
num_param<int> CBasicMAPRM::m_bRays("appox_ray",10,1,100);
bool CBasicMAPRM::ReadParameters(int & start, int end, char ** argv, num_param<int>& nnode)
{
	m_bApprox.PutDesc("BOOL","");
	m_bRays.PutDesc("INTEGER","");
	if( start>end || argv==NULL ) return false;
	for(;start<end;start++ ){
		if( m_bApprox.AckCmdLine(&start,(int)end,argv) ){}
		else if( m_bRays.AckCmdLine(&start,(int)end,argv) ){}
		else if( nnode.AckCmdLine(&start,(int)end,argv) ){}
		else return false;
	}

#ifndef USE_VCLIP
	if( m_bApprox.GetValue()==false ){ //use exact penetration
		cerr<<" Error : Compute exact penetration for BasicMAPRM requres VCLIP."
		    <<" Please Recompile OBPRM with USE_VCLIP flag"<<endl;
		return false;
	}
#endif
#if !defined(USE_VCLIP) && !defined(USE_PQP)
	cerr<<" Error : BasicMAPRM requres VCLIP or PQP."
	    <<" Please Recompile OBPRM with USE_PQP or/and USE_VCLIP flag"<<endl;
	return false;
#endif
	
	return true;
}


//===================================================================
// Basic MAPRM
// written by Brent, June 2000  -- FUNCTION IS IN DEVELOPMENT
//
// Do NOT count on this function staying the way it is
//
// Medial Axis version - take 1
// Points NOT in collision and push them to medial axis
// Points in collision are moved out of via random dir, then
//    pushed towards MA.
//
// Originally just works with VCLIP and Cfg_free robots.
// cfg_free robots are in (x, y, z, roll, pitch, yaw) format
//===================================================================
void CBasicMAPRM::
BasicMAPRM(Environment *_env, CollisionDetection* cd, 
           DistanceMetric *dm,GN& _gn, GNInfo &_info)
{
	bool     collided;
	Cfg      cfg;
	
#ifndef QUIET
	cout << endl << "Begin BasicMAPRM..." << endl;
	cout << "(numNodes=" << _gn.numNodes.GetValue() << ") " << endl;
	cout << "_env PositionRes  = " << _env->GetPositionRes() << endl;
	cout << "Expansion Factor  = " << EXPANSION_FACTOR << endl;
#endif
	
#if INTERMEDIATE_FILES
	vector<Cfg> path; 
	path.reserve(_gn.numNodes.GetValue());
#endif
	
#ifdef USE_VCLIP
	if( m_bApprox.GetValue()==false ) //use exact penetration (VCLIP require). 
		BuildVCLIP(_env); //Make sure vclip is buid
#endif
	
	cout<<"- "<<flush;
	for (int i=0; i < _gn.numNodes.GetValue(); i++)
	{
		if( i%80==0 && i!=0 ) cout<<"("<<i<<"/"<<_gn.numNodes.GetValue()<<")"<<endl<<"- ";
		cout<<"#"<<flush;

		// Get a random configuration that STARTS in the bounding box of env
		cfg = Cfg::GetRandomCfg(_env);  // should always be in bounding box

		//use approximate computation for moving out robot from obs
		if( m_bApprox.GetValue() ){
			_info.cdInfo.ret_all_info = false;
			collided = cfg.isCollision(_env, cd, _info.cdsetid, _info.cdInfo);
			if( collided )	MoveOutObstacle(cfg,_env,cd,_info);
			if(cd->isInsideObstacle(cfg,_env,_info.cdsetid,_info.cdInfo)){
				collided=true;
				_info.cdInfo.ret_all_info = true;
				cfg.isCollision(_env, cd, _info.cdsetid, _info.cdInfo);
				Vector3D trans_dir=(_info.cdInfo.object_point-_info.cdInfo.robot_point)*1.00001;
				_info.cdInfo.ret_all_info = false;
				MoveOutObstacle(cfg,trans_dir,_env,cd,_info);
			}
			if( collided ) collided = cfg.isCollision(_env, cd, _info.cdsetid, _info.cdInfo);
		}
#ifdef USE_VCLIP
		else{ //use exact computation for penetration

			_info.cdInfo.ret_all_info = true;	
			cfg.isCollision(_env, cd, VCLIP, _info.cdInfo); //use vclip
			_info.cdInfo.ret_all_info = false;
			if( collided ){
				Vector3D dir=(_info.cdInfo.object_point-_info.cdInfo.robot_point)*1.00001;
				cfg.SetSingleParam(0, cfg.GetData()[0]+dir[0]);
				cfg.SetSingleParam(1, cfg.GetData()[1]+dir[1]);
				cfg.SetSingleParam(2, cfg.GetData()[2]+dir[2]);
				collided = cfg.isCollision(_env, cd, _info.cdsetid, _info.cdInfo);
			}
		}
#endif

		// So we "should" be out of collision and INSIDE bounding box 
		// so we can move cfg toward Medial Axis
		if (!collided)
		{
#if INTERMEDIATE_FILES
      		      	MoveToMedialAxis(cfg, &path, _env, cd, dm, _gn, _info);
			//_info.nodes.push_back(cfg);
#else
            		MoveToMedialAxis(cfg, NULL, _env, cd, dm, _gn, _info);
			//_info.nodes.push_back(cfg);
#endif
		}
		else
		{
			//cout << "BasicMAPRM unable to move random cfg out of collision." << endl;
			cout<<"?"<<flush;
			i--;
		}
		
	} // end for i = 1 to _gn.numNodes.GetValue()
	
#if INTERMEDIATE_FILES
	WritePathConfigurations("maprm.path", path, _env);
#endif
	
#ifndef QUIET
	cout << endl << "... END BasicMAPRM" << endl;
#endif
	
} // end BasicMAPRM


void CBasicMAPRM::MoveOutObstacle
(Cfg & cfg, Environment *_env, CollisionDetection* cd, GNInfo &_info)
{
	// Store cfg in its "original" state
	Cfg orig_cfg = cfg;
	Cfg non_collide_cfg = cfg;

	//bounding box
	static double * bb = _env->GetBoundingBox();
	static double x_scale = bb[1] - bb[0];
	static double y_scale = bb[3] - bb[2];
	static double z_scale = bb[5] - bb[4];

	// since what points are "near" is not clearly defined when collisions
	// occur - particularly across collision detection libraries
	// we will pick a completely random direction and push our point
	// that way, once we get out of collision we will apply the
	// same method as if we were originally not in collision
	
	// Set _info so we do NOT get all info (for speed)
	_info.cdInfo.ResetVars();
	
	// Generate Random direction
	Cfg move_out_dir_cfg = Cfg::GetRandomRay( _env->GetPositionRes() );
	
	long attempts_out = 0;
	long steps_out = 0;
	long min_steps_out = 9999999;   // init to arbitrary LARGE number
	long max_tries_out = m_bRays.GetValue();  // how many times do we try to get random cfg out of collision
	
	// We will always try max_tries_out times and we keep the one
	// that moved out in the fewest number of steps
	while(attempts_out < max_tries_out)
	{
		// Move cfg out of collision in move_out_dir_cfg
		// later use trans_cfg instead of incrCfg... should get odd? results ???
		non_collide_cfg = non_collide_cfg + move_out_dir_cfg;
		steps_out++;

		if( steps_out<=min_steps_out ){
			if( non_collide_cfg.isCollision(_env, cd, _info.cdsetid, _info.cdInfo)==true ) 
				continue;
		}
		
		//make sure this node is inside bbx
		if (non_collide_cfg.InBoundingBox(_env)){ //in bbx
			if (steps_out < min_steps_out){
				cfg = non_collide_cfg;
				min_steps_out = steps_out;
			}
		}

		steps_out=0;
		non_collide_cfg=orig_cfg;
		attempts_out++;
	} // end while not "too many" attempts
}

//robot is completely inside obs
void CBasicMAPRM::MoveOutObstacle
(Cfg & cfg, Vector3D & dir,Environment *_env, CollisionDetection* cd, GNInfo &_info)
{
	// Store cfg in its "original" state
	Cfg orig_cfg = cfg;
	cfg.SetSingleParam(0, cfg.GetData()[0]+dir[0]);
	cfg.SetSingleParam(1, cfg.GetData()[1]+dir[1]);
	cfg.SetSingleParam(2, cfg.GetData()[2]+dir[2]);
	
	//Make smaller increament
	Cfg trans_cfg; //increament
	dir.normalize();
	trans_cfg.SetSingleParam(0, dir[0]);
	trans_cfg.SetSingleParam(1, dir[1]);
	trans_cfg.SetSingleParam(2, dir[2]);
	trans_cfg=trans_cfg*0.1;

	//move in same dir until out of collision
	do{
		cfg=cfg+trans_cfg;
		if (!cfg.InBoundingBox(_env)){ cfg=orig_cfg; break; } //can't find a way to push out
	}
	while(cfg.isCollision(_env, cd, _info.cdsetid, _info.cdInfo));
}

//===================================================================
// MoveToMedialAxis
// written by Brent, September 2000  -- FUNCTION IS IN DEVELOPMENT
//
// Move a configuration - that is NOT in collision - to the MA
// (or at least close to the Medial Axis)
//===================================================================
void CBasicMAPRM::
MoveToMedialAxis(Cfg &cfg, vector<Cfg> *path, Environment *_env, 
                 CollisionDetection* cd, DistanceMetric *dm,GN& _gn, GNInfo &_info, int l)
{
	//if(l>3){ _info.nodes.push_back(cfg); return;}

	Vector3D trans_dir;
	Cfg      trans_cfg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	CDInfo   oldInfo;
	
	// Set flag in _info.cdInfo to cause isCollision 
	// to return "all info" such as witness pairs and min dist 
	// and collision object index
	// NEW info currently always put in _info.cdInfo
	// note we do NOTHING with cd->stuff
	_info.cdInfo.ResetVars();
	_info.cdInfo.ret_all_info = true;
	
	// find closest obstacle -- and collided better come back false!
	getCollisionInfo(cfg,_env,cd,_info.cdsetid,_info.cdInfo);
	trans_dir=_info.cdInfo.robot_point-_info.cdInfo.object_point;	
	trans_dir.normalize();
	
	// And then scale arbitrarily 'small' - prefer to have scale determined by the _env
	trans_dir = trans_dir * 0.2;
	
	// Create a translation variable cfg
	// Make some BIG assumptions about parameter order - for cfg_free this should be right
	trans_cfg.SetSingleParam(0, trans_dir[0]);
	trans_cfg.SetSingleParam(1, trans_dir[1]);
	trans_cfg.SetSingleParam(2, trans_dir[2]);
	
	//Find the cfg that changes cloeset obs id
	//(This is not good (for non-convex), we should check closest pt.)
	Cfg oldcfg=cfg;	Cfg newcfg=cfg;
	Vector3D diff;

	do  //want to translate cfg in trans_dir until closest obstacle changes
	{
		oldcfg = newcfg;
		newcfg = oldcfg + trans_cfg;
		oldInfo = _info.cdInfo;   // oldInfo used in loop termination check
		_info.cdInfo.ResetVars();
		_info.cdInfo.ret_all_info = true;
		getCollisionInfo(newcfg,_env,cd,_info.cdsetid,_info.cdInfo);
		diff=_info.cdInfo.object_point-oldInfo.object_point;
	} while ( diff.normsqr()<1e-2 );
	
	if( cd->isInsideObstacle(newcfg,_env,_info.cdsetid,_info.cdInfo) ) return;
	//make sure newcfg is collision free
	_info.cdInfo.ResetVars();
	while( true ){
		if( newcfg.isCollision(_env, cd, _info.cdsetid, _info.cdInfo)==false ) break;
		if( (oldcfg-newcfg).PositionMagnitude() < _env->GetPositionRes() ) return;
		newcfg=(oldcfg+newcfg)/2;
	}

	cfg=newcfg;
// while we should never move into a collision state, we check anyway	
#if INTERMEDIATE_FILES
	//MoveToMedialAxis(cfg, path, _env, cd, dm, _gn, _info,++l);
	path->push_back(cfg);
#endif

	_info.nodes.push_back(cfg);

} // end MoveToMedialAxis


//Gather collision info including bbx and other obscales
bool CBasicMAPRM::
getCollisionInfo( Cfg & cfg, Environment * _env, CollisionDetection* cd, SID cdsetid, CDInfo& cdInfo)
{
	//cout<<cfg<<endl;
	//if( !cfg.InBoundingBox(_env) ) 
	//cout<<"Ho...Shit"<<endl;
	
	if( cfg.isCollision(_env, cd, cdsetid, cdInfo) ) return true;
	const static double * bbx = _env->GetBoundingBox();
	MultiBody *robot = _env->GetMultiBody(_env->GetRobotIndex());
	
	//find cloest pt between robot and bbx
	for(int m=0; m<robot->GetFreeBodyCount(); m++) {
		GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
		for(int j = 0 ; j < poly.numVertices ; j++){
			//cout<<j<<" "<<poly.vertexList[j]<<endl;
			bool change=false;
			if( (poly.vertexList[j][0]-bbx[0])<cdInfo.min_dist ){
				change=true;
				cdInfo.object_point[0]=bbx[0];
				cdInfo.object_point[1]=poly.vertexList[j][1];
				cdInfo.object_point[2]=poly.vertexList[j][2];
				cdInfo.min_dist=poly.vertexList[j][0]-bbx[0];
				cdInfo.nearest_obst_index=-1; //bbx
			}
			
			if( (bbx[1]-poly.vertexList[j][0])<cdInfo.min_dist ){
				change=true;
				cdInfo.object_point[0]=bbx[1];
				cdInfo.object_point[1]=poly.vertexList[j][1];
				cdInfo.object_point[2]=poly.vertexList[j][2];
				cdInfo.min_dist=bbx[1]-poly.vertexList[j][0];
				cdInfo.nearest_obst_index=-2; //bbx
			}
			
			if( (poly.vertexList[j][1]-bbx[2])<cdInfo.min_dist ){
				change=true;
				cdInfo.object_point[0]=poly.vertexList[j][0]; 
				cdInfo.object_point[1]=bbx[2];
				cdInfo.object_point[2]=poly.vertexList[j][2];
				cdInfo.min_dist=poly.vertexList[j][1]-bbx[2];
				cdInfo.nearest_obst_index=-3; //bbx
			}
			
			if( (bbx[3]-poly.vertexList[j][1])<cdInfo.min_dist ){
				change=true;
				cdInfo.object_point[0]=poly.vertexList[j][0];
				cdInfo.object_point[1]=bbx[3];
				cdInfo.object_point[2]=poly.vertexList[j][2];
				cdInfo.min_dist=bbx[3]-poly.vertexList[j][1];
				cdInfo.nearest_obst_index=-4; //bbx
			}
			
			if( (poly.vertexList[j][2]-bbx[4])<cdInfo.min_dist ){
				change=true;
				cdInfo.object_point[0]=poly.vertexList[j][0]; 
				cdInfo.object_point[1]=poly.vertexList[j][1]; 
				cdInfo.object_point[2]=bbx[4];
				cdInfo.min_dist=poly.vertexList[j][2]-bbx[4];
				cdInfo.nearest_obst_index=-5; //bbx
			}
			
			if( (bbx[5]-poly.vertexList[j][2])<cdInfo.min_dist ){
				change=true;
				cdInfo.object_point[0]=poly.vertexList[j][0];
				cdInfo.object_point[1]=poly.vertexList[j][1];
				cdInfo.object_point[2]=bbx[5];
				cdInfo.min_dist=bbx[5]-poly.vertexList[j][2];
				cdInfo.nearest_obst_index=-6; //bbx
			}
			
			if(change==true){
				cdInfo.robot_point[0]=poly.vertexList[j][0]; 
				cdInfo.robot_point[1]=poly.vertexList[j][1];
				cdInfo.robot_point[2]=poly.vertexList[j][2];
			}
		}
	}
	
	return (cdInfo.min_dist>=0)?false:true;;
}

#ifdef USE_VCLIP
void CBasicMAPRM::BuildVCLIP( Environment * env ){
		int n=env->GetMultiBodyCount();
		for( int iM=0;iM<n;iM++ ){ //for each m-body
			MultiBody * mbody=env->GetMultiBody(iM);
			int nb=mbody->GetBodyCount();
			for( int iB=0;iB<nb;iB++ ){
				Body * body=mbody->GetBody(iB);
				PolyTree * tree=body->GetVclipBody();
				if( tree==NULL )
				       	body->buildCDstructure(VCLIP);
			}//end for each body
		}//end for each m-body
}
#endif
