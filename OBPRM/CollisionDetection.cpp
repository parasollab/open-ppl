// $Id$
/////////////////////////////////////////////////////////////////////
//
//  CollisionDetection.c
//
//  General Description
//
//  Created
//      8/11/98  Daniel Vallejo
//
/////////////////////////////////////////////////////////////////////

#include "CollisionDetection.h"

#include "Cfg.h"
#include "Input.h"
#include "Environment.h"
#include "MultiBody.h"
#include "GenerateMapNodes.h"
#include "ConnectMapNodes.h"
#include "Stat_Class.h"
#include <string.h>
#include "util.h"

extern Stat_Class Stats;

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CollisionDetection
//
/////////////////////////////////////////////////////////////////////
//==================================
// CollisionDetection class Methods: Constructors and Destructor
//==================================

CollisionDetection::
CollisionDetection() {
    DefaultInit();
};

CollisionDetection::
~CollisionDetection() {
};

//==================================
// CollisionDetection class Methods: Collision Detection Functions
//==================================

void
CollisionDetection::
DefaultInit()
{
	penetration=-1;	
}


void CollisionDetection::
SetPenetration(double times)
{
	penetration=times;
}

void
CollisionDetection::
UserInit(Input * input,  GenerateMapNodes* gn, ConnectMapNodes* cn)
{
	//-----------------------------------------------
	// initialize collision detection
	//  CAUTION:  DO NOT CHANGE ORDER OF SET DEFN's
	//           w/o CHANGING ENUM ORDER in "OBPRM.h"
	//-----------------------------------------------
	
    // initialize cd sets
#ifdef USE_CSTK
    collisionCheckers.MakeCDSet("cstk");	// enum CSTK
	// ie,c-space toolkit
#endif
	
#ifdef USE_VCLIP
    collisionCheckers.MakeCDSet("vclip");    // enum VCLIP
#endif
	// ie,voronoi clip
#ifdef USE_RAPID
    collisionCheckers.MakeCDSet("RAPID");    // enum RAPID
#endif
	
#ifdef USE_PQP
    collisionCheckers.MakeCDSet("PQP");    // enum PQP
	m_pRay=NULL;
#endif
	
    if( input->numCDs == 0 ){                  	// use default CD sets
    }
    else{                                     	// make user-defined sets
        gn->gnInfo.cdsetid=CD_USER1;
        cn->cnInfo.cdsetid=CD_USER1;
		for (int i = 0; i < input->numCDs; i++) {
       	    collisionCheckers.MakeCDSet(input->CDstrings[i]->GetValue());
		}
    }
};


#ifdef USE_VCLIP
VclipPose CollisionDetection::GetVclipPose(const Transformation &myT,
										   const Transformation &obstT) {
	
	Transformation diff = Transformation(obstT).Inverse() * myT;
	
	diff.orientation.ConvertType(Orientation::EulerXYZ);
	
	//------------------------------------------------
	// here's where it really starts.
	//------------------------------------------------
	
	Vect3 XYZ(diff.position.getX(),diff.position.getY(),diff.position.getZ());
	
	Quat RPY         (diff.orientation.alpha,Vect3::I);
	RPY.postmult(Quat(diff.orientation.beta ,Vect3::J));
	RPY.postmult(Quat(diff.orientation.gamma,Vect3::K));
	
	// the above is for EulerXYZ.
	// For EulerZYX, or FixedXYZ, we should have the following instead,
	// i.e. Rotation = Rz(alpha) * Ry(beta) * Rx(gamma)
	//Quat RPY         (diff.orientation.alpha,Vect3::K);
	//RPY.postmult(Quat(diff.orientation.beta ,Vect3::J));
	//RPY.postmult(Quat(diff.orientation.gamma,Vect3::I));
	
	return VclipPose(RPY,XYZ);
}

#endif
//----------------------------------------------------------------------
// SetLineTransformation
// set linTrans[12], which is used by cstk collision checker.
//----------------------------------------------------------------------
void CollisionDetection::
SetLineTransformation(const Transformation& trans, double linTrans[12]) {
    Transformation tmp = trans;
    tmp.orientation.ConvertType(Orientation::Matrix);
    for(int n=0; n<3; n++) {
		for(int j=0; j<3; j++) {
			linTrans[4*n+j]=tmp.orientation.matrix[n][j];
		}
		linTrans[4*n+3]=tmp.position[n];
    }
}



#ifdef DEBUG
vector<Cfg> acceptable;
#endif 
//////////////////////////////////////////
// AcceptablePenetration
//
// If there is a collision, check whether it is in acceptable range
//
//
/////////////////////////////////////////////
bool CollisionDetection::
AcceptablePenetration(Cfg c,Environment *env,CollisionDetection *cd, 
					  SID cdsetid, CDInfo& cdInfo)
{
	int numOkCfgs=0;
	
	for(int i=0;i<directions.size();i++)
	{
		Cfg next=c+directions[i];
		
		if (!next.isCollision(env,cd,cdsetid,cdInfo,false)) {
			numOkCfgs++;
			if ((numOkCfgs*1.0/directions.size())>acceptableRatio){
#ifdef DEBUG
				acceptable.push_back(c);
#endif
				return true;
			}
		}
		
	}
	if((numOkCfgs*1.0/directions.size())>acceptableRatio)  {
		
		return true; }
	else
		return false;	
}

//////////////////////////////////////////////////////////////////////////
// InitializePenetration
//
// Set the penetrationdepth and find n direction vectors
//
//
//////////////////////////////////////////////////////////////////////////

void CollisionDetection::
InitializePenetration(double times,int nCfgs, Environment *env,
					  DistanceMetric * dm, SID dmsetid,double ratio)
{
	
	Cfg origin;
	acceptableRatio=ratio; 
	// first find the environment resolution
	Cfg res=Cfg::GetResolutionCfg(env);
	
	// now find the length of that resolution
	double length=dm->Distance(env,res,origin,dmsetid);
	
	// penetration is length*times
	penetration=times*length;
	
	cout << "Penetration is " << penetration << endl;
	
	for(int i=0;i<nCfgs;i++) {
		directions.push_back(Cfg::GetRandomCfg(env,dm,dmsetid,penetration*drand48()));
		cout <<"Added Cfg\n"<<flush;
	}
}


//////////////////////////////////////////////////////////////////////////
// IsInCollision
//
// modified by Brent, June 2000
// Added option to "get all info"
//
// Behavior is as follows:
// lineRobot parameter defaults to NULL if not sent
//
// if we DON'T want all info we 
//    will exit on the first collision detected
//    _cdInfo.colliding_obst_index is set
//    or loop through all objects and return no collision
//
// else 
//    we will ALWAYS go examine every obstacle
//    we still return if there is a collision
//    all of _cdInfo will be set to correspond to the closest obstacle
//    Notice if the robot self-collides, NOT all info is gotten
//
//////////////////////////////////////////////////////////////////////////
bool
CollisionDetection::
IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo, MultiBody* lineRobot,bool enablePenetration)
{
	int nmulti, robot;
	bool ret_val, collision_found; // needed to go thru ALL obstacles to get ALL info
	CDInfo local_cd_info;
	nmulti = env->GetMultiBodyCount();
	robot = env->GetRobotIndex();
	
	MultiBody * rob = env->GetMultiBody(robot);
	
	// A line Segment generated on the fly, to check if 'seemingly connectable'.
	if (lineRobot) 
	{
		rob = lineRobot;
	}
	
	ret_val = false;
	
	for (int i = 0; i < nmulti; i++)
	{
		if ( i != robot )
		{
			// Note that the below call sets _cdInfo as needed
			collision_found = IsInCollision(env, _cdsetid, _cdInfo, rob, env->GetMultiBody(i));
			
			if ( (collision_found) && ( ! _cdInfo.ret_all_info) )
			{
				_cdInfo.colliding_obst_index = i;
				return true;
			}
			else  if (_cdInfo.ret_all_info)   // store more info
			{
				if ((collision_found) && (!ret_val))
				{
					// colliding_obst_index is always the FIRST obstacle found in collision
					// nearest_obst_index is 'nearest' obstacle (colliding or not)
					local_cd_info.colliding_obst_index = i;
					ret_val = true;
				}
				
				// Be certain that IsInCollision set _cdInfo.min_dist
				// Check new mins against old, reset *_points if needed
				// Store everything in local_cd_info, copy back to _cdInfo at end of function
				if (_cdInfo.min_dist < local_cd_info.min_dist)
				{
					local_cd_info.nearest_obst_index = i;
					local_cd_info.min_dist = _cdInfo.min_dist;
					local_cd_info.robot_point = _cdInfo.robot_point;
					local_cd_info.object_point = _cdInfo.object_point;
				} // end updating local_cd_info
			}
		} 
		else 
		{
			// robot self checking. Warning: rob and env->GetMultiBody(robot) may NOT be the same.
			if ( (rob->GetBodyCount() > 1) && 
				(IsInCollision(env, _cdsetid, _cdInfo, rob, rob)) )
			{
				if (_cdInfo.ret_all_info)
				{
					// set stuff to indicate odd happenning
					_cdInfo.colliding_obst_index = -1;
					_cdInfo.min_dist = MaxDist;
					_cdInfo.nearest_obst_index = -1;
					_cdInfo.robot_point[0] = _cdInfo.robot_point[1] = _cdInfo.robot_point[2] = 0;
					_cdInfo.object_point[0] = _cdInfo.object_point[1] = _cdInfo.object_point[2] = 0;
				}
				
				return true;
			}
		} // end  if-else i == robot
		
	} // end for i
	
	if (_cdInfo.ret_all_info)
	{
		// local_cd_info should contain "all the info" across all objects
		// _cdInfo only contains info for the last one processed above
		_cdInfo = local_cd_info;
	}
	
	return ret_val;
} // end IsInCollision ( 4 params, 4th defaults to NULL)

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

double
CollisionDetection::
Clearance(Environment * env){
#ifdef USE_CSTK
	
    int nmulti, robot;
    nmulti = env->GetMultiBodyCount();
    robot = env->GetRobotIndex();
	
    MultiBody *rob, *obst;
    rob = env->GetMultiBody(robot);
	
    double tmp, dist = MaxDist;
    for(int i = 0 ; i < nmulti ; i++){
        if(i != robot){
			obst = env->GetMultiBody(i);
			tmp = cstkDistance(rob, obst);
			if(tmp < dist){
				dist = tmp;
			}
		}
    }
    return dist;
#else
	cout << "Clearance function is not supported by "
		<< "current collision detection library." << endl
		<< "Please recompile with a supporting library.\n";
    exit(5);
#endif
}


bool CollisionDetection::
isInsideObstacle(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo)
{
	vector<CD> cdset = collisionCheckers.GetCDSet(_cdsetid);
	CDF cdfcn = cdset[0].GetCollisionDetection();
#ifdef USE_CSTK
	if( cdfcn==&CollisionDetection::IsInCollision_cstk )
		return isInsideObs_cstk(cfg, env, _cdsetid, _cdInfo);
#endif

#ifdef USE_VCLIP
	if( cdfcn==&CollisionDetection::IsInCollision_vclip )
		return isInsideObs_vclip(cfg, env, _cdsetid, _cdInfo);
#endif

#ifdef USE_RAPID
	if( cdfcn==&CollisionDetection::IsInCollision_RAPID )
		return isInsideObs_RAPID(cfg, env, _cdsetid, _cdInfo);
#endif

#ifdef USE_PQP
	if( cdfcn==&CollisionDetection::IsInCollision_PQP )
		return isInsideObs_PQP(cfg, env, _cdsetid, _cdInfo);
#endif

	cerr<<"! Error: CollisionDetection::isInsideObstacle"<<endl;
	return false;
}

#ifdef USE_CSTK
	bool CollisionDetection::
	isInsideObs_cstk(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo)
	{
		cerr<<"isInsideObs_cstk: Not implemeneted yet"<<endl;
		exit(1);
	}
#endif //USE_CSTK

#ifdef USE_VCLIP
	bool CollisionDetection:: 
	isInsideObs_vclip(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo)
	{
		cerr<<"isInsideObs_vclip: Not implemeneted yet"<<endl;
		exit(1);
	}
#endif //USE_VCLIP

#ifdef USE_RAPID
	bool CollisionDetection::
	isInsideObs_RAPID(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo)
	{
		cerr<<"isInsideObs_vclip: Not implemeneted yet"<<endl;
		exit(1);
	}
#endif //USE_RAPID

#ifdef USE_PQP
	bool CollisionDetection::
	isInsideObs_PQP(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo)
	{
		if( m_pRay==NULL ) m_pRay=BuildPQPSegment(1e10,0,0);
		assert(m_pRay!=NULL);
		int nmulti = env->GetMultiBodyCount();
		int robot = env->GetRobotIndex();
		PQP_REAL t[3]={cfg.GetData()[0],cfg.GetData()[1],cfg.GetData()[2]};
		static PQP_REAL r[3][3]={{1,0,0},{0,1,0},{0,0,1}};

		for( int i=0;i<nmulti;i++ ){//for all obstacles
			if( i==robot ) continue;
			MultiBody * obstacle=env->GetMultiBody(i);
			for(int j=0; j<obstacle->GetBodyCount(); j++)
			{
				PQP_Model * obst = obstacle->GetBody(j)->GetPqpBody();
				GMSPolyhedron & poly=obstacle->GetBody(j)->GetPolyhedron();
				Transformation &t2 = obstacle->GetBody(j)->WorldTransformation();
				t2.orientation.ConvertType(Orientation::Matrix);
				double p2[3];
				for(int p=0; p<3; p++) p2[p] = t2.position[p];

				PQP_CollideResult result;
				PQP_Collide(&result,r,t,m_pRay,t2.orientation.matrix,p2,obst);
    
				//anaylize result (check if there are adjacent triangle)
				vector<int> tri;
				//for each tri
				for( int iT=0;iT<result.NumPairs();iT++ ){
					bool add=true;
					int * tri1=poly.polygonList[result.Id2(iT)].vertexList;
					//for each checked triangle
					for( int i=0;i<tri.size();i++){
						int * tri2=poly.polygonList[tri[i]].vertexList;
						//check if they share same vertices
						for(int itri1=0;itri1<3;itri1++){
							for(int itri2=0;itri2<3;itri2++){
								if( tri2[itri2]==tri1[itri1] ){
									add=false;
									break;
								}
							}
						}
						if( add==false ) break;
					}
					//Ok no one shares vertex with you...
					if( add==true ) tri.push_back(result.Id2(iT));
				}
				int size=tri.size();
				if( (tri.size()%2)==1 ) return true;
			}//end of each part of obs
		}//end of each obstacle

		return false;
	}//end of function

	PQP_Model * CollisionDetection::
	BuildPQPSegment(PQP_REAL dX, PQP_REAL dY, PQP_REAL dZ) const
	{
		//build a narrow triangle.
		PQP_Model * pRay = new PQP_Model();
		if( pRay==NULL ) return NULL;
    
		if( dY==0 && dZ==0 && dX==0 ) 
			cout<<"! CollisionDetection::BuildPQPRay Warning : All are [0]"<<endl;

		static PQP_REAL tiny_v=((double)1e-20)/LONG_MAX;
		static PQP_REAL pico_v=tiny_v/2;
		PQP_REAL p1[3] = { tiny_v, tiny_v, tiny_v };
		PQP_REAL p2[3] = { pico_v, pico_v, pico_v };
		PQP_REAL p3[3] = { dX, dY, dZ};
    
		pRay->BeginModel();
		pRay->AddTri(p1, p2, p3, 0);
		pRay->EndModel();
    
		return pRay;
	}
#endif //USE_PQP


bool
CollisionDetection::
clearanceAvailable() {
  vector<CD> cds = collisionCheckers.GetCDs();
  for(int i=0; i<cds.size(); i++)
    if(!strcmp(cds[i].GetName(),"cstk"))
      return true;
 
  return false;
};


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

bool
CollisionDetection::
IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo, int robot, int obstacle) {
	
    MultiBody *rob, *obst;
    rob = env->GetMultiBody(robot);
    obst = env->GetMultiBody(obstacle);
	
    return IsInCollision(env, _cdsetid, _cdInfo, rob, obst);
}


bool
CollisionDetection::
IsInCollision(Environment * env, SID _cdsetid, CDInfo& _cdInfo, 
			  MultiBody* rob, MultiBody* obst) {
	
    int nFreeRobot;
    nFreeRobot = rob->GetFreeBodyCount();
	
    vector<CD> cdset = collisionCheckers.GetCDSet(_cdsetid);
    for(int cd = 0 ; cd < cdset.size() ; cd++){
		
		CDF cdfcn = cdset[cd].GetCollisionDetection();
		int tp = cdset[cd].GetType();
		
		// Type Out: no collision sure; collision unsure.
		if((tp == Out) && (cdfcn(rob,obst,cdset[cd],_cdInfo) == false)){
			return false;
		}
		
		// Type In: no collision unsure; collision sure.
		if((tp == In) && (cdfcn(rob,obst,cdset[cd],_cdInfo) == true)){
			return true;
		}
		
		// Type Exact: no collision sure; collision sure.
		if(tp == Exact){
			if(cdfcn(rob,obst,cdset[cd],_cdInfo) == true){
				return true;
			}
			else{
				return false;
			}
		}
    }
    return true;
}


bool
CollisionDetection::
IsInCollision_boundingSpheres
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo){
	cout << endl << "boundingSpheres Collision Check invocation";
	Stats.IncNumCollDetCalls( "boundingSpheres" );
	return true;
}

bool
CollisionDetection::
IsInCollision_insideSpheres
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo){
	cout << endl << "insideSpheres Collision Check invocation";
	Stats.IncNumCollDetCalls( "insideSpheres" );
	return false;
}

bool
CollisionDetection::
IsInCollision_naive
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo){
	cout << endl << "naive Collision Check invocation";
	Stats.IncNumCollDetCalls( "naive" );
	return false;
}

bool
CollisionDetection::
IsInCollision_quinlan
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo){
	cout << endl << "Quinlan Collision Check invocation";
	Stats.IncNumCollDetCalls( "quinlan" );
	return false;
}


////////////////////////////////////////////////////////// VCLIP BEGIN
// hash table used by "vclip"

#ifdef USE_VCLIP
ClosestFeaturesHT closestFeaturesHT(3000);
bool
CollisionDetection::
IsInCollision_vclip
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo)
{
	Stats.IncNumCollDetCalls( "vclip" );
	
	Real dist;
	VclipPose X12;
	PolyTree *rob, *obst;
	Vect3 cp1, cp2;   // closest points between bodies, in local frame
	// we're throwing this info away for now
	
	if (_cdInfo.ret_all_info == true)
	{
		bool ret_val;
		ret_val = IsInColl_AllInfo_vclip(robot, obstacle, _cd, _cdInfo);
		return ret_val;
	}
	
	for(int i=0 ; i<robot->GetFreeBodyCount(); i++)
	{
		
		rob = robot->GetFreeBody(i)->GetVclipBody();
		
		for(int j=0; j<obstacle->GetBodyCount(); j++)
		{
			
			// if robot check self collision, skip adjacent links.
			if(robot == obstacle &&
				robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
			{
				continue;
			}
			
			obst = obstacle->GetBody(j)->GetVclipBody();
			X12 = GetVclipPose(robot->GetFreeBody(i)->WorldTransformation(),
				obstacle->GetBody(j)->WorldTransformation());
			dist = PolyTree::vclip(rob,obst,X12,closestFeaturesHT, cp1, cp2);
			
			if(dist < 0.0)   // once was < 0.001 ????
			{
				return true;
			}
		} // end for j
	} // end for i
	
	return false;
} // end IsInCollision_vclip()

//////////////////////////////////////////////////////////////////////////
// IsInColl_AllInfo_vclip
// written by Brent, June 2000
//
// This function will fill in as much of _cdInfo as possible
// w.r.t. the robot and obstacle sent
// Notice each obstacle could change the results in _cdInfo
// Trace back to general IsInCollision call to see how it all
// gets updated correctly.
//////////////////////////////////////////////////////////////////////////
bool
CollisionDetection::
IsInColl_AllInfo_vclip
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo)
{
	Real dist, min_dist_so_far;
	VclipPose X12;
	PolyTree *rob, *obst;
	Vect3 cp1, cp2;   // closest points between bodies, in local frame
	Vector3D robot_pt, obs_pt;
	bool ret_val;
	
	ret_val = false;
	min_dist_so_far = MaxDist;  // =  1e10 by CollisionDetection.h
	
	for(int i=0 ; i<robot->GetFreeBodyCount(); i++)
	{
		rob = robot->GetFreeBody(i)->GetVclipBody();
		
		for(int j=0; j<obstacle->GetBodyCount(); j++)
		{
			
			// if robot check self collision, skip adjacent links.
			if(robot == obstacle &&
				robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
			{
				continue;
			}
			
			obst = obstacle->GetBody(j)->GetVclipBody();
			X12 = GetVclipPose(robot->GetFreeBody(i)->WorldTransformation(),
				obstacle->GetBody(j)->WorldTransformation());
			dist = PolyTree::vclip(rob,obst,X12,closestFeaturesHT, cp1, cp2);
			
			if ( dist < 0.0 )  
			{
				ret_val = true;
			}
			
			if (dist < min_dist_so_far)
			{
				min_dist_so_far = dist;
				// _cdInfo.nearest_obst_index =  is set by IsInCollision()
				// which called this function - look there for more info
				_cdInfo.min_dist = dist;
				
				// change a 3 elmt array to Vector3D class
				robot_pt[0] = cp1[0];
				robot_pt[1] = cp1[1];
				robot_pt[2] = cp1[2];
				
				obs_pt[0] = cp2[0];
				obs_pt[1] = cp2[1];
				obs_pt[2] = cp2[2];
				
				//cout << "CD method, robot pt = " << robot_pt << endl;
				//cout << "CD method, obs_pt = " << obs_pt << endl;
				
				// transform points to world coords
				// using *_pt vars in case overloaded * was not done well.
				_cdInfo.robot_point = robot->GetFreeBody(i)->WorldTransformation() * robot_pt;
				_cdInfo.object_point = obstacle->GetBody(j)->WorldTransformation() * obs_pt;
				
			}
		} // end for j
	} // end for i
	
	return ret_val;
} // end IsInColl_AllInfo_vclip()

////////////////////////////////////////////////////////// VCLIP END
#endif

#ifdef USE_RAPID
bool
CollisionDetection::
IsInCollision_RAPID
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo){
	Stats.IncNumCollDetCalls( "RAPID" );
	
    RAPID_model *rob, *obst;
	
    if (_cdInfo.ret_all_info == true)
    {
		cout << endl;
		cout << "Currently unable to return ALL info using RAPID cd." << endl;
		cout << "Defaulting to minimal information." << endl;
	}
	
    for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
		
		rob = robot->GetFreeBody(i)->GetRapidBody();
		
		for(int j=0; j<obstacle->GetBodyCount(); j++){
			
            // if robot check self collision, skip adjacent links.
            if(robot == obstacle &&
				robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
				continue;
			
            obst = obstacle->GetBody(j)->GetRapidBody();
			Transformation &t1 = robot->GetFreeBody(i)->WorldTransformation();
			Transformation &t2 = obstacle->GetBody(j)->WorldTransformation();
			t1.orientation.ConvertType(Orientation::Matrix);
			t2.orientation.ConvertType(Orientation::Matrix);
			double p1[3], p2[3];
			for(int p=0; p<3; p++) {
				p1[p] = t1.position[p];
				p2[p] = t2.position[p];
			}
			
			if(RAPID_Collide(t1.orientation.matrix, p1, rob,
				t2.orientation.matrix, p2, obst, RAPID_FIRST_CONTACT)) {
				cout << "Error in CollisionDetection::RAPID_Collide, RAPID_ERR_COLLIDE_OUT_OF_MEMORY"
					<< RAPID_Collide(t1.orientation.matrix, p1, rob, t2.orientation.matrix, p2, obst, RAPID_FIRST_CONTACT) << endl;
				exit(1);
			}
			if(RAPID_num_contacts) {
				return true;
            }
			
		}
    }
    return false;
}
#endif

#ifdef USE_PQP
bool
CollisionDetection::
IsInCollision_PQP
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo){
	Stats.IncNumCollDetCalls( "PQP" );
	
    PQP_Model *rob, *obst;
    PQP_CollideResult result;
	
    if (_cdInfo.ret_all_info == true)
    {
		/////////////////////////////////////////////////////////////////////////////
		//	11/2/01 modified by Jyh-Ming Lien : Start
		/////////////////////////////////////////////////////////////////////////////
		
		PQP_DistanceResult res;
		double min_dist_so_far = MaxDist;
		Vector3D robot_pt, obs_pt;
		bool ret_val=false;
		
		//for each part of robot
		for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
			
			rob = robot->GetFreeBody(i)->GetPqpBody();
			Transformation &t1 = robot->GetFreeBody(i)->WorldTransformation();
			t1.orientation.ConvertType(Orientation::Matrix);
			double p1[3]; for(int ip1=0; ip1<3; ip1++) p1[ip1] = t1.position[ip1];
			
			//for each part of obstacle
			for(int j=0; j<obstacle->GetBodyCount(); j++){
				
				// if robot check self collision, skip adjacent links.
				if(robot == obstacle &&
					robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
				{
					continue;
				}
				
				obst = obstacle->GetBody(j)->GetPqpBody();
				
				Transformation &t2 = obstacle->GetBody(j)->WorldTransformation();
				t2.orientation.ConvertType(Orientation::Matrix);
				double p2[3]; for(int ip2=0; ip2<3; ip2++) p2[ip2] = t2.position[ip2];
				
				if(PQP_Distance(&res,t1.orientation.matrix,p1,rob,
					t2.orientation.matrix,p2,obst,0.0,0.0))
				{
					cout << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"<<endl;
					exit(1);
				}
				
				if ( res.Distance() < 0.0 )  
				{
					ret_val = true;
				}
				
				if( res.Distance()<min_dist_so_far ){
					// _cdInfo.nearest_obst_index =  is set by IsInCollision()
					// which called this function - look there for more info
					min_dist_so_far=res.Distance();
					_cdInfo.min_dist = min_dist_so_far;
					
					// change a 3 elmt array to Vector3D class
					for( int k=0;k<3;k++ ){
						robot_pt[k] = res.P1()[k];
						obs_pt[k] = res.P2()[k];
					}
					
					// transform points to world coords
					// using *_pt vars in case overloaded * was not done well.
					_cdInfo.robot_point = robot->GetFreeBody(i)->WorldTransformation() * robot_pt;
					_cdInfo.object_point = obstacle->GetBody(j)->WorldTransformation() * obs_pt;
				}
			}//end of each part of obs
		}//end of each part of robot
		return ret_val;
		
		/////////////////////////////////////////////////////////////////////////////
		//	11/2/01 modified by Jyh-Ming Lien : End
		/////////////////////////////////////////////////////////////////////////////
	}
	
    for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
		
		rob = robot->GetFreeBody(i)->GetPqpBody();
		
		for(int j=0; j<obstacle->GetBodyCount(); j++){
			
            // if robot check self collision, skip adjacent links.
            if(robot == obstacle &&
				robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
				continue;
			
            obst = obstacle->GetBody(j)->GetPqpBody();
			Transformation &t1 = robot->GetFreeBody(i)->WorldTransformation();
			Transformation &t2 = obstacle->GetBody(j)->WorldTransformation();
			t1.orientation.ConvertType(Orientation::Matrix);
			t2.orientation.ConvertType(Orientation::Matrix);
			double p1[3], p2[3];
			for(int p=0; p<3; p++) {
				p1[p] = t1.position[p];
				p2[p] = t2.position[p];
			}
			
			if(PQP_Collide(&result, t1.orientation.matrix, p1, rob,
				t2.orientation.matrix, p2, obst, PQP_FIRST_CONTACT)) {
				cout << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"
					<< PQP_Collide(&result, t1.orientation.matrix, p1, rob, t2.orientation.matrix, p2, obst, PQP_FIRST_CONTACT) << endl;
				exit(1);
			}
			if(result.Colliding()) {
				return true;
            }
			
		}
    }
    return false;
}
#endif

#ifdef USE_CSTK
bool
CollisionDetection::
IsInCollision_cstk
(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo){
	
    Stats.IncNumCollDetCalls( "cstk" );
	
	// Identity in row-major order
    double linTrans[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};
	
    void *rob, *obst;
    cstkReal tolerance = 0.001;
    cstkReal dist;
	
    for(int i = 0 ; i < robot->GetFreeBodyCount(); i++){
		
        rob = robot->GetFreeBody(i)->GetCstkBody();
		SetLineTransformation(robot->GetFreeBody(i)->WorldTransformation(), linTrans);
        cstkUpdateLMovableBody(rob, linTrans);
		
        for(int k = 0 ; k < obstacle->GetBodyCount(); k++){
			
            // if robot check self collision, skip adjacent links.
            if(robot == obstacle &&
				robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(k)) )
				continue;
			
            obst = obstacle->GetBody(k)->GetCstkBody();
            if(!obstacle->GetBody(k)->IsFixedBody()) { // FreeBody, cstkUpdate
				SetLineTransformation(obstacle->GetBody(k)->WorldTransformation(), linTrans);
                cstkUpdateLMovableBody(obst, linTrans);
            }
			
            dist = cstkBodyBodyDist(rob, obst, 500, 0, NULL, 0, NULL);
            if(dist < tolerance){
                return (true);
            }
        }
    }
    return false;
}

double
CollisionDetection::
cstkDistance(MultiBody* robot, MultiBody* obstacle){
	
    Stats.IncNumCollDetCalls( "cstkDistance" );
	
    double linTrans[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};  // Identity in row-major order
	
    void *rob, *obst;
    cstkReal tmp, dist = MaxDist;
	
    for(int i = 0 ; i < robot->GetFreeBodyCount(); i++){
        rob = robot->GetFreeBody(i)->GetCstkBody();
		SetLineTransformation(robot->GetFreeBody(i)->WorldTransformation(), linTrans);
        cstkUpdateLMovableBody(rob, linTrans);
        for(int k = 0 ; k < obstacle->GetFixedBodyCount(); k++){
            obst = obstacle->GetFixedBody(k)->GetCstkBody();
            tmp = cstkBodyBodyDist(rob, obst, 500, 0, NULL, 0, NULL);
            if(tmp < dist){
                dist = tmp;
            }
        }
    }
    return dist;
}

#endif




/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CD
//
/////////////////////////////////////////////////////////////////////

CD::
CD() {
	strcpy(name,"");
	collision_detection = 0;
	cdid = INVALID_EID;
	type = -1;
}

CD::
~CD() {
}

CD&
CD::
operator=(const CD& _cd) {
	strcpy(name,_cd.name);
	return *this;
};

bool
CD::
operator==(const CD& _cd) const
{
	if ( strcmp(name,_cd.name) != 0 ) {
		return false;
	} else if ( !strcmp(name,"boundingSpheres") ) {
		return true;
	} else if ( !strcmp(name,"insideSpheres") ) {
		return true;
	} else if ( !strcmp(name,"naive") ) {
		return true;
	} else if ( !strcmp(name,"quinlan") ) {
		return true;
	} else if ( !strcmp(name,"cstk") ) {
#ifdef USE_CSTK
		return true;
#else
		cout << "Current compilation does not include CSTK." << endl <<
			"Please recompile with CSTK if you'd like to use cstk option\n";
		exit(5);
		
#endif
	} else if ( !strcmp(name,"vclip") ) {
#ifdef USE_VCLIP
		return true;
#else
		cout << "Current compilation does not include VCLIP." << endl
			<< "Please recompile with VCLIP if you'd like to use VCLIP\n";
		exit(5);
#endif
	} else if ( !strcmp(name,"RAPID") ) {
#ifdef USE_RAPID
		return true;
#else
		cout << "Current compilation does not include RAPID."
			<< endl << "Please recompile with RAPID\n";
		exit(5);
		
#endif
	} else if ( !strcmp(name,"PQP") ) {
#ifdef USE_PQP
		return true;
#else
		cout << "Current compilation does not include PQP."
			<< endl << "Please recompile with PQP\n";
		exit(5);
		
#endif
	} else {
		return false;
	}
};

char*
CD::
GetName() const {
	return const_cast<char*>(name);
};

CDF
CD::
GetCollisionDetection(){
	return collision_detection;
};

int
CD::
GetType() const {
	return type;
};

ostream& operator<< (ostream& _os, const CD& cd){
	_os<< cd.GetName();
	if ( !strcmp(cd.GetName(),"boundingSpheres")){
		_os << ", Type = " << cd.GetType();
	}
	if ( !strcmp(cd.GetName(),"insideSpheres")){
		_os << ", Type = " << cd.GetType();
	}
	if ( !strcmp(cd.GetName(),"naive") ){
		_os << ", Type = " << cd.GetType();
	}
	if ( strstr(cd.GetName(),"quinlan") ){
		_os << ", Type = " << cd.GetType();
	}
	
	if ( strstr(cd.GetName(),"cstk") ){
#ifdef USE_CSTK
		_os << ", Type = " << cd.GetType();
#else
		cout << "Current compilation does not include CSTK." << endl <<
			"Please recompile with CSTK if you'd like to use cstk\n";
		exit(5);
#endif
	}
	if ( strstr(cd.GetName(),"vclip") ){
#ifdef USE_VCLIP
		_os << ", Type = " << cd.GetType();
#else
		cout << "Current compilation does not include VCLIP." <<
			"Please recompile with VCLIP if you'd like to use vclip option\n";
		exit(5);
#endif
	}
	if ( strstr(cd.GetName(),"RAPID") ){
#ifdef USE_RAPID
		_os << ", Type = " << cd.GetType();
#else
		cout << "Current compilation does not include RAPID." << endl
			<< "Please recompile with RAPID\n";
		exit(5);
#endif
		
	}
	if ( strstr(cd.GetName(),"PQP") ){
#ifdef USE_PQP
		_os << ", Type = " << cd.GetType();
#else
		cout << "Current compilation does not include PQP." << endl
			<< "Please recompile with PQP\n";
		exit(5);
#endif
	}
	return _os;
};



/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CDSets
//
/////////////////////////////////////////////////////////////////////

//==================================
// CDSets class Methods: Constructors and Destructor
//==================================

CDSets::
CDSets(){
};

CDSets::
~CDSets(){
};


//===================================================================
// CDSets class Methods: Adding CDs, Making & Modifying CD sets
//===================================================================

int
CDSets::
AddCD(const char* _cdinfo) {
	
	//Could we use 
	SID sid = MakeCDSet(_cdinfo);
	SetIDs--;
	return DeleteOSet(sid);        // delete the set, but not elements
};

int
CDSets::
AddCDToSet(const SID _sid, const EID _cdid) {
	return AddElementToOSet(_sid,_cdid);
};

int
CDSets::
DeleteCDFromSet(const SID _sid, const EID _cdid) {
	return DeleteElementFromOSet(_sid,_cdid);
};


SID
CDSets::
MakeCDSet(const char* _cdlist){
	
	///Modified for VC
	istrstream  is( (char *)_cdlist);
	if (!is) {
		cout << endl << "In MakeCDSet: can't open instring: " << _cdlist ;
		return INVALID_SID;
	}
	
	return MakeCDSet(is);
};


SID
CDSets::
MakeCDSet(const EID _eid) {
	return MakeOSet(_eid);
}

SID
CDSets::
MakeCDSet(const vector<EID> _eidvector) {
	return MakeOSet(_eidvector);
}

SID
CDSets::
MakeCDSet(istream& _myistream) {
	char cdname[100];
	vector<EID> cdvec;  // vector of cdids for this set
	
	while ( _myistream >> cdname ) { // while cds to process...
		
		if (!strcmp(cdname,"boundingSpheres")) {              // boundingSpheres
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = &CollisionDetection::IsInCollision_boundingSpheres;
			cd1.type = Out;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
			
		} else if (!strcmp(cdname,"insideSpheres")) {              // insideSpheres
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = &CollisionDetection::IsInCollision_insideSpheres;
			cd1.type = In;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
			
		} else if (!strcmp(cdname,"naive")) {              // naive
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = &CollisionDetection::IsInCollision_naive;
			cd1.type = Exact;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
			
		} else if (!strcmp(cdname,"quinlan")) {          // Quinlan
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = & CollisionDetection::IsInCollision_quinlan;
			cd1.type = Exact;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
			
		} else if (!strcmp(cdname,"cstk")) {          // cstk
#ifdef USE_CSTK
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = & CollisionDetection::IsInCollision_cstk;
			cd1.type = Exact;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
#else
			cout << "Current compilation does not include CSTK." << endl
				<< "Please recompile with CSTK\n";
			exit(5);
#endif
			
		} else if (!strcmp(cdname,"vclip")) {          // vclip
#ifdef USE_VCLIP
			
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = & CollisionDetection::IsInCollision_vclip;
			cd1.type = Exact;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
#else
			cout << "Current compilation does not include VCLIP."
				<< "Please recompile with VCLIP if you'd like to use vclip\n";
			exit(5);
#endif
			
			
		} else if (!strcmp(cdname,"RAPID")) {          // RAPID
#ifdef USE_RAPID
			
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = & CollisionDetection::IsInCollision_RAPID;
			cd1.type = Exact;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
			
#else
			cout << "Current compilation does not include RAPID." << endl
				<< "Please recompile with RAPID if you'd like to use rapid\n";
			exit(5);
#endif
			
		} else if (!strcmp(cdname,"PQP")) {          // PQP
#ifdef USE_PQP
			
			CD cd1;
			strcpy(cd1.name,cdname);
			cd1.collision_detection = & CollisionDetection::IsInCollision_PQP;
			cd1.type = Exact;
			cd1.cdid = AddElementToUniverse(cd1);
			if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			cdvec.push_back( cd1.cdid );
			
#else
			cout << "Current compilation does not include PQP." << endl
				<< "Please recompile with PQP if you'd like to use PQP\n";
			exit(5);
#endif
			
		} else {
			cout << "INVALID: Collision Detection name = " << cdname;
			exit(-1);
		}
  } // end while
  
  return MakeOSet(cdvec);
}


int
CDSets::
DeleteCDSet(const SID _sid) {
	return DeleteOSet(_sid);
};

//===================================================================
// CDSets class Methods: Getting Data & Statistics
//===================================================================

CD
CDSets::
GetCD(const EID _cdid) const {
	return GetElement(_cdid);
};

vector<CD>
CDSets::
GetCDs() const {
	vector<CD> elts2;
	vector<pair<EID,CD> > elts1 = GetElements();
	for (int i=0; i < elts1.size(); i++)
		elts2.push_back( elts1[i].second );
	return elts2;
};

vector<CD>
CDSets::
GetCDSet(const SID _sid) const {
	vector<CD> elts2;
	vector<pair<EID,CD> > elts1 = GetOSet(_sid);
	for (int i=0; i < elts1.size(); i++)
		elts2.push_back( elts1[i].second );
	return elts2;
};


vector<pair<SID,vector<CD> > >
CDSets::
GetCDSets() const {
	
	vector<pair<SID,vector<CD> > > s2;
	vector<CD> thesecds;
	
	vector<pair<SID,vector<pair<EID,CD> > > > s1 = GetOSets();
	
	for (int i=0; i < s1.size(); i++)  {
		thesecds.erase(thesecds.begin(),thesecds.end());
		for (int j=0; j < s1[i].second.size(); j++ )
			thesecds.push_back (s1[i].second[j].second);
		s2.push_back( pair<SID,vector<CD> > (s1[i].first,thesecds) );
	}
	return s2;
};


//===================================================================
// CDSets class Methods: Display, Input, Output
//===================================================================

void
CDSets::
DisplayCDs() const{
	DisplayElements();
};

void
CDSets::
DisplayCD(const EID _cdid) const{
	DisplayElement(_cdid);
};

void
CDSets::
DisplayCDSets() const{
	DisplayOSets();
};

void
CDSets::
DisplayCDSet(const SID _sid) const{
	DisplayOSet(_sid);
};

void
CDSets::
WriteCDs(const char* _fname) const {
	
	ofstream  myofstream(_fname);
	if (!myofstream) {
		cout << endl << "In WriteCDS: can't open outfile: " << _fname ;
	}
	WriteCDs(myofstream);
	myofstream.close();
};

void
CDSets::
WriteCDs(ostream& _myostream) const {
	
	vector<CD> cds = GetCDs();
	
	_myostream << endl << "#####CDSTART#####";
	_myostream << endl << cds.size();  // number of cds
	
	//format: CD_NAME (a string) CD_PARMS (double, int, etc)
	for (int i = 0; i < cds.size() ; i++) {
		_myostream << endl;
		_myostream << cds[i].name << " ";
	}
	_myostream << endl << "#####CDSTOP#####";
};

void
CDSets::
ReadCDs(const char* _fname) {
	
	ifstream  myifstream(_fname);
	if (!myifstream) {
		cout << endl << "In ReadCDs: can't open infile: " << _fname ;
		return;
	}
	ReadCDs(myifstream);
	myifstream.close();
};

void
CDSets::
ReadCDs(istream& _myistream) {
	
	char tagstring[100];
	char cddesc[100];
	int  numCDs;
	
	_myistream >> tagstring;
	if ( !strstr(tagstring,"CDSTART") ) {
		cout << endl << "In ReadCDs: didn't read CDSTART tag right";
		return;
	}
	
	_myistream >> numCDs;
	_myistream.getline(cddesc,100,'\n');  // throw out rest of this line
	for (int i = 0; i < numCDs; i++) {
        _myistream.getline(cddesc,100,'\n');
        AddCD(cddesc);
	}
	
	_myistream >> tagstring;
	if ( !strstr(tagstring,"CDSTOP") ) {
		cout << endl << "In ReadCDs: didn't read CDSTOP tag right";
		return;
	}
};



/////////////////////////////////////////////////////////////////////////
// BEGIN CLASS CDInfo
/////////////////////////////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////////////////////////////
CDInfo::CDInfo()
{
	colliding_obst_index = -1;
	
	ret_all_info = false;
	nearest_obst_index = -1;
	min_dist = MaxDist;      // =  1e10 by CollisionDetection.h
	robot_point = 0;         // hope Vector3D class defined well
	object_point = 0;
	
} // end constructor

/////////////////////////////////////////////////////////////////////////
// Destructor
/////////////////////////////////////////////////////////////////////////
CDInfo::~CDInfo()
{
	// do nothing
}

/////////////////////////////////////////////////////////////////////////
// ResetVars
// 
// Re-Init vars as done by constructor
/////////////////////////////////////////////////////////////////////////
void CDInfo::ResetVars()
{
	colliding_obst_index = -1;
	
	ret_all_info = false;
	nearest_obst_index = -1;
	min_dist = MaxDist;      // =  1e10 by CollisionDetection.h
	robot_point = 0;         // hope Vector3D class defined well
	object_point = 0;
	
} // end ResetVars

/////////////////////////////////////////////////////////////////////////
// END CLASS CDInfo
/////////////////////////////////////////////////////////////////////////
