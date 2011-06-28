/**
 * MedialAxisUtility.h
 *  - Utility that pushes a cfg to the medial axis
 *  - Contains both exact and approx collision computation
 * Author: Kasra Manavi
 * Date  : 06/2011
 */

#ifndef MedialAxisUtil_h
#define MedialAxisUtil_h

#include "util.h"
#include "CollisionDetectionValidity.hpp"

class Cfg;
class Environment;
class CDInfo;
class DistanceMetric;
class Stat_Class;

//***********************************//
// Main Push To Medial Axis Function //
//***********************************//
template <class CFG>
bool PushToMedialAxis(MPProblem* mp, Environment* env, CFG& cfg, Stat_Class& stats, string str_vcm, 
											string str_dm, bool c_exact, int clearance, bool p_exact, int penetration, bool use_bbx) {

	// Initialization
	std::string call("MedialAxisUtility::PushToMedialAxis()");
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetDMMethod(str_dm);
	ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);

	// Variable Initialization
  CDInfo   tmpInfo;
  bool     inside, found, pushed=true, to_wall=false;

  // Reset cdInfo and set flag to return all info, setup origin
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;

	// If inside an obstacle, push to the outside
	inside = vcm->isInsideObstacle(cfg,env,tmpInfo);
	if (inside)
		pushed = PushFromInsideObstacle(mp,cfg,env,stats,str_vcm,str_dm,p_exact,penetration,to_wall);
	if ( !pushed ) return false;

	// Now that cfg is free, find medial axis
	found = FindMedialAxisCfg(mp,cfg,env,stats,str_vcm,str_dm,c_exact,clearance,use_bbx);
	if ( !found )  return false;
	return true;
} // END pushToMedialAxis

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
// TODO: push to the wall, not just outside                      //
//***************************************************************//
template <class CFG>
bool PushFromInsideObstacle(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats,
														string str_vcm, string str_dm, bool p_exact, int penetration, bool to_wall) {	
	// Initialization
	string call("MedialAxisUtility::pushFromInsideObstacle");
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetDMMethod(str_dm);
	ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);
	
	// Variables
  CDInfo   tmpInfo;
  Vector3D trans_dir;
  CFG      tmp_cfg=cfg, held_cfg=cfg, trans_cfg=cfg;
  bool     valid, in_bbx, inside=true, tmp_inside=true;
	double   step_size=1.0, step_inc=2.0, step_dec=0.5;
	double   res = env->GetPositionRes();

	// Origin and tmpInfo Setup
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;
	Cfg* orig = cfg.CreateNewCfg();
	orig->subtract( *orig, *orig);

	// Determine direction to move, set trans_cfg, and scale by resolution
	valid = CalculateCollisionInfo(mp,cfg,env,stats,tmpInfo,str_vcm,str_dm,p_exact,0,penetration,true);
	if (!valid)	return false;

	trans_dir = tmpInfo.object_point - tmpInfo.robot_point;
	if (trans_dir[0] == 0 &&
			trans_dir[1] == 0 )
		return false;

	trans_cfg.SetSingleParam(0, trans_dir[0]);
	trans_cfg.SetSingleParam(1, trans_dir[1]);
	trans_cfg.SetSingleParam(2, trans_dir[2]);	
	dm->ScaleCfg(env, res, *orig, trans_cfg);
	
	// Determine gap for medial axis
  while ( tmp_inside ) {
		held_cfg.equals(tmp_cfg);
		tmp_cfg.multiply(trans_cfg,step_size);
		tmp_cfg.add(cfg, tmp_cfg);
		tmp_inside = vcm->isInsideObstacle(tmp_cfg,env,tmpInfo);
		in_bbx = tmp_cfg.InBoundingBox(env);

		// If tmp is valid, move gap on to next step
		if ( vc->IsValid(vcm,tmp_cfg,env,stats,tmpInfo,true,&call) && in_bbx ) {
			step_size *= step_inc;
		} else { // Else reduce step size or fall out of the loop
			if ( in_bbx ) {
				if ( inside ) step_size *= step_inc;
				else          step_size *= step_dec;
			} else {
				if ( inside ) return false;	// Straight from obstacle to out of BBX			
				else          step_size *= step_dec;
			}
		}
	}
	
	// If specified, push back to the wall
	if (to_wall) { } // TODO::MAY COME IN HANDY

	// Broke out between held_cfg and 
	cfg.equals(tmp_cfg);
	return true;
}

//***************************************************************//
// Find Medial Axis Cfg                                          //
//   In this function, is a wrapper function that determines the //
// gap and the medial axis cfg, makes for cleaner funciton call  //
//***************************************************************//
template <class CFG>
bool FindMedialAxisCfg(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats,
											 string str_vcm, string str_dm, bool c_exact, int clearance, bool use_bbx) {
	// Variables
	bool gap, found;
	CFG startcfg, endcfg;
	startcfg.equals(cfg);
	endcfg.equals(cfg);

	// Find gap and determine medial axis cfg
	gap = DetermineMedialAxisGap(mp,startcfg,endcfg,env,stats,str_vcm,str_dm,c_exact,clearance,use_bbx);
	if ( !gap )	  return false;
	found = DetermineMedialAxisCfg(mp,startcfg,endcfg,cfg,env,stats,str_vcm,str_dm,c_exact,clearance,use_bbx);
	if ( !found )	return false;
	return true;
}

//***************************************************************//
// Determine Medial Axis Gap                                     //
//   In this function, a startcfg and transition cfg are used to //
// determine a gap which contains the medial axis. This gap will //
// be assigned to the start and end cfgs.                        //
//***************************************************************//
template <class CFG>
bool DetermineMedialAxisGap(MPProblem* mp, CFG& startcfg, CFG& endcfg, Environment* env, Stat_Class& stats,
														string str_vcm, string str_dm, bool c_exact, int clearance, bool use_bbx) {
	// Initialization
	string call("MedialAxisUtility::determineMedialAxisGap");
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetDMMethod(str_dm);
	ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);
	
	// Variables
  Vector3D trans_dir;
  CDInfo   tmpInfo, oldInfo, peekInfo;
  CFG      trans_cfg, held_cfg, tmp_cfg, peek_cfg;
	double   step_size=1, step_inc=2.0, step_dec=0.9;
	bool     found_gap=false, in_bbx=true, good_tmp=true, good_peek=true, valid;
	double   res = env->GetPositionRes();
	bool     inside = vcm->isInsideObstacle(startcfg,env,peekInfo);

	// tmpInfo and Origin Setup
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;
	Cfg* orig = trans_cfg.CreateNewCfg();
	orig->subtract( *orig, *orig);

	// Determine direction to move, set trans_cfg, and scale by resolution
	valid = CalculateCollisionInfo(mp,startcfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
	if (!valid)	return false;
	trans_dir = tmpInfo.robot_point - tmpInfo.object_point;
	if (trans_dir[0] == 0 && trans_dir[1] == 0 )
		return false;
	trans_cfg.SetSingleParam(0, trans_dir[0]);
	trans_cfg.SetSingleParam(1, trans_dir[1]);
	trans_cfg.SetSingleParam(2, trans_dir[2]);	
	dm->ScaleCfg(env, res, *orig, trans_cfg);

	// Initialize temp Info
	oldInfo = tmpInfo;
	peekInfo = tmpInfo;

	// Determine gap for medial axis
  while ( (tmpInfo.min_dist  >= oldInfo.min_dist) ||
					(peekInfo.min_dist >= oldInfo.min_dist) ) {
		oldInfo = tmpInfo;
		tmp_cfg.multiply(trans_cfg,step_size);
		tmp_cfg.add(endcfg, tmp_cfg);
		peek_cfg.multiply(trans_cfg,step_size+res);
		peek_cfg.add(endcfg, peek_cfg);
		inside = vcm->isInsideObstacle(tmp_cfg,env,tmpInfo);
		in_bbx = tmp_cfg.InBoundingBox(env);

		// If inside obstacle or out of the bbx, step back
		if ( inside || !in_bbx) {
			if ( !use_bbx && !in_bbx) {
				startcfg.equals(endcfg);
				endcfg.equals(tmp_cfg);
				return false;
			}
			step_size *= step_dec;
		} else {
			// If tmp is valid, move gap on to next step
			if ( vc->IsValid(vcm,tmp_cfg ,env,stats,tmpInfo,true,&call) ) {
				found_gap = true;
				good_tmp  = CalculateCollisionInfo(mp,tmp_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
				good_peek = CalculateCollisionInfo(mp,peek_cfg,env,stats,peekInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
				if (!good_tmp || !good_peek) return false;
				step_size *= step_inc;
				held_cfg.equals(startcfg);
				startcfg.equals(endcfg);
				endcfg.equals(tmp_cfg);
			} else { // Else reduce step size or fall out of the loop
				step_size *= step_dec;
			}
		}
	}

	if ( use_bbx ) {
		// Shorten Gap (best gap between old and new or held and peek)
		peek_cfg.add(startcfg, endcfg);
		peek_cfg.divide(peek_cfg,2);
		good_peek = CalculateCollisionInfo(mp,peek_cfg,env,stats,peekInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
		if ( good_peek && oldInfo.min_dist > peekInfo.min_dist ) {
			startcfg.equals(held_cfg);
			endcfg.equals(peek_cfg);
		}
	}
	return true;
}

//*************************************************************************//
// Determine Medial Axis Cfg                                               //
//   In this function startcfg and endcfg define a gap over a medial axis. //
// The medial axis cfg is search for in a modified binomial fashion. The   //
// gap is divided into 4 parts and the 2 consecutive parts which define a  //
// peak are then used in the next iteration of the gap. Once the gap has   //
// reached a size of epslilon, the reference finalcfg is set and returned  //
//*************************************************************************//
template <class CFG>
bool DetermineMedialAxisCfg(MPProblem* mp, CFG& startcfg, CFG& endcfg, CFG& finalcfg, Environment* env, 
														Stat_Class& stats, string str_vcm, string str_dm, bool c_exact, int clearance, bool use_bbx) {
	// Initialization
	string call("MedialAxisUtility::determineMedialAxisCfg");
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetDMMethod(str_dm);
	ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);

	// Variables
	CDInfo tmpInfo;
	CFG mid1cfg, mid2cfg, mid3cfg;
	int peak=-1, bad_peaks=0, attempts=0;
	int max_attempts=25, max_bad_peaks=5;
	double max_dist=0.0, gap_dist=0.0; 
	double res=env->GetPositionRes(), eps=(c_exact)?res/10.0:res;
	bool get_info=true, peaked=false;
	vector<double> dists(5,0), deltas(4,0);

	// Initialize mid2cfg and dist values, return false on error
	mid2cfg.add(startcfg, endcfg ); mid2cfg.divide(mid2cfg,2);
	if ( CalculateCollisionInfo(mp,endcfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) )
		dists[4] = tmpInfo.min_dist;
	else return false;
	if ( CalculateCollisionInfo(mp,mid2cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) )
		dists[2] = tmpInfo.min_dist;
	else return false;
	if ( CalculateCollisionInfo(mp,startcfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) )
		dists[0] = tmpInfo.min_dist;
	else return false;

	do { // Modified Binomial search to find peaks
		attempts++;
		mid1cfg.add(startcfg, mid2cfg); mid1cfg.divide(mid1cfg,2);
		mid3cfg.add( mid2cfg, endcfg ); mid3cfg.divide(mid3cfg,2);
		
		// Get new Info for old_middle and and new_middle CFGS
		if ( CalculateCollisionInfo(mp,mid1cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) )
			dists[1] = tmpInfo.min_dist;
		else return false;
		if ( CalculateCollisionInfo(mp,mid3cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) )
			dists[3] = tmpInfo.min_dist;
		else return false;
		
		// Compute Deltas and Max Distance
		deltas.clear();
		for ( int i=0; i<dists.size()-1; i++ ) {
			double tmp = dists[i+1] - dists[i];
			deltas.push_back(tmp);
			max_dist = (max_dist>dists[i+1])?max_dist:dists[i+1];
		}
		
		// Determine Peak
		if ( deltas[0] > 0 && deltas[1] < 0 && dists[1] == max_dist) {
			peak = 1;
			endcfg.equals(mid2cfg); 
			mid2cfg.equals(mid1cfg);
			dists[4] = dists[2];
			dists[2] = dists[1];
		} else if ( deltas[1] > 0 && deltas[2] < 0 && dists[2] == max_dist) {
			peak = 2;
			startcfg.equals(mid1cfg);
			endcfg.equals(mid3cfg);
			dists[0] = dists[1];
			dists[4] = dists[3];
		} else if ( deltas[2] > 0 && deltas[3] < 0 && dists[3] == max_dist) {
			peak = 3;
			startcfg.equals(mid2cfg);
			mid2cfg.equals(mid3cfg);
			dists[0] = dists[2];
			dists[2] = dists[3];
		} else {
			// No peak found, recalculate old, mid and new clearance
			bad_peaks++;
			peak = -1;
			if ( CalculateCollisionInfo(mp,startcfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) &&
					 dists[0] >= tmpInfo.min_dist )
				dists[0] = tmpInfo.min_dist;
			if ( CalculateCollisionInfo(mp,mid2cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) &&
					 dists[2] >= tmpInfo.min_dist )
				dists[2] = tmpInfo.min_dist;
			if ( CalculateCollisionInfo(mp,endcfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) &&
					 dists[4] >= tmpInfo.min_dist )
				dists[4] = tmpInfo.min_dist;
		}
 
		// Check if minimum gap distance has been acheived
		gap_dist = dm->Distance(env,startcfg,endcfg);
		if ( eps >= gap_dist )
			peaked = true;
		
	} while ( !peaked && get_info && 
						attempts < max_attempts && 
						bad_peaks < max_bad_peaks );
	
	// Set finalcfg and return
	finalcfg.equals((use_bbx)?mid2cfg:startcfg);
	return (bad_peaks==max_bad_peaks)?false:true;
}

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
template <class CFG>
bool CalculateCollisionInfo(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats, CDInfo& cdInfo, 
														string str_vcm, string str_dm, bool exact, int clearance, int penetration, bool use_bbx) {
	if ( exact )
		return GetExactCollisionInfo(mp,cfg,env,stats,cdInfo,str_vcm,use_bbx);
	else
		return GetApproxCollisionInfo(mp,cfg,env,stats,cdInfo,str_vcm,str_dm,clearance,penetration,use_bbx);
}

//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
template <class CFG>
bool GetExactCollisionInfo(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats,
													 CDInfo& cdInfo, string str_vcm, bool use_bbx) {
	// Setup Validity Checker
	std::string call("MedialAxisUtility::getExactCollisionInfo");
	ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);
  cdInfo.ResetVars();
  cdInfo.ret_all_info = true;
	
	// If not in BBX or valid, return false
	if ( !cfg.InBoundingBox(env) || 
			 !(vc->IsValid(vcm,cfg,env,stats,cdInfo,true,&call)) ) 
		return false;
	
	// If not using the bbx, done
	if ( !use_bbx )
		return true;

	// CFG is now know as good, get BBX and ROBOT info                                                                                                      
	boost::shared_ptr<BoundingBox> bbx = env->GetBoundingBox();
	boost::shared_ptr<MultiBody> robot = env->GetMultiBody(env->GetRobotIndex());
	std::pair<double,double> bbx_range;

	// Find closest point between robot and bbx, set if less than min dist from obstacles
	for(int m=0; m < robot->GetFreeBodyCount(); m++) {
		GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
		for(size_t j = 0 ; j < poly.vertexList.size() ; j++){
			for (int k=0; k<cfg.posDOF(); k++) { // For all positional DOFs
				bbx_range = bbx->GetRange(k);
				if ( (poly.vertexList[j][k]-bbx_range.first) < cdInfo.min_dist ) {
					cdInfo.object_point[k] = bbx_range.first; // Lower Bound
					cdInfo.min_dist = poly.vertexList[j][k]-bbx_range.first;
					cdInfo.nearest_obst_index = -(k+1); // Different bbx face
				}
				if ( (bbx_range.second-poly.vertexList[j][k]) < cdInfo.min_dist ) {
					cdInfo.object_point[k] = bbx_range.second; // Upper Bound
					cdInfo.min_dist = bbx_range.second-poly.vertexList[j][k];
					cdInfo.nearest_obst_index = -(k+1); // Different bbx face
				}
			}
		}
	}
	return (cdInfo.min_dist>=0)?true:false;
} // END getExactCollisionInfo

//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best calididate.            //
//*********************************************************************//
template <class CFG>
bool GetApproxCollisionInfo(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats,
														CDInfo& cdInfo, string str_vcm, string str_dm, 
														int clearance, int penetration, bool use_bbx) {
	
	// Initialization
  string call = "MedialAxisUtility::getApproxCollisionInfo";
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetDMMethod(str_dm);
	ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);
 
	// If not in BBX, error out and return false                                                                                                            
	if ( !cfg.InBoundingBox(env) ) {
		cdInfo.min_dist = 0;
		return false;
	}
	
	// Check validity to get cdInfo, return false if not valid
  cdInfo.ResetVars();
  cdInfo.ret_all_info = true;
	if ( !(vc->IsValid(vcm,cfg,env,stats,cdInfo,true,&call)) ) {
		cdInfo.min_dist = 0;
		return false;
	}

	// Set Robot point
	cdInfo.robot_point[0] = cfg.GetSingleParam(0);
	cdInfo.robot_point[1] = cfg.GetSingleParam(1);
	cdInfo.robot_point[2] = cfg.GetSingleParam(2);

	// Setup Major Variables and Constants:
	CDInfo tmpInfo;                                                               // Temp CDInfo
	double res = env->GetPositionRes();                                           // Resolution
  bool init_validity = vc->IsValid(vcm,cfg,env,stats,tmpInfo,true,&call);       // Initial Validity
	double incr_max = env->GetMultiBody(env->GetRobotIndex())->GetMaxAxisRange(); // Max Step Size
	int maxNumSteps = 25;                                                         // Max Steps (Refinement)
	
  // Generate 'num_rays' random directions at a distance 'dist' away from 'cfg'
	int num_rays = (vcm->isInsideObstacle(cfg,env,tmpInfo))?penetration:clearance;
  double dist = 100*res;
  vector<Cfg*> directions, cand_in, cand_out;
  Cfg* tmp;

	// Setup translation rays
  for(int i=0; i<num_rays; i++) {
    tmp = cfg.CreateNewCfg();
    tmp->GetRandomRay(dist, env, dm);
    directions.push_back(tmp);
  }

	// Setup incremental steps for each direction
	int n_ticks;
	vector<Cfg*> tick;
	vector<pair<Cfg*,double> > incr;
	vector<Cfg*>::iterator I;
	for(I = directions.begin(); I != directions.end(); ++I) {
		tmp = cfg.CreateNewCfg();
		tick.push_back(tmp);
		tmp = cfg.CreateNewCfg();
		tmp->FindIncrement(cfg, **I, &n_ticks, res, res);
		incr.push_back(make_pair(tmp,
														 tmp->OrientationMagnitude() +
														 tmp->PositionMagnitude()));
	}
	
	// Setup to step out along each direction:
	vector<bool> ignored(directions.size(), false);
	bool stateChangedFlag = false, curr_validity;
	size_t lastLapIndex = -1, end_index = -1;
	int iterations = 0, max_iterations=100;

	// Shoot out each ray to determine the candidates
	while ( !stateChangedFlag && iterations < max_iterations) {
		iterations++;
		// For Eact Ray
		for ( size_t i=0; i<directions.size(); ++i ) {
			// If ignored, continue on to next ray, else increment
			if ( ignored[i] ) continue;
			tick[i]->Increment(*incr[i].first);
			curr_validity = vc->IsValid(vcm,*tick[i],env,stats,tmpInfo,true,&call);				
			// Determine tick status
			if ( !use_bbx ) {
				if ( (curr_validity != init_validity) &&
						 (tick[i]->InBoundingBox(env)) ) { // If Validity State has changed and inside bbx
					if ( lastLapIndex != i ) {
						Cfg* cand_o = tick[i]->CreateNewCfg();
						Cfg* cand_i = tick[i]->CreateNewCfg();
						cand_i->subtract(*cand_i,*incr[i].first);
						cand_out.push_back(cand_o);
						cand_in.push_back(cand_i);
					} else
						stateChangedFlag = true; // lastLapIndex == i, made full pass
					if ( lastLapIndex == end_index )  // Set lastLapIndex to first ray changing state
						lastLapIndex = i;
				}
			} else {
				if ( curr_validity != init_validity ) { // If Validity State has changed
					if ( lastLapIndex != i ) {
						Cfg* cand_o = tick[i]->CreateNewCfg();
						Cfg* cand_i = tick[i]->CreateNewCfg();
						cand_i->subtract(*cand_i,*incr[i].first);
						cand_out.push_back(cand_o);
						cand_in.push_back(cand_i);
					} else
						stateChangedFlag = true; // lastLapIndex == i, made full pass
					if ( lastLapIndex == end_index )  // Set lastLapIndex to first ray changing state
						lastLapIndex = i;
				}
			}

			// Once validity changes, exit loop
			if ( stateChangedFlag )
				break;
			// If increment still less than a max bound
			if ( incr[i].second < incr_max ) {
				incr[i].first->multiply(*incr[i].first,2);
				incr[i].second *= 2;
			}
		} // End for
	} // End while

	// If no candidates found, return false;
	if (cand_in.size() == 0) {
		cout << "cand_in.size = 0" << endl;
		return false;
	}

	// Refine each candidate to find the best one
	cdInfo.min_dist = res*100;
	bool mid_validity;

	Cfg* innerCfg = cand_in[0]->CreateNewCfg();
	Cfg* outerCfg = cand_out[0]->CreateNewCfg();
	Cfg* 	 midCfg = cfg.CreateNewCfg();
	
	for ( size_t i=0; i<cand_out.size(); i++ ) {
		innerCfg = cand_in[i]->CreateNewCfg();
		outerCfg = cand_out[i]->CreateNewCfg();
		midCfg   = cfg.CreateNewCfg();
		int j=0;

		do {
			// Calculate new midpoint
			midCfg->add(*innerCfg,*outerCfg);
			midCfg->divide(*midCfg,2);
			mid_validity = vc->IsValid(vcm,*midCfg,env,stats,tmpInfo,true,&call);
			
			if ( !(midCfg->InBoundingBox(env)) || 
					 (mid_validity != init_validity) ) // If Outside BBX or Validity has changed
				outerCfg->equals(*midCfg);
			else
				innerCfg->equals(*midCfg);
			
		} while ((dm->Distance(env,*innerCfg,*outerCfg) > res) &&
						 (++j <= maxNumSteps));

		// Determine if new clearance is better than existing solution
		if ( cdInfo.min_dist > dm->Distance(env,*innerCfg,cfg) ) {
			tmp = innerCfg->CreateNewCfg();
			if ( tmp->InBoundingBox(env) ) {
				cdInfo.min_dist = dm->Distance(env,*innerCfg,cfg);
				// Set Object Point
				cdInfo.object_point[0] = innerCfg->GetSingleParam(0);
				cdInfo.object_point[1] = innerCfg->GetSingleParam(1);
				cdInfo.object_point[2] = innerCfg->GetSingleParam(2);
			} else
				cdInfo.min_dist = 0;
		}
	}	 

	// Clean up allocated memory
	delete innerCfg;
	delete midCfg;
	delete outerCfg;
	for(vector<pair<Cfg*, double> >::iterator I = incr.begin(); I != incr.end(); ++I) delete I->first;
	for(vector<Cfg*>::iterator I = directions.begin(); I != directions.end(); ++I)    delete *I;
	for(vector<Cfg*>::iterator I = cand_out.begin(); I != cand_out.end(); ++I)        delete *I;
	for(vector<Cfg*>::iterator I = cand_in.begin(); I != cand_in.end(); ++I)          delete *I;
	for(vector<Cfg*>::iterator I = tick.begin(); I != tick.end(); ++I)                delete *I;	
	return (cdInfo.min_dist == 0)?false:true;
} // END getApproxCollisionInfo
#endif
