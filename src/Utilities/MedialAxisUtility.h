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
bool PushToMedialAxis(MPProblem* mp, Environment* env, CFG& cfg, Stat_Class& stats, string str_vcm, string str_dm, 
                      bool c_exact, int clearance, bool p_exact, int penetration, bool use_bbx, double eps, int h_len, bool m_debug) {
  // Initialization
  std::string call("MedialAxisUtility::PushToMedialAxis()");
  if (m_debug) cout << endl << call << endl << "CFG: " << cfg;
  ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);
  bool   inside, in_collision, found, pushed=true;
  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;

  // If invalid, push to the outside of the obstacle
  inside       = vcm->isInsideObstacle(cfg,env,tmpInfo);
  in_collision = !(vc->IsValid(vcm,cfg,env,stats,tmpInfo,true,&call));
  if (m_debug) cout << " Inside/In-Collision: " << inside << "/" << in_collision << endl;
  if (inside || in_collision)
    pushed = PushFromInsideObstacle(mp,cfg,env,stats,str_vcm,str_dm,p_exact,penetration,m_debug);
  if ( !pushed ) return false;

  // Cfg is free, find medial axis
  found = PushCfgToMedialAxis(mp,cfg,env,stats,str_vcm,str_dm,c_exact,clearance,use_bbx,eps,h_len,m_debug);
  if ( !found )  { 
    if (m_debug) cout << "Not found!! ERR in pushtomedialaxis" << endl; 
    return false; 
  }

  if (m_debug) cout << "FINAL CFG: " << cfg << endl << call << "::END" << endl << endl;
  return true;
} // END pushToMedialAxis

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//
template <class CFG>
bool PushFromInsideObstacle(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats,
                            string str_vcm, string str_dm, bool p_exact, int penetration, bool m_debug) {	
  // Initialization
  string call("MedialAxisUtility::PushFromInsideObstacle");
  if (m_debug) cout << call << endl << " CFG: " << cfg << endl;
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetDMMethod(str_dm);
  ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);
	
  // Variables
  CDInfo   tmpInfo;
  Vector3D trans_dir, dif;
  vector<double> g_dif;
  bool     valid, in_bbx;
  CFG      end_cfg=cfg, tmp_cfg=cfg, held_cfg=cfg, trans_cfg=cfg;
  double   step_size=1.0, res=env->GetPositionRes(), factor; // TODO: ori_res=env->GetOrientationRes()
  bool     init_validity = vc->IsValid(vcm,cfg,env,stats,tmpInfo,true,&call);
  bool     tmp_validity = false, prev_validity = false;

  // If in collision (using the exact case), must use approx
  if ( init_validity == false )
    p_exact = false;

  // Get Collision info, if not computable, exit
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;
  valid = CalculateCollisionInfo(mp,cfg,env,stats,tmpInfo,str_vcm,str_dm,p_exact,0,penetration,true);
  if (!valid)	return false;

  // Determine direction to move
  trans_dir = tmpInfo.object_point - tmpInfo.robot_point;
  for (int i=0; i<trans_cfg.DOF(); i++) {
    if ( i < trans_cfg.posDOF() ) trans_cfg.SetSingleParam(i, trans_dir[i]);
    else                          trans_cfg.SetSingleParam(i, 0.0);
  }
  dif[0] = trans_cfg.GetSingleParam(0);
  dif[1] = trans_cfg.GetSingleParam(1);
  dif[2] = trans_cfg.GetSingleParam(2);
  factor = res/sqrt(dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2]);
  trans_cfg.multiply(trans_cfg,factor);
  if (m_debug) cout << "TRANS CFG: " << trans_cfg << endl;

  // Check if valid and outside obstacle
  while ( !tmp_validity ) {
    held_cfg.equals(tmp_cfg);
    tmp_cfg.multiply(trans_cfg,step_size);
    tmp_cfg.add(cfg, tmp_cfg);
    tmp_validity = vc->IsValid(vcm,tmp_cfg,env,stats,tmpInfo,true,&call);
    tmp_validity = tmp_validity && !vcm->isInsideObstacle(tmp_cfg,env,tmpInfo);
    if ( tmp_validity ) {
      tmp_validity = tmp_validity && prev_validity;
      if ( !prev_validity )
        prev_validity = true;
    }
    in_bbx = tmp_cfg.InBoundingBox(env);
    if ( !in_bbx ) { 
      if (m_debug) cout << "Fell out of BBX, error out... " << endl;
      return false;
    }
    step_size += 1.0;
  }
  cfg.equals(tmp_cfg);
  if (m_debug) cout << "FINAL CFG: " << cfg << " steps: " << step_size-1.0 << endl << call << "::END " << endl;
  return true;
}

//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//
template <class CFG>
bool PushCfgToMedialAxis(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats,
                         string str_vcm, string str_dm, bool c_exact, int clearance, 
                         bool use_bbx, double eps, int h_len, bool m_debug) {
  // Initialization
  string call("MedialAxisUtility::PushCfgToMedialAxis");
  if (m_debug) cout << call << endl << "CFG: " << cfg << " eps: " << eps << endl;
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetDMMethod(str_dm);
  ValidityChecker<CFG>*             vc  = mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(str_vcm);
	
  // Variables
  Vector3D trans_dir, dif;
  vector<double> g_dif;
  CDInfo   tmpInfo;
  CFG      trans_cfg, tmp_cfg, peek_cfg, held_cfg;
  double   step_size=1.0, factor, res=env->GetPositionRes();// TODO: ori_res=env->GetOrientationRes();
  bool     in_bbx=true, good_tmp=true, valid, inside=vcm->isInsideObstacle(cfg,env,tmpInfo);
  if ( inside ) return false;

  // tmpInfo and Origin Setup
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;
  tmp_cfg.equals(cfg);

  // Determine direction to move and clearance
  valid = CalculateCollisionInfo(mp,cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
  if ( !valid ) return false;

  trans_dir = tmpInfo.robot_point - tmpInfo.object_point;
  for (int i=0; i<trans_cfg.DOF(); i++) {
    if ( i < trans_cfg.posDOF() ) trans_cfg.SetSingleParam(i, trans_dir[i]);
    else                          trans_cfg.SetSingleParam(i, 0.0);
  }
  dif[0] = trans_cfg.GetSingleParam(0);
  dif[1] = trans_cfg.GetSingleParam(1);
  dif[2] = trans_cfg.GetSingleParam(2);
  factor = res/sqrt(dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2]);
  trans_cfg.multiply(trans_cfg,factor);
  if (m_debug) cout << "TRANS CFG: " << trans_cfg << endl;

  // Initialize temp Info
  int seg_len=h_len, pos_cnt, neg_cnt, steps_taken;
  bool peek_found = false, broke=false;
  //vector<double> seg_dists(seg_len,0), seg_deltas(seg_len-1,0);
  vector<double> seg_dists;
  vector<CFG> seg_cfgs;
  double max_dist=0.0;

  // Determine gap for medial axis
  while ( !peek_found ) {

    tmp_cfg.multiply(trans_cfg,step_size);
    tmp_cfg.add(cfg, tmp_cfg);
    inside = vcm->isInsideObstacle(tmp_cfg,env,tmpInfo);
    in_bbx = tmp_cfg.InBoundingBox(env);

    // If inside obstacle or out of the bbx, step back
    if ( inside || !in_bbx) {
      if ( !in_bbx && !use_bbx ) return false;
      if ( seg_cfgs.size() > 0 ) break;
      else                       return false;
    } else {
    // If tmp is valid, move gap on to next step
      if ( vc->IsValid(vcm,tmp_cfg,env,stats,tmpInfo,true,&call) ) {
        if (m_debug) cout << "TMP CFG: " << tmp_cfg;
        good_tmp = CalculateCollisionInfo(mp,tmp_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
        if ( !good_tmp && broke ) return false; // If can't compute twice in a row, error out.
        broke = !good_tmp;
        if ( good_tmp ) {
          if (m_debug) cout << "  clearance: " << tmpInfo.min_dist;
          ++steps_taken;
          if ( int(seg_dists.size()) > seg_len ) {
            seg_dists.erase(seg_dists.begin());
            seg_cfgs.erase(seg_cfgs.begin());
          }
          seg_dists.push_back(tmpInfo.min_dist);
          seg_cfgs.push_back(tmp_cfg);
        } else {
          if (m_debug) cout << "BROKE!" << endl;
        }
        step_size += 1.0;

        max_dist = seg_dists[0]; pos_cnt=0; neg_cnt=0;
        for ( int i=0; i<int(seg_dists.size())-1; i++ ) {
          double tmp = seg_dists[i+1] - seg_dists[i];
          if ( tmp > 0.0 ) ++pos_cnt;
          if ( tmp < 0.0 ) ++neg_cnt;
          max_dist = (max_dist>seg_dists[i+1])?max_dist:seg_dists[i+1];
        }
        if (m_debug) cout << "  Pos/Neg counts: " << pos_cnt << "/" << neg_cnt << endl;
        if ( (neg_cnt > 0) && 
             (neg_cnt >= pos_cnt) && 
             (seg_dists[0] >= seg_dists[seg_dists.size()-1]) ) {
          peek_found = true;
          if (m_debug) {
            cout << "Found peek!  Dists: ";
            for (int i=0; i<int(seg_dists.size()); i++)
              cout << seg_dists[i] << " ";
            cout << endl;
          }
        }
      } else { // Else reduce step size or fall out of the loop
        return false;
      }
    }
  }

  // Variables for modified binary search
  CFG start_cfg, mid_s_cfg, mid_m_cfg, mid_e_cfg, endin_cfg;
  Cfg* dif_cfg = cfg.CreateNewCfg();
  double gap_dist, prev_gap;
  int attempts=0, max_attempts=20, bad_peaks=0, max_bad_peaks=10, peak;
  bool peaked=false;
  vector<double> dists(5,0), deltas(4,0);

  // Setup start, middle and end CFGs  
  start_cfg.equals(seg_cfgs[0]);
  endin_cfg.equals(seg_cfgs[seg_cfgs.size()-1]);
  mid_m_cfg.add(start_cfg, endin_cfg);
  mid_m_cfg.divide(mid_m_cfg,2);
  CalculateCollisionInfo(mp,start_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
  dists[0] = tmpInfo.min_dist;
  CalculateCollisionInfo(mp,mid_m_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
  dists[2] = tmpInfo.min_dist;
  CalculateCollisionInfo(mp,endin_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
  dists[4] = tmpInfo.min_dist;

  if (m_debug) cout << "start/mid/end: " << endl << start_cfg << endl << mid_m_cfg << endl << endin_cfg << endl;

  mid_s_cfg.add(start_cfg, mid_m_cfg);
  mid_s_cfg.divide(mid_s_cfg,2);
  mid_e_cfg.add(mid_m_cfg, endin_cfg);
  mid_e_cfg.divide(mid_e_cfg,2);

  if (m_debug) {
    cout << "dists: ";
    for (int i=0; i<int(dists.size()); i++)
      cout << dists[i] << " ";
    cout << endl;
  }
  // Get collision info and gap distance
  dif_cfg->subtract(start_cfg,endin_cfg);
  g_dif.clear();
  for(int i=0; i<start_cfg.DOF(); ++i) {
    if ( i < start_cfg.posDOF() ) gap_dist += (start_cfg.GetSingleParam(i) - endin_cfg.GetSingleParam(i))*
                                              (start_cfg.GetSingleParam(i) - endin_cfg.GetSingleParam(i));
    else                          gap_dist += (DirectedAngularDistance(start_cfg.GetSingleParam(i), endin_cfg.GetSingleParam(i)))*
                                              (DirectedAngularDistance(start_cfg.GetSingleParam(i), endin_cfg.GetSingleParam(i)));
  }
  gap_dist = sqrt(gap_dist);
  prev_gap = gap_dist;
  
  do { // Modified Binomial search to find peaks                                                                                                           
    attempts++;
    mid_s_cfg.add(start_cfg, mid_m_cfg); mid_s_cfg.divide(mid_s_cfg,2);
    mid_e_cfg.add(mid_m_cfg, endin_cfg); mid_e_cfg.divide(mid_e_cfg,2);

    CalculateCollisionInfo(mp,mid_s_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
    dists[1] = tmpInfo.min_dist;
    CalculateCollisionInfo(mp,mid_e_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx);
    dists[3] = tmpInfo.min_dist;

    // Compute Deltas and Max Distance
    deltas.clear();
    max_dist = (dists[0]);
    if (m_debug) cout << "Deltas: ";
    for ( int i=0; i<int(dists.size())-1; i++ ) {
      double tmp = dists[i+1] - dists[i];
      if (m_debug) cout << tmp << " ";
      deltas.push_back(tmp);
      max_dist = (max_dist>dists[i+1])?max_dist:dists[i+1];
    } 
    if (m_debug) cout << endl;

    // Determine Peak
    if ( deltas[0] > 0 && deltas[1] < 0 && dists[1] == max_dist) {
      peak = 1;
      endin_cfg.equals(mid_m_cfg);
      mid_m_cfg.equals(mid_s_cfg);
      dists[4] = dists[2];
      dists[2] = dists[1];
    } else if ( deltas[1] > 0 && deltas[2] < 0 && dists[2] == max_dist) {
      peak = 2;
      start_cfg.equals(mid_s_cfg);
      endin_cfg.equals(mid_e_cfg);
      dists[0] = dists[1];
      dists[4] = dists[3];
    } else if ( deltas[2] > 0 && deltas[3] < 0 && dists[3] == max_dist) {
      peak = 3;
      start_cfg.equals(mid_m_cfg);
      mid_m_cfg.equals(mid_e_cfg);
      dists[0] = dists[2];
      dists[2] = dists[3];
    } else {
      if ( deltas[0] > 0 && deltas[1] > 0 && deltas[2] > 0 && deltas[3] > 0) {
        peak = 3;
        start_cfg.equals(mid_m_cfg);
        mid_m_cfg.equals(mid_e_cfg);
        dists[0] = dists[2];
        dists[2] = dists[3];
      } else if ( deltas[0] < 0 && deltas[1] < 0 && deltas[2] < 0 && deltas[3] < 0) {
        peak = 1;
        endin_cfg.equals(mid_m_cfg);
        mid_m_cfg.equals(mid_s_cfg);
        dists[4] = dists[2];
        dists[2] = dists[1];
      } else {
        // No peak found, recalculate old, mid and new clearance
        bad_peaks++;
        peak = -1;
        if ( CalculateCollisionInfo(mp,start_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) &&
             dists[0] >= tmpInfo.min_dist )
          dists[0] = tmpInfo.min_dist;
        if ( CalculateCollisionInfo(mp,mid_m_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) &&
             dists[2] >= tmpInfo.min_dist )
          dists[2] = tmpInfo.min_dist;
        if ( CalculateCollisionInfo(mp,endin_cfg,env,stats,tmpInfo,str_vcm,str_dm,c_exact,clearance,0,use_bbx) &&
             dists[4] >= tmpInfo.min_dist )
          dists[4] = tmpInfo.min_dist;
      }
    }
    if (m_debug) cout << " peak: " << peak << "  cfg: " << mid_m_cfg;

    // Check if minimum gap distance has been acheived
    dif_cfg->subtract(start_cfg,endin_cfg);
    g_dif.clear(); 
    gap_dist = 0.0;
    for(int i=0; i<start_cfg.DOF(); ++i) {
      if ( i < start_cfg.posDOF() ) gap_dist += (start_cfg.GetSingleParam(i) - endin_cfg.GetSingleParam(i))*
                                                (start_cfg.GetSingleParam(i) - endin_cfg.GetSingleParam(i));
      else                          gap_dist += (DirectedAngularDistance(start_cfg.GetSingleParam(i), endin_cfg.GetSingleParam(i)))*
                                                (DirectedAngularDistance(start_cfg.GetSingleParam(i), endin_cfg.GetSingleParam(i)));
    }

    gap_dist = sqrt(gap_dist);
    if (m_debug) cout << " gap: " << gap_dist << endl;
    if ( eps >= gap_dist )
      peaked = true;

  } while ( !peaked &&
            attempts < max_attempts &&
            bad_peaks < max_bad_peaks );

  cfg.equals(mid_m_cfg);
  if (m_debug) cout << "FINAL CFG: " << cfg << " steps: " << step_size << endl;
  return true;
}

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
template <class CFG>
bool CalculateCollisionInfo(MPProblem* mp, CFG& cfg, Environment* env, Stat_Class& stats, CDInfo& cdInfo, 
                            string str_vcm, string str_dm, bool exact, int clearance, int penetration, bool use_bbx) {
  if ( exact ) return GetExactCollisionInfo(mp,cfg,env,stats,cdInfo,str_vcm,use_bbx);
  else         return GetApproxCollisionInfo(mp,cfg,env,stats,cdInfo,str_vcm,str_dm,clearance,penetration,use_bbx);
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
	
  // If not in BBX or valid, return false (IsValid gets cdInfo)
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
          cdInfo.nearest_obst_index = -(k+1);
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
 
  // If in BBX, check validity to get cdInfo, return false if not valid
  if ( !cfg.InBoundingBox(env) ) return false;
  cdInfo.ResetVars();
  cdInfo.ret_all_info = true;	
  bool init_inside   = vcm->isInsideObstacle(cfg,env,cdInfo);                  // Initially Inside Obst
  bool init_validity = vc->IsValid(vcm,cfg,env,stats,cdInfo,true,&call);       // Initial Validity
  bool init_inbbx    = cfg.InBoundingBox(env);
  init_validity = init_validity && !init_inside;
  if ( use_bbx ) init_validity = (init_validity && init_inbbx);

  // Set Robot point in positional space // TODO: Higher DOF
  for (int i=0; i<cfg.posDOF(); i++)
    cdInfo.robot_point[i] = cfg.GetSingleParam(i);

  // Setup Major Variables and Constants:
  CDInfo tmpInfo;
  Vector3D dif;
  int num_rays;
  double res=env->GetPositionRes();// TODO: ori_res=env->GetOrientationRes();

  // Generate 'num_rays' random directions at a distance 'dist' away from 'cfg'
  if ( init_validity ) num_rays = (init_inside)?penetration:clearance;
  else                 num_rays = penetration;    

  vector<Cfg*> directions, cand_in, cand_out;
  vector<Cfg*> tick;
  vector<pair<Cfg*,double> > incr;

  // Setup translation rays
  for(int i=0; i<num_rays; i++) {
    Cfg* tmp1 = cfg.CreateNewCfg();
    tick.push_back(tmp1);
    Cfg* tmp2 = cfg.CreateNewCfg();
    tmp2->GetRandomRayPos(res, env);
    incr.push_back(make_pair(tmp2, tmp2->OrientationMagnitude() +
                                   tmp2->PositionMagnitude()));
  }

  // Setup to step out along each direction:
  bool stateChangedFlag = false, curr_validity, curr_inside, curr_inbbx;
  size_t lastLapIndex = -1, end_index = -1;
  int iterations = 0, max_iterations=1000, maxNumSteps = 25; // Arbitrary //TODO: Smarter number

  // Shoot out each ray to determine the candidates
  while ( !stateChangedFlag && iterations < max_iterations ) {
    iterations++;
    // For Eact Ray
    for ( size_t i=0; i<incr.size(); ++i ) {
      // Increment
      tick[i]->Increment(*incr[i].first);
      curr_inside   = vcm->isInsideObstacle(*tick[i],env,cdInfo);
      curr_validity = vc->IsValid(vcm,*tick[i],env,stats,tmpInfo,true,&call);
      curr_inbbx    = tick[i]->InBoundingBox(env);
      curr_validity = curr_validity && !curr_inside;
      if ( use_bbx ) curr_validity = (curr_validity && curr_inbbx);
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
      if ( stateChangedFlag ) break; // Once validity changes, exit loop
    } // End for
  } // End while

  // If no candidates found, return false;
  if ( cand_in.size() == 0 ) return false;

  // Refine each candidate to find the best one
  cdInfo.min_dist = MAX_DBL;
  bool mid_validity;

  Cfg* innerCfg = cand_in[0]->CreateNewCfg();
  Cfg* outerCfg = cand_out[0]->CreateNewCfg();
  Cfg*   midCfg = cfg.CreateNewCfg();
  Cfg* dif_cfg  = cfg.CreateNewCfg();
  double tmp_dist;

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
           (mid_validity != init_validity) ) { // If Outside BBX or Validity has changed
        if ( init_validity ) outerCfg->equals(*midCfg);
        else                 innerCfg->equals(*midCfg);
      } else {
        if ( init_validity ) innerCfg->equals(*midCfg);
        else                 outerCfg->equals(*midCfg);
      }

      // Compute Real Dist
      dif_cfg->subtract(*innerCfg,*outerCfg);
      tmp_dist = 0.0;
      for (int j=0; j<cfg.posDOF(); j++)
        tmp_dist += dif_cfg->GetSingleParam(j)*dif_cfg->GetSingleParam(j);
      tmp_dist = sqrt(tmp_dist);
    } while ((tmp_dist > res/100) &&
             (++j <= maxNumSteps));

    midCfg->equals((init_validity)?*innerCfg:*outerCfg);
    dif_cfg->subtract(*midCfg,cfg);
    tmp_dist = 0.0;
    for (int j=0; j<cfg.posDOF(); j++)
      tmp_dist += dif_cfg->GetSingleParam(j)*dif_cfg->GetSingleParam(j);
    tmp_dist = sqrt(tmp_dist);

    // Determine if new clearance is better than existing solution
    if ( cdInfo.min_dist > tmp_dist ) {
      if ( midCfg->InBoundingBox(env) ) {
        cdInfo.min_dist = tmp_dist;
        for (int j=0; j<midCfg->posDOF();j++)
          cdInfo.object_point[j] = midCfg->GetSingleParam(j);
      } else cdInfo.min_dist = 0;
    }
  }
  return (cdInfo.min_dist == 0)?false:true;
} // END getApproxCollisionInfo
#endif
