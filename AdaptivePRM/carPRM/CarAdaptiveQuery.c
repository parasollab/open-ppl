////////////////////////////////////////////////////
//
//  CarAdaptiveQuery.c
//
//  derived class of AdaptiveQuery.h
//  specific for car-like robots
//
/////////////////////////////////////////////////////


#include"CarAdaptiveQuery.h"
#include"DistanceMetrics.h"
#include"Environment.h"
#include"util.h"
#include"GraphAlgo.h"
#include"CfgCarLike.h"
#include"CurveWeight.h"


CarAdaptiveQuery::CarAdaptiveQuery(Input *input, MyQueryCmds *Qinput, CollisionDetection* cd, 
		 DistanceMetric* dm, LocalPlanners* lp,ConnectMapNodes* cn) :
	 AdaptiveQuery(input, Qinput, cd, dm, lp, cn) {}


bool
CarAdaptiveQuery::PerformQuery(CollisionDetection* cd, ConnectMapNodes* cn,
			       LocalPlanners* lp, DistanceMetric* dm) {
  return AdaptiveQuery::PerformQuery(cd, cn, lp, dm);
}


typedef pair<Cfg, Cfg> CfgPairType;
typedef pair<CfgPairType, double> DIS_TYPE;
bool DIST_Compare(const DIST_TYPE &_cc1, const DIST_TYPE &_cc2) {
        return (_cc1.second < _cc2.second ) ;
}


void
FindKClosestPairs(vector<CfgPairType> &kp, Environment *_env,DistanceMetric * dm,
                  const int kclosest, const Cfg& c, const vector<Cfg> &vertices, SID dmsid){
  if(kclosest > vertices.size()) {
     for(int i=0; i<vertices.size(); ++i)
        kp.push_back(CfgPairType(c,vertices[i]));
     return;
  }

  vector<DIS_TYPE> tmp;
  int i;
  for(i=0; i<kclosest; ++i)
        tmp.push_back(DIS_TYPE(CfgPairType(), MAX_DIST));
  for(i=0; i<vertices.size(); ++i) {
     double dist = dm->Distance(_env, c, vertices[i], dmsid);
     if(dist < tmp[kclosest-1].second ) {
        tmp[kclosest-1] = DIST_TYPE(CfgPairType(c, vertices[i]), dist);
        sort(tmp.begin(), tmp.end(), ptr_fun(DIST_Compare));
     }
  }
 
  for(i=0; i<kclosest; ++i)
     kp.push_back(tmp[i].first);
}


bool 
CarAdaptiveQuery::PerformQuery(Cfg _start, Cfg _goal, CollisionDetection *cd,
     ConnectMapNodes*cn, LocalPlanners *lp, DistanceMetric* dm, SID _lpsid, vector<Cfg>* _path) {

  Environment *env = rdmp.GetEnvironment();

  // check if start & goal satisfy query requirements.
  if(!queryReq.isNodeValid(_start, env, cd, cdsetid) || 
     !queryReq.isNodeValid(_goal, env, cd, cdsetid) ) 
	return false;

  // check to see if bad nodes need to be thrown away
  if(checkAllNodes) {
    vector<Cfg> allnodes;
    rdmp.m_pRoadmap->GetVerticesData(allnodes);
    removeBadNodes(allnodes, env, cd, cdsetid);
  }

  LPInfo sci, ci;
  sci.positionRes =ci.positionRes = rdmp.GetEnvironment()->GetPositionRes();
  sci.orientationRes =ci.orientationRes = rdmp.GetEnvironment()->GetOrientationRes();
  sci.checkCollision = ci.checkCollision = true;
  sci.cdsetid = ci.cdsetid = cdsetid;
  sci.dmsetid = ci.dmsetid = dmsetid;
  sci.savePath = false;
  ci.savePath = true;

  if(!rdmp.m_pRoadmap->IsVertex(_start) && !rdmp.m_pRoadmap->IsVertex(_goal)) {
     cout << "connecting start and goal to the roadmap ... " << endl;
     // connect start & goal to the roadmap.
     vector<Cfg> vertices;
     rdmp.m_pRoadmap->GetVerticesData(vertices);
     //int connectNum = vertices.size()/100;
     const int kclosest = 200;  //100 > connectNum ? 100 : connectNum;
     rdmp.m_pRoadmap->AddVertex(_start);
     rdmp.m_pRoadmap->AddVertex(_goal);
     vector<CfgPairType> kp;
     FindKClosestPairs(kp, env, dm, kclosest, _start, vertices, dmsetid);
     FindKClosestPairs(kp, env, dm, kclosest, _goal, vertices, dmsetid);
     if(((CarQueryReq*)queryReq.GetIQueryReq())->CheckTurningRadius()) {
       CfgCarLike::turningRadius = ((CarQueryReq*)queryReq.GetIQueryReq())->GetTurningRadius();
     }
     for(int i=0; i<kp.size(); ++i) {
       if(((CarQueryReq*)queryReq.GetIQueryReq())->CheckTurningRadius()) {
	 CfgCarLike::AddCurvedEdges(rdmp.m_pRoadmap, kp[i].first, kp[i].second);
       } else {
	 if(lp->IsConnected(&rdmp,cd,dm,kp[i].first,kp[i].second, _lpsid,&sci))
	   rdmp.m_pRoadmap->AddEdge(kp[i].first, kp[i].second, sci.edge);
       }
     }	
  }

  // CfgCarLike should have a member called turning_radius that can be set
  // in both mkmp and query phases with different values.

  // remove all the edges with too small turing radius.
  if(((CarQueryReq*)queryReq.GetIQueryReq())->CheckTurningRadius()) {
     vector< pair<pair<VID,VID>, WEIGHT> > elist;
     rdmp.m_pRoadmap->GetEdges(elist);
     CurveWeight* tmp = new CurveWeight();
     for(int i=0; i<elist.size(); ++i) {
         tmp = (CurveWeight*)elist[i].second.GetIWeight();

	 if (tmp->Radius() < ((CarQueryReq*)queryReq.GetIQueryReq())->GetTurningRadius())
            rdmp.m_pRoadmap->DeleteEdge(elist[i].first.first, elist[i].first.second);
     }
  }

  while(IsSameCC(*(rdmp.m_pRoadmap), _start, _goal)) {
      cout << endl << "start and goal seem connected!!" << endl;
      vector< pair<Cfg,WEIGHT> > rp;
      FindPathDijkstra(*(rdmp.m_pRoadmap), _start, _goal, rp);

      int i;
      _path->push_back(_start);
      bool failedToRecreatePath = false;
      bool badNodeFound = false;

      if (!checkAllNodes) { //remove bad nodes in path first
        vector<Cfg> pathnodes;
        for (i=0; i<rp.size(); i++)
          pathnodes.push_back(rp[i].first);
        badNodeFound = removeBadNodes(pathnodes, env, cd, cdsetid);
      }

      if (!badNodeFound) { //didn't remove a bad node, so check edges
        for (i=0; i<rp.size()-1; i++) {
          if ( !(GetPathSegment(rp[i].first,rp[i+1].first,cd,lp,dm,rp[i].second,&ci) )) {
               cout << endl << "In PerformQuery: can't recreate path" << endl;
               rdmp.m_pRoadmap->DeleteEdge(rp[i].first,rp[i+1].first);
	       failedToRecreatePath = true;
	       _path->erase(_path->begin(), _path->end());
	       break;
          } else {
	       if(!queryReq.isEdgeValid(ci.path, env, cd, cdsetid)) {
		  cout << endl << "In PerformQuery: can't meet query requirements!" 
		       << endl;
                  rdmp.m_pRoadmap->DeleteEdge(rp[i].first,rp[i+1].first);
		  failedToRecreatePath = true;
                  _path->erase(_path->begin(), _path->end());
                  break;
	       } else {
                  _path->insert(_path->end(), ci.path.begin(),ci.path.end());
	       }
          }
	}
      } else { //removed a bad node, so couldn't recreate path
	cout << endl << "In PerformQuery: can't meet query requirements!" << endl;
	failedToRecreatePath = true;
      }

      if(failedToRecreatePath) continue;

      #if INTERMEDIATE_FILES
          //-----------------------------------------------------
          // Print out all cfg's both in human & pv readable form
          //-----------------------------------------------------
          vector<Cfg> _mapcfgs;                         // pv (.path)
          ofstream os("mapnodes.cfgs");
          for(i=0; i<rp.size(); ++i) {
              _mapcfgs.push_back(rp[i].first);
              os << rdmp.m_pRoadmap->GetVID(rp[i].first) << " ";
              os << rp[i].second << "  ";
              os << rp[i].first << "\n";
          }
          WritePathConfigurations("mapnodes.path", _mapcfgs, env);
      #endif

      // Optimize the path to make it smooth: continous curvature
      bool optimizePathCubicSpline = false;
      if(optimizePathCubicSpline) {
         vector<Cfg> splinePath;  
	 splinePath.push_back(_start);
	 CfgCarLike::GetCubicSplinePath(splinePath, rp, ci.positionRes, ci.orientationRes);
	 if(queryReq.isEdgeValid(splinePath, env, cd, cdsetid)) {
	    cout << "Path optimized with cubic splines!!!" << endl;
	    //_path = &splinePath;
	    _path->erase(_path->begin(), _path->end());
	    _path->insert(_path->end(), splinePath.begin(), splinePath.end());
	 }
      }

      return true;
  }

  return false;
}


bool
CarAdaptiveQuery::GetPathSegment(Cfg _c1, Cfg _c2, CollisionDetection *cd,
               LocalPlanners * lp,DistanceMetric * dm,WEIGHT _weight, LPInfo* _ci){

   // clear the potential old storage.
   _ci->path.erase(_ci->path.begin(),_ci->path.end());

   // this is new. Used only for car-like robots.
   if( ((CarQueryReq*)queryReq.GetIQueryReq())->CheckTurningRadius() ) { //Car like robot
      CfgCarLike::GetEdgeNodesSequence(_c1, _c2, _weight, _ci->path,
				       _ci->positionRes, _ci->orientationRes);
      return true;
   }

   return AdaptiveQuery::GetPathSegment(_c1, _c2, cd, lp, dm, _weight, _ci);
}
