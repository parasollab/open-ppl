////////////////////////////////////////////////////
//
//  AdaptiveQuery.c
//
//  derived class of Query
//
/////////////////////////////////////////////////////


#include"AdaptiveQuery.h"
#include"GraphAlgo.h"


AdaptiveQuery::AdaptiveQuery(Input *input, MyQueryCmds *Qinput, CollisionDetection* cd, 
		 DistanceMetric* dm, LocalPlanners* lp,ConnectMapNodes* cn) :
	 Query(input, Qinput, cd, dm, lp, cn) {

   char tmp[80];
   strcpy(tmp, Qinput->queryFile.GetValue());
   strcat(tmp, ".requirement"); 

   queryReq = QueryRequirementsObject(tmp);
   checkAllNodes = Qinput->checkAllNodes.GetValue();
}


AdaptiveQuery::~AdaptiveQuery() {}


// this is needed to make this method visible outside.
bool
AdaptiveQuery::PerformQuery(CollisionDetection* cd, ConnectMapNodes* cn, 
		      LocalPlanners* lp, DistanceMetric* dm) {

  if(query.size() == 1) { // only goal is provided.
     //vector<Cfg> vertices = rdmp.roadmap.GetVerticesData();
     vector<Cfg> vertices;
     rdmp.m_pRoadmap->GetVerticesData(vertices);

     // choose a random cfg as start...
     Cfg &start = vertices[(int)vertices.size()*drand48()];
     query.insert(query.begin(), start);
  }

  return Query::PerformQuery(cd,cn,lp,dm);
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
AdaptiveQuery::PerformQuery(Cfg _start, Cfg _goal, CollisionDetection *cd,
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

  if (!rdmp.m_pRoadmap->IsVertex(_start) && !rdmp.m_pRoadmap->IsVertex(_goal)) {
     // connect start & goal to the roadmap.
     vector<Cfg> vertices;
     rdmp.m_pRoadmap->GetVerticesData(vertices);
     int connectNum = vertices.size()/100;
     const int kclosest = 100 > connectNum ? 100 : connectNum;

     rdmp.m_pRoadmap->AddVertex(_start);
     rdmp.m_pRoadmap->AddVertex(_goal);
     vector<CfgPairType> kp;

     FindKClosestPairs(kp, env, dm, kclosest, _start, vertices, dmsetid);
     FindKClosestPairs(kp, env, dm, kclosest, _goal, vertices, dmsetid);
    
     for(int i=0; i<kp.size(); ++i) {
        if(lp->IsConnected(&rdmp,cd,dm,kp[i].first,kp[i].second, _lpsid,&sci)) {
	  rdmp.m_pRoadmap->AddEdge(kp[i].first, kp[i].second, sci.edge);
        }
     }	
  }

  while(IsSameCC(*(rdmp.m_pRoadmap), _start, _goal)) {
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
              //os << rdmp.roadmap.GetVID(rp[i].first) << " ";
	      os << rdmp.m_pRoadmap->GetVID(rp[i].first) << " ";
              os << rp[i].second << "  ";
              os << rp[i].first << "\n";
          }
	  WritePathConfigurations("mapnodes.path", _mapcfgs, env);
      #endif
      return true;
  }
  
  return false;
};


// This method removes all nodes that don't meet requirements in the given
// vector of cfgs.
// returns true if a node was removed, false if no nodes were removed
bool AdaptiveQuery::removeBadNodes(vector <Cfg>& cfgs, Environment *env,
				   CollisionDetection *_cd, SID _cdsetid) {
  bool removed_a_node = false;

  for(int i=0; i<cfgs.size(); ++i) {
    if (!queryReq.isNodeValid(cfgs[i], env, _cd, _cdsetid)) { //if node is bad, remove it
      rdmp.m_pRoadmap->DeleteVertex(cfgs[i]);
      removed_a_node = true;
    }
  }

//remove later
  CDInfo info;
  for(int i=0; i<cfgs.size(); ++i) {
    if(cfgs[i].isCollision(env, _cd, _cdsetid, info)) {
      rdmp.m_pRoadmap->DeleteVertex(cfgs[i]);
      removed_a_node = true;
    }
  }
//endremove
  
  return removed_a_node;
};


bool
AdaptiveQuery::
GetPathSegment(Cfg _c1, Cfg _c2, CollisionDetection *cd,
               LocalPlanners * lp,DistanceMetric * dm,WEIGHT _weight, LPInfo* _ci){

   // clear the potential old storage.
   _ci->path.erase(_ci->path.begin(),_ci->path.end());

   vector<pair<SID,vector<LP> > > sets = lp->planners.GetLPSets();

   int i,powf2,Found,FoundInBitPosition;
   for (i=0, powf2=2, Found=0; !Found && i<sets.size(); ++i, powf2 *= 2){
     Found = (powf2-1) & _weight.LP() ;
     FoundInBitPosition = i;
   }

   if (Found) {


     LPInfo info,info_rev;
        info.checkCollision=info_rev.checkCollision= _ci->checkCollision;
        info.savePath      =info_rev.savePath      = _ci->savePath;
        info.positionRes   =info_rev.positionRes   = _ci->positionRes;
        info.orientationRes=info_rev.orientationRes= _ci->orientationRes;
        info.cdsetid       =info_rev.cdsetid       = _ci->cdsetid;
        info.dmsetid       =info_rev.dmsetid       = _ci->dmsetid;

     while(FoundInBitPosition < lp->planners.GetElements().size() ) {
        LP Lp = lp->planners.GetLP(FoundInBitPosition);

        // FORWARD
        if ( lp->IsConnected(Lp.GetPlanner(), rdmp.GetEnvironment(),
            cd,dm,_c1, _c2, Lp, &info)) {
              _ci->path.insert(_ci->path.end(), info.path.begin(),info.path.end());
              info.path.erase(info.path.begin(),info.path.end());
              return true;

        // BACKWARD
        } else if ( lp->IsConnected(Lp.GetPlanner(), rdmp.GetEnvironment(),
        cd,dm,_c2, _c1, Lp, &info_rev) ){
              reverse(info_rev.path.begin(),info_rev.path.end());
              _ci->path.insert(_ci->path.end(),
                info_rev.path.begin(),info_rev.path.end());
              info_rev.path.erase(info_rev.path.begin(),info_rev.path.end());
              return true;

        } else {                                           // NEITHER!
              char *lpfcn_name = Lp.GetName();
              cout << "\n\n\t lpfcn: "<<lpfcn_name<<" FAILED!!! \n\n";
        }

	++FoundInBitPosition;

     }

   } else {

     // LKD: should have a method
     //         rdmp.lp->planners.IsPlanner(FoundInBitPosition)
     //      but this check'll do for now...
     cout << "\nERROR: _weight(" << _weight
          << ") is out of bounds (1,2^numLPs)"
          << "\n       where numLPs = "     << sets.size()
          << "\n";
   }
   return false;

}

