////////////////////////////////////////////////////
//
//  PriorityQuery.c
//
//  derived class of PriorityQuery
//
/////////////////////////////////////////////////////


#include"PriorityQuery.h"
#include"GraphAlgo.h"
#include"PriorityLocalPlanners.h"
#include"PriorityWeight.h"


class MyEdgeInfo {
public:
  MyEdgeInfo() {}
  MyEdgeInfo(VID _v1, VID _v2, pair<WEIGHT,WEIGHT> _wts) : v1(_v1), v2(_v2), edgewts(_wts) {}

  VID v1;
  VID v2;
  ///_wts.first is weight for _v1->_v2 and _wts.second is *weight for _v2->_v1
  pair<WEIGHT,WEIGHT> edgewts;
  
  bool operator< (const MyEdgeInfo& tmp) const {
    MyEdgeInfo _x = *this;
    MyEdgeInfo _y = tmp;
    double x = _x.edgewts.first.Weight();
    double y = _y.edgewts.first.Weight();
    return (x < y);
  }

};


PriorityQuery::PriorityQuery(Input *input, MyQueryCmds *Qinput, CollisionDetection* cd, 
		 DistanceMetric* dm, PriorityLocalPlanners* lp,ConnectMapNodes* cn) :
	 AdaptiveQuery(input, Qinput, cd, dm, lp, cn) {}


PriorityQuery::~PriorityQuery() {}


// this is needed to make this method visible outside.
bool
PriorityQuery::PerformQuery(CollisionDetection* cd, ConnectMapNodes* cn, 
		      PriorityLocalPlanners* lp, DistanceMetric* dm) {
  if(query.size() == 1) { // only goal is provided.
     //vector<Cfg> vertices = rdmp.roadmap.GetVerticesData();
     vector<Cfg> vertices;
     rdmp.m_pRoadmap->GetVerticesData(vertices);

     // choose a random cfg as start...
     Cfg &start = vertices[(int)vertices.size()*drand48()];
     query.insert(query.begin(), start);
  }

  for (int i=0; i < query.size()-1; i++ ) {
     cout << "\nquery is ...     ";
                                   query[i].Write(cout);
     cout << "\n                 ";
                                   query[i+1].Write(cout);
     cout << "\nworking  ...     "
          << endl;

     if ( !PerformQuery(query[i],query[i+1],cd,cn,lp,dm,lpsetid,&path) ) {
        cout << endl << "In PerformQuery(): didn't connect";
        return false;
     } 
  }

  return true;
}


struct less_edge_info : public binary_function<MyEdgeInfo,MyEdgeInfo,bool> {
  bool operator()(MyEdgeInfo x, MyEdgeInfo y) {
    return x.edgewts.first.Weight() < y.edgewts.first.Weight();
  }
};


bool 
PriorityQuery::PerformQuery(Cfg _start, Cfg _goal, CollisionDetection *cd,
     ConnectMapNodes*cn, PriorityLocalPlanners *lp, DistanceMetric* dm, SID _lpsid, vector<Cfg>* _path) {

  Environment *env = rdmp.GetEnvironment();

  PriorityWeightFactory* fact = new PriorityWeightFactory();
  WeightObject::SetWeightFactory(fact);

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
  sci.positionRes    = ci.positionRes    = rdmp.GetEnvironment()->GetPositionRes();
  sci.orientationRes = ci.orientationRes = rdmp.GetEnvironment()->GetOrientationRes();
  sci.checkCollision = ci.checkCollision = true;
  sci.cdsetid        = ci.cdsetid        = cdsetid;
  sci.dmsetid        = ci.dmsetid        = dmsetid;
  ((PriorityWeight*)sci.edge.first.GetIWeight())->Level()  = ((PriorityWeight*)ci.edge.first.GetIWeight())->Level()  = 0;
  ((PriorityWeight*)sci.edge.second.GetIWeight())->Level() = ((PriorityWeight*)ci.edge.second.GetIWeight())->Level() = 0;
  sci.savePath = false;
  ci.savePath = true;

  //add priority weights to edges in roadmap:
  vector<pair<pair<VID,VID>,WEIGHT> > edges;
  rdmp.m_pRoadmap->GetEdges(edges);
  for (int i=0; i<edges.size(); i++) {
    rdmp.m_pRoadmap->DeleteEdge(edges[i].first.first, edges[i].first.second);
    if(lp->IsConnected(&rdmp, cd, dm, rdmp.m_pRoadmap->GetData(edges[i].first.first),
		       rdmp.m_pRoadmap->GetData(edges[i].first.second), _lpsid, &sci)) {
      rdmp.m_pRoadmap->AddEdge(edges[i].first.first, edges[i].first.second, sci.edge);
    }
  }

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
      cout << "\nstart and goal seem connected!" << endl;
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
	//check path for collision:
	priority_queue<MyEdgeInfo> Q;
	MyEdgeInfo tmpEdge;
	for(int i=0; i<rp.size()-1; i++) {
	  tmpEdge.v1 = rdmp.m_pRoadmap->GetVID(rp[i].first);
	  tmpEdge.v2 = rdmp.m_pRoadmap->GetVID(rp[i+1].first);
	  tmpEdge.edgewts.first = rdmp.m_pRoadmap->GetEdgeWeight(rp[i].first, rp[i+1].first);
	  tmpEdge.edgewts.second = rdmp.m_pRoadmap->GetEdgeWeight(rp[i+1].first, rp[i].first);
	  Q.push(tmpEdge);
	}
	bool path_collision_free = false;
	MyEdgeInfo worst_edge;

	while (!failedToRecreatePath) {
	  worst_edge = Q.top();
	  Q.pop();
	  if(worst_edge.edgewts.first.Weight() == 0) { //probability == 1, path collision free
	    path_collision_free = true;
	    break;
	  } else { //path not collision free
	    //increase level, check again
	    int level = 1;
	    level += ((PriorityWeight*)worst_edge.edgewts.first.GetIWeight())->Level();
	    ((PriorityWeight*)worst_edge.edgewts.first.GetIWeight())->Level() = level;
	    ((PriorityWeight*)ci.edge.first.GetIWeight())->Level() = level;
	    
	    level = 1;
	    level += ((PriorityWeight*)worst_edge.edgewts.second.GetIWeight())->Level();
	    ((PriorityWeight*)worst_edge.edgewts.second.GetIWeight())->Level() = level;
	    ((PriorityWeight*)ci.edge.second.GetIWeight())->Level() = level;
	    
	    if ( !(GetPathSegment(rdmp.m_pRoadmap->GetData(worst_edge.v1),
				  rdmp.m_pRoadmap->GetData(worst_edge.v2),
				  cd, lp, dm, worst_edge.edgewts.first, &ci)) ) {
	      cout << endl << "In PerformQuery: can't recreate path!" << endl;
	      rdmp.m_pRoadmap->DeleteEdge(worst_edge.v1, worst_edge.v2);
	      failedToRecreatePath = true;
	      _path->erase(_path->begin(), _path->end());
	    } else {
	      //put back in queue with new weight
	      worst_edge.edgewts.first.Weight() = ci.edge.first.Weight();
	      worst_edge.edgewts.second.Weight() = ci.edge.second.Weight();
	      Q.push(worst_edge);

	      //update weight in roadmap
	      rdmp.m_pRoadmap->DeleteEdge(worst_edge.v1, worst_edge.v2);
	      rdmp.m_pRoadmap->AddEdge(worst_edge.v1, worst_edge.v2, worst_edge.edgewts);
	    }
	  }
	}

	//check query requirements:
	if(path_collision_free) {
	  for (i=0; i<rp.size()-1; i++) {
	    ci.path.erase(ci.path.begin(), ci.path.end()); //clear potential old storage
	    ci.checkCollision = 0;
	    lp->IsConnected(env, cd, dm, rp[i].first, rp[i+1].first, _lpsid, &ci);
	    ci.checkCollision = 1;
	    if(!queryReq.isEdgeValid(ci.path, env, cd, cdsetid)) {
	      cout << endl << "In PerformQuery: can't meet query requirements!" << endl;
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
}


bool
PriorityQuery::GetPathSegment(Cfg _c1, Cfg _c2, CollisionDetection *cd,
	    PriorityLocalPlanners * lp,DistanceMetric * dm,WEIGHT _weight, LPInfo* _ci) {

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
        info.checkCollision = info_rev.checkCollision = _ci->checkCollision;
        info.savePath       = info_rev.savePath       = _ci->savePath;
        info.positionRes    = info_rev.positionRes    = _ci->positionRes;
        info.orientationRes = info_rev.orientationRes = _ci->orientationRes;
        info.cdsetid        = info_rev.cdsetid        = _ci->cdsetid;
        info.dmsetid        = info_rev.dmsetid        = _ci->dmsetid;
	info.edge.first     = info_rev.edge.first     = _ci->edge.first;
	info.edge.second    = info_rev.edge.second    = _ci->edge.second;

     while(FoundInBitPosition < lp->planners.GetElements().size() ) {
        LP Lp = lp->planners.GetLP(FoundInBitPosition);

        // FORWARD
        if ( lp->IsConnected(Lp.GetPlanner(), rdmp.GetEnvironment(),
			     cd,dm,_c1, _c2, Lp, &info)) {
              _ci->path.insert(_ci->path.end(), info.path.begin(),info.path.end());
              info.path.erase(info.path.begin(),info.path.end());
	      _ci->edge.first = info.edge.first;
	      _ci->edge.second = info.edge.second;
              return true;

        // BACKWARD
        } else if ( lp->IsConnected(Lp.GetPlanner(), rdmp.GetEnvironment(),
				    cd,dm,_c2, _c1, Lp, &info_rev) ) {
              reverse(info_rev.path.begin(),info_rev.path.end());
              _ci->path.insert(_ci->path.end(),
                info_rev.path.begin(),info_rev.path.end());
              info_rev.path.erase(info_rev.path.begin(),info_rev.path.end());
	      _ci->edge.first = info_rev.edge.first;
	      _ci->edge.second = info_rev.edge.second;
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
