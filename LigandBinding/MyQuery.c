// $Id$
////////////////////////////////////////////////////
//
//  MyQuery.c
//
//  derived class of Query
//
/////////////////////////////////////////////////////


#include"MyQuery.h"
#include"BioPotentials.h"
#include"DistanceMetrics.h"
#include"Environment.h"


MyQuery::MyQuery() : Query() {};

MyQuery::MyQuery(Input *input, QueryCmds *Qinput, CollisionDetection* cd, 
		 DistanceMetric* dm, LocalPlanners* lp,ConnectMapNodes* cn) :
	 Query(input, Qinput, cd, dm, lp, cn) {};

MyQuery::~MyQuery() {};


// this is needed to make this method visible outside.
bool
MyQuery::PerformQuery(CollisionDetection* cd, ConnectMapNodes* cn, 
		      LocalPlanners* lp, DistanceMetric* dm) {

  return Query::PerformQuery(cd,cn,lp,dm);
}
/////////////////////////////////////////////////////////////////
//
// My version of performQuery method
//
/////////////////////////////////////////////////////////////////
typedef pair<Cfg, Cfg> CfgPairType;
typedef pair<CfgPairType, double> DIS_TYPE;
bool DIST_Compare(const DIST_TYPE &_cc1, const DIST_TYPE &_cc2) {
        return (_cc1.second < _cc2.second ) ;
};

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
//ofstream os1("rmsd.m");
//ofstream os2("euclid.m");
//ofstream os3("wed.m");
  for(i=0; i<kclosest; ++i)
        tmp.push_back(DIS_TYPE(CfgPairType(), MAX_DIST));
  for(i=0; i<vertices.size(); ++i) {
     double dist = dm->Distance(_env, c, vertices[i], dmsid);
//cout << "dist is " << dist << endl;
//os2 << dist << "\n";
//os1 << dm->Distance(_env, c, vertices[i], -1) << "\n";
//os3 << dm->Distance(_env, c, vertices[i], -2) << "\n";
     
     if(dist < tmp[kclosest-1].second ) {
        tmp[kclosest-1] = DIST_TYPE(CfgPairType(c, vertices[i]), dist);
        sort(tmp.begin(), tmp.end(), ptr_fun(DIST_Compare));
     }
  }
 
  for(i=0; i<kclosest; ++i)
     kp.push_back(tmp[i].first);
}




bool MyQuery::PerformQuery(Cfg _start, Cfg _goal, CollisionDetection *cd,
     ConnectMapNodes*cn, LocalPlanners *lp, DistanceMetric* dm, SID _lpsid, vector<Cfg>* _path) {

  LPInfo sci, ci;
  sci.positionRes =ci.positionRes = rdmp.GetEnvironment()->GetPositionRes();
  sci.orientationRes =ci.orientationRes = rdmp.GetEnvironment()->GetOrientationRes();
  sci.checkCollision = ci.checkCollision = true;
  sci.cdsetid = ci.cdsetid = cdsetid;
  sci.dmsetid = ci.dmsetid = dmsetid;
  sci.savePath = false;
  ci.savePath = true;

  Environment *env = rdmp.GetEnvironment();
  vector<Cfg> vertices = rdmp.roadmap.GetVerticesData();
  int connectNum = vertices.size()/100;
  const int kclosest = 20 > connectNum ? 20 : connectNum;
  rdmp.roadmap.AddVertex(_start);
  rdmp.roadmap.AddVertex(_goal);
  vector<CfgPairType> kp;
  FindKClosestPairs(kp, env, dm, kclosest, _start, vertices, dmsetid);
  FindKClosestPairs(kp, env, dm, kclosest, _goal, vertices, dmsetid);
  for(int i=0; i<kp.size(); ++i) {
if(i<kp.size()/2) 
    cout << " trying connecting start to roadmap ... " << endl;
else
    cout << " trying connecting goal to roadmap ... " << endl;
     if(lp->IsConnected(&rdmp,cd,dm,kp[i].first,kp[i].second, _lpsid,&sci)) {
        rdmp.roadmap.AddEdge(kp[i].first, kp[i].second, sci.edge);
cout << "connection succeeded " << endl;
}
  }

  while(rdmp.roadmap.IsSameCC(_start, _goal)) {
      cout << endl << "start and goal seems connected!!" << endl;
      vector< pair<Cfg,WEIGHT> > rp = rdmp.roadmap.FindPathDijkstra(_start, _goal);

      int i;
      // added by Guang on 04/11/99: add Start Cfg to path.
      _path->push_back(_start);
      bool failedToRecreatePath = false;
      for (i=0; i<rp.size()-1; i++) {
          if ( !(GetPathSegment(rp[i].first,rp[i+1].first,cd,lp,dm,rp[i].second,&ci) )) {
               cout << endl << "In PerformQuery: can't recreate path" << endl;
	       rdmp.roadmap.DeleteEdge(rp[i].first,rp[i+1].first);
	       failedToRecreatePath = true;
	       _path->erase(_path->begin(), _path->end());
	       break;
          } else {
               _path->insert(_path->end(), ci.path.begin(),ci.path.end());
               ci.path.erase(ci.path.begin(),ci.path.end());
          }
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
              os << rdmp.roadmap.GetVID(rp[i].first) << " ";
              os << rp[i].second << "  ";
              os << rp[i].first << "\n";
          }
          WritePathConfigurations("mapnodes.path", _mapcfgs, env);
      #endif

      // this part outputs the potential of cfgs along the path.
      ofstream osp("potential.dat");
      for(i=0; i<_path->size(); ++i)
         osp << BioPotentials::GetPotential((*_path)[i], env) << "\n";
      cout << "last node " << (*_path)[_path->size()-1] << endl;
      return true;
  }
  return false;


};


