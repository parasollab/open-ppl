/////////////////////////////////////////////////////////////////////
//
//   ConnectCCs.cpp
//
//   General Description
//      This class contains data and methods to try to connect different
//	connected components of a roadmap using different approaches such 
//	as RayTracing and RRT
//  Created
//      09/06/02 Marco Morales
/////////////////////////////////////////////////////////////////////

#include "Defines.h"
#include <fstream.h>
#include <string.h>

#include "RayTracer.h"
#include "ConnectCCs.h"
#include "Environment.h"
#include "GraphAlgo.h"
/////////////////////////////////////////////////////////////////////
//
//  METHODS for class ConnectCCs
//
/////////////////////////////////////////////////////////////////////
//==================================================================
// ConnectCCs class Methods: Constructors and Destructor
//==================================================================

ConnectCCs::
ConnectCCs() 
{
  //    outputPathFile=NULL;
}

ConnectCCs::
ConnectCCs(Input *input,ConnectCCsCmds *connect_CCs_input,
    CollisionDetection *cd, DistanceMetric *dm, LocalPlanners *lp,
        ConnectMapNodes    *cn)
{
    rdmp.InitRoadmap(input,cd,dm,lp,connect_CCs_input->mapFile.GetValue() );

    connect_cc_method = string(connect_CCs_input->connect_cc_method.GetValue());
    
//      ReadConnectCCs( connect_CCs_input->queryFile.GetValue() );

    initDefaultSetIDs(cn);

//      outputPathFile = new char[strlen(connect_CCs_input->pathFile.GetValue())+1];
//      strcpy(outputPathFile,connect_CCs_input->pathFile.GetValue());
}

ConnectCCs::
~ConnectCCs()
{
    //fre file name memory
//      if( outputPathFile!=NULL )
//          delete [] outputPathFile;
//      outputPathFile=NULL;
}

//==================================================================
// ConnectCCs class Methods: Other Methods
//==================================================================

void 
ConnectCCs::
initDefaultSetIDs(ConnectMapNodes    *cn) {
  // allow user to specify new sets for connection start/goal
  // default for QUERY is most powerful set
  if (cn->cnInfo.lpsetid < LP_USER1){
        lpsetid = SL_R5_AD69;
  } else {
        lpsetid = cn->cnInfo.lpsetid;
  }

  // user specifed, same defaults as for ROADMAP construction
  dmsetid = cn->cnInfo.dmsetid;
  cdsetid = cn->cnInfo.cdsetid;
}


bool 
ConnectCCs::
PerformConnectCCs(CollisionDetection *cd, ConnectMapNodes *cn, LocalPlanners * lp,DistanceMetric * dm) 
{

  //here is where we will call the proper method from (RRT or RayTracer
  if (connect_cc_method == string("RRT")) {
    cout << "using RRT to attempt to connect CCs" << endl;
    //The call to RRT from ConnectMapNodes goes here
    (*cn).ConnectNodes(&rdmp,cd,lp,dm, (*cn).cnInfo.cnsetid, (*cn).cnInfo);
  }
  else if (connect_cc_method == string("RayTracer")) {
    
    bool path_found=false;
    cout << "usint RayTracer to attempt to connect CCs" << endl;
    //The call to RayTracer from ConnectMapNodes goes here
      Environment * environment = rdmp.GetEnvironment();
      CDInfo info;
      Cfg source = Cfg::GetFreeRandomCfg(environment, cd, cdsetid,info);
      Cfg target = Cfg::GetFreeRandomCfg(environment, cd, cdsetid,info);
      RayTracer tracer(environment, source, target);
      tracer.setDirection(RT_TARGET_ORIENTED);
  
      while (!path_found && !tracer.exhausted()) {
         //Trace the ray
         cout<< "Trying new direction for ray"<<endl;
         path_found=tracer.trace(cd, cdsetid, info, dm, dmsetid);
         tracer.newDirection();
         }
    
  }
  else {
    cout << "Unknown choice, doing nothing" << endl;
    return false;
  }


//    for (int i=0; i < query.size()-1; i++ ){
//       cout << "\nquery is ...     ";
//                                     query[i].Write(cout);
//       cout << "\n                 ";
//                                     query[i+1].Write(cout);
//       cout << "\nworking  ...     "
//            << endl;

//       if ( !PerformConnectCCs(query[i],query[i+1],cd,cn,lp,dm,lpsetid,&path) ) {
//          cout << endl << "In PerformConnectCCs(): didn't connect";
//          return false;
//       } 
//    }

  return true;
};

bool 
ConnectCCs::
PerformConnectCCs(Cfg _start, Cfg _goal, CollisionDetection *cd,
         ConnectMapNodes *cn, LocalPlanners * lp,DistanceMetric * dm,SID _lpsid, vector<Cfg>* _path) {

  LPInfo sci, gci;   // connection info for start, goal nodes
  sci.positionRes =gci.positionRes = rdmp.GetEnvironment()->GetPositionRes();
  sci.orientationRes =gci.orientationRes = rdmp.GetEnvironment()->GetOrientationRes(); 
  sci.checkCollision = gci.checkCollision = true;
  sci.savePath = gci.savePath = true;
  sci.cdsetid = gci.cdsetid = cdsetid;
  sci.dmsetid = gci.dmsetid = dmsetid;


  vector< pair<int,VID> > ccs;
  GetCCStats(*(rdmp.m_pRoadmap),ccs);  

  bool connected = false;
  int  i, thiscc = 0;
  VID scvid, gcvid;

  while ( !connected && thiscc < ccs.size() ) {

    //get cc
    Cfg t=rdmp.m_pRoadmap->GetData(ccs[thiscc].second);
    //-gti vector<Cfg> cc = rdmp.m_pRoadmap->GetCC(t);
    vector<Cfg> cc;
    GetCC(*(rdmp.m_pRoadmap) , t,cc);
    //connect start and goal to cc
    if ( CanConnectToCC(_start,cd,cn,lp,dm,cc,_lpsid,&scvid,&sci) && 
         CanConnectToCC(_goal, cd,cn,lp,dm,cc,_lpsid,&gcvid,&gci) ) {

    cout << endl << "Start("
            << scvid
            << ") & Goal("
            << gcvid
            << ") Connected to same CC["
                    << thiscc+1
                    << "]!" 
            << endl;

       connected = true;

       // Add to Path: [ start cfg ]
       _path->push_back(_start);

       // Add to Path "tick" cfg's: [ start->rdmp ]
       _path->insert(_path->end(),                   
        sci.path.begin(),sci.path.end());

       LPInfo ci;
         ci.positionRes = rdmp.GetEnvironment()->GetPositionRes();
         ci.orientationRes = rdmp.GetEnvironment()->GetOrientationRes();
         ci.checkCollision = true;
         ci.savePath = true;
         ci.cdsetid = cdsetid;
         ci.dmsetid = dmsetid;

       if ( scvid != gcvid ) {

          vector< pair<Cfg,WEIGHT> > rp;
	  //-gti rp = rdmp.m_pRoadmap->FindPathDijkstra(scvid,gcvid);
	  FindPathDijkstra(*(rdmp.m_pRoadmap),scvid,gcvid,rp);
	  //cout<<rp.size()<<endl;

      #if INTERMEDIATE_FILES
          //-----------------------------------------------------
          // Print out all start, all graph nodes, goal
          // ie, *NO* "ticks" from local planners
          //-----------------------------------------------------
          vector<Cfg> _mapcfgs;
          WritePathConfigurations("mapnodes.path", 
                                         _mapcfgs, 
                                         rdmp.GetEnvironment());
      #endif

          for (i=0; i<rp.size()-1; i++) {
            if ( !(GetPathSegment(rp[i].first,rp[i+1].first,cd,lp,dm,rp[i].second,&ci) )) {
                  cout << endl << "In PerformConnectCCs: can't recreate path" << endl;
            } else {
                  // Add to Path rdmp cfg's & "tick"s: [ rdmp.rdmp ]
                  _path->insert(_path->end(),
            ci.path.begin(),ci.path.end());
            }

          } // for i

       } //if ( scvid != gcvid )
           
       // Add to Path "tick" cfg's: [ rdmp.goal ]
       reverse(gci.path.begin(),gci.path.end());
       _path->insert(_path->end(),
        gci.path.begin(),gci.path.end());

       // Add to Path: [ goal cfg ]
       _path->push_back(_goal);

    }
    thiscc++;  // try next connected component
  }

  if(connected) {
     // add start and goal to the roadmap
     // to extend current roadmap if successful query.
     rdmp.m_pRoadmap->AddVertex(_start);
     rdmp.m_pRoadmap->AddVertex(_goal);
     rdmp.m_pRoadmap->AddEdge(rdmp.m_pRoadmap->GetVID(_start), scvid, sci.edge);
     rdmp.m_pRoadmap->AddEdge(rdmp.m_pRoadmap->GetVID(_goal), gcvid, gci.edge);
  }

  return connected;
};


bool
ConnectCCs::
CanConnectToCC(Cfg _cfg, CollisionDetection *cd,
        ConnectMapNodes *cn, LocalPlanners *lp, DistanceMetric *dm,
        vector<Cfg> _cc, SID _lpsid, VID *_vid,LPInfo *_ci) {

   // erase previous connection attempt. 
   // (when start or goal connection is sucessful.
   _ci->path.erase( _ci->path.begin(),_ci->path.end() );

   // sort the cfgs in _cc by distance from _cfg
   cn->SortByDistFromCfg(rdmp.GetEnvironment(),dm,cn->cnInfo,_cfg,_cc);

   // try to connect _cfg to (closest) config in _cc 
   // (now try all, later only k closest)
   for (int i=0; i < _cc.size(); i++ ) {
      if ( lp->IsConnected(&rdmp,cd, dm,_cfg,_cc[i],_lpsid,_ci) ) {
          *_vid = rdmp.m_pRoadmap->GetVID(_cc[i]);
          return true;
      } else {
          // clear out previous (ie, "old") connection attempt
          _ci->path.erase( _ci->path.begin(),_ci->path.end() );
      }

   }
   *_vid = INVALID_VID;
   return false;
}

bool
ConnectCCs::
GetPathSegment(Cfg _c1, Cfg _c2, CollisionDetection *cd,
               LocalPlanners * lp,DistanceMetric * dm,WEIGHT _weight, LPInfo* _ci){
   // clear possible old storage.
   _ci->path.erase(_ci->path.begin(), _ci->path.end());

   vector<pair<SID,vector<LP> > > sets = lp->planners.GetLPSets();

   int i,powf2,Found,FoundInBitPosition;
   for (i=0, powf2=2, Found=0; !Found && i<sets.size(); ++i, powf2 *= 2){
     Found = (powf2-1) & _weight.LP() ;
     FoundInBitPosition = i;
   }

   if (Found) {

     LP Lp = lp->planners.GetLP(FoundInBitPosition);

     LPInfo info,info_rev;
        info.checkCollision=info_rev.checkCollision= _ci->checkCollision;
        info.savePath      =info_rev.savePath      = _ci->savePath;
    info.positionRes   =info_rev.positionRes   = _ci->positionRes;
    info.orientationRes=info_rev.orientationRes= _ci->orientationRes;
    info.cdsetid       =info_rev.cdsetid       = _ci->cdsetid;
    info.dmsetid       =info_rev.dmsetid       = _ci->dmsetid;

     // FORWARD
     if ( lp->IsConnected(Lp.GetPlanner(), rdmp.GetEnvironment(),cd,dm,_c1, _c2, Lp, &info) ) {
    _ci->path.insert(_ci->path.end(),
        info.path.begin(),info.path.end());
    info.path.erase(info.path.begin(),info.path.end());
        return true;

     // BACKWARD
     } else if ( lp->IsConnected(Lp.GetPlanner(), rdmp.GetEnvironment(),cd,dm,_c2, _c1, Lp, &info_rev) ){
    reverse(info_rev.path.begin(),info_rev.path.end());
    _ci->path.insert(_ci->path.end(),
        info_rev.path.begin(),info_rev.path.end());
    info_rev.path.erase(info_rev.path.begin(),info_rev.path.end());
        return true;

     } else {                       // NEITHER!
    char *lpfcn_name = Lp.GetName();
        cout << "\n\n\t lpfcn: "<<lpfcn_name<<" FAILED!!! \n\n";
     }

   } else { ///not found

     // LKD: should have a method 
     //     rdmp.lp->planners.IsPlanner(FoundInBitPosition)
     //      but this check'll do for now...
     cout << "\nERROR: _weight(" << _weight 
          << ") is out of bounds (1,2^numLPs)" 
          << "\n       where numLPs = "     << sets.size() 
          << "\n";
   }
   return false;

}

//===================================================================
// ConnectCCs class Methods: Display, Input, Output
//===================================================================

//  void 
//  ConnectCCs::
//  ReadConnectCCs(const char* _filename) {


//     Cfg tempCfg;

//     ifstream  myifstream(_filename);
//     if (!myifstream) {
//           cout << endl << "In ReadConnectCCs: can't open infile: " << _filename ;
//           return;
//     }


//     while (1) {
//        tempCfg.Read(myifstream);
//        if(!myifstream) break;
//        query.push_back(tempCfg);
//     }

//     myifstream.close();
//  };

//  void 
//  ConnectCCs::
//  WritePath() {
//     WritePath( outputPathFile );
//  }

//  void 
//  ConnectCCs::
//  WritePath(char* _filename ) {
//     WritePathConfigurations(_filename, path, rdmp.GetEnvironment());
//  }

