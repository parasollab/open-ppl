// $Id$

////////////////////////////////////////////////////////////////////////////////////////////
/**@file Query.h
  *
  * This is the main OBPRM class which contains data and methods
  * to manipulate the environment with specified moving bodies
  * (ie, robot(s)) and the corresponding roadmap.
  *
  * This file contains the prototype declarations for the class. 
  * Definitions are in the file "Roadmap.cpp".
  *
  * @date 08/18/98
  * @author Nancy Amato
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Query_h
#define Query_h

////////////////////////////////////////////////////////////////////////////////////////////
#include "Roadmap.h"     
#include "util.h"

#include "ConnectMap.h"
class QueryCmds;

template <class CFG, class WEIGHT>
class Query {

public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor */
    //@{

        /**
         *Default Constructor. Set Every thing to NULL.
         *This is useful when client needs to init data member by themself
         *for example instead of read map from file, client could set roadmap
         *directly.
         */
        Query();

        /**
          *Preferred*, fills input & inits.
          *Initialize roadmap (#rdmp). 
          *Read roadmap data from file.
          *@see Roadmap::InitRoadmap
          */
        Query
        (Input *, QueryCmds*, CollisionDetection*, DistanceMetric*, LocalPlanners<CFG,WEIGHT>*, ConnectMap<CFG, WEIGHT>*);  

        ///Destructor. Free memory.
        ~Query();

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Helpe Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Help methods*/
    //@{

        /**Try to connect given Cfg to given connected component.
          *This method tries to connect every Cfgs in given 
          *connected component to this given Cfg.
          *(Cfgs in cc are sorted before connecting)
          *
          *@param _cfg connect this Cfg to connected component.
          *@param _cc A list of Cfgs in same connected component.
          *@param _vid if Connection is made then this parameter will
          *be set as the id of vertex (Cfg) in the roadmap graph. If
          *no connection is made, this parameter will be set as 
          *INVALID_VID.
          *@param _ci Connected path will be put in here.
          *
          *@see ConnectMap::SortByDistFromCfg
          */
        virtual 
	  bool CanConnectToCC(CFG _cfg, Stat_Class& Stats,
			      CollisionDetection *cd,
			      ConnectMap<CFG, WEIGHT> *cn, 
			      LocalPlanners<CFG,WEIGHT> *lp,
			      DistanceMetric *dm,
			      vector<CFG> _cc, 
			      VID *_vid,
			      LPOutput<CFG,WEIGHT> *_ci);

	virtual
	  bool CanRecreatePath(vector<pair<CFG,WEIGHT> >& attemptedPath,
			       Stat_Class& Stats, CollisionDetection* cd, 
			       LocalPlanners<CFG,WEIGHT>* lp, 
			       DistanceMetric* dm, 
			       vector<CFG>& recreatedPath);
    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Query Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Query methods*/
    //@{

        /**Query path(s) for Cfgs in #query.
          *This method will read Cfg list, #query, and make connection for all Cfgs
          *inside this list.
          *Messages will be output to standard ouput to tell client
          *what current status is.
          *
          *@return true if path is found. Otherwise false will be returned.
          *@note path will be stored in #path.
          */
        virtual 
	bool PerformQuery
        (Stat_Class& Stats, CollisionDetection *cd, ConnectMap<CFG, WEIGHT> *cn, LocalPlanners<CFG,WEIGHT> * lp,DistanceMetric * dm);

        /**Query path for two given Cfgs.
          *Algorithm:
          * -#  For every connected component, cc, in roadmap.
          *     -# if start and goal could connect to this cc
          *         -# insert path from start to connection point, cc_start, in cc 
          *            into return_path.
          *         -# search path from cc_start to cc_end and insert it to return_path
          *         -# insert path from connection point, cc_end, in cc 
          *            to goal into return_path.
          *         -# expand cc by adding start and goal as new vertices and paths
          *            (start->cc_start) and (cc_end->goal) as new edges.
          *     -# else
          *         cc = next connected component in roadmap
          *     -# end if
          * -# end for
          * -# return false
          *
          *@param _start start configuration of Robot.
          *@param _goal goal configuration for Robot.
          *@param _lpsid Which local planer will used to connect start and goal to
          *connected components.
          *@path _path Result.
          *
          *@return true if path is found. Otherwise false will be returned.
          *@sa GetPathSegment
          */
        virtual 
	bool PerformQuery(CFG _start, 
			  CFG _goal,
			  Stat_Class& Stats,
			  CollisionDetection*,
			  ConnectMap<CFG, WEIGHT>*, 
			  LocalPlanners<CFG,WEIGHT>*, 
			  DistanceMetric*, 
			  vector<CFG>* _path);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  I/O (Display, Input, Output)
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O*/
    //@{

        ///Read query Cfgs from given file.
        virtual 
	void ReadQuery(const char* _filename);

        ///Write path data to #outputPathFile.
        virtual 
	void WritePath();

        ///Write path data to given file.
        virtual 
	void WritePath(char* _filename);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    Roadmap<CFG, WEIGHT> rdmp;

    vector<CFG> query;                  ///< start,[intermediate nodes],goal 
    vector<CFG> path;                   ///< output paths

    char *outputPathFile;               ///< File name for path data

};


#include "Environment.h"
#include "GraphAlgo.h"
#include "QueryCmds.h"
/////////////////////////////////////////////////////////////////////
//
//  METHODS for class Query
//
/////////////////////////////////////////////////////////////////////
//==================================================================
// Query class Methods: Constructors and Destructor
//==================================================================

template <class CFG, class WEIGHT>
Query<CFG, WEIGHT>::
Query() 
{
  outputPathFile=NULL;
}

template <class CFG, class WEIGHT>
Query<CFG, WEIGHT>::
Query(Input* input, QueryCmds* Qinput,
      CollisionDetection* cd, DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp,
      ConnectMap<CFG, WEIGHT>* cn)
{
    rdmp.InitRoadmap(input,cd,dm,lp,Qinput->mapFile.GetValue() );
    
    ReadQuery( Qinput->queryFile.GetValue() );
    
    outputPathFile = new char[strlen(Qinput->pathFile.GetValue())+1];
    strcpy(outputPathFile,Qinput->pathFile.GetValue());
}

template <class CFG, class WEIGHT>
Query<CFG, WEIGHT>::
~Query()
{
  //fre file name memory
  if( outputPathFile!=NULL )
    delete [] outputPathFile;
  outputPathFile=NULL;
}

//==================================================================
// Query class Methods: Other Methods
//==================================================================

template <class CFG, class WEIGHT>
bool 
Query<CFG, WEIGHT>::
PerformQuery(Stat_Class& Stats, CollisionDetection* cd, ConnectMap<CFG, WEIGHT>* cn, LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric* dm) 
{
  for(vector<CFG>::iterator Q = query.begin(); (Q+1) != query.end(); ++Q) {
    cout << "\nquery is ...     ";
    Q->Write(cout);
    cout << "\n                 ";
    (Q+1)->Write(cout);
    cout << "\nworking  ...     "
	 << endl;
    
    if ( !PerformQuery(*Q,*(Q+1),Stats,cd,cn,lp,dm,&path) ) {
      cout << endl << "In PerformQuery(): didn't connect";
      return false;
    } 
  }
  
  return true;
};

template <class CFG, class WEIGHT>
bool 
Query<CFG, WEIGHT>::
PerformQuery(CFG _start, CFG _goal, Stat_Class& Stats, CollisionDetection* cd,
	     ConnectMap<CFG, WEIGHT>* cn, LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric* dm, vector<CFG>* _path) {

  LPOutput<CFG,WEIGHT> sci, gci;   // connection info for start, goal nodes
  VID scvid, gcvid;

  vector<CFG> cc; 
  vector< pair<int,VID> > ccs;
  GetCCStats(*(rdmp.m_pRoadmap),ccs);  
  
  VID svid;
  if(rdmp.m_pRoadmap->IsVertex(_start))
    svid = rdmp.m_pRoadmap->GetVID(_start);
  else
    svid = rdmp.m_pRoadmap->AddVertex(_start);
  VID gvid;
  if(rdmp.m_pRoadmap->IsVertex(_goal))
    gvid = rdmp.m_pRoadmap->GetVID(_goal);
  else
    gvid = rdmp.m_pRoadmap->AddVertex(_goal);

  bool connected = false;
  vector<pair<int,VID> >::const_iterator CC, ccsBegin = ccs.begin();
  for(CC = ccs.begin(); CC != ccs.end(); ++CC) {
    //get cc data
    CFG cc_cfg;
    cc_cfg.equals(rdmp.m_pRoadmap->GetData(CC->second));
    cc.clear();
    GetCC(*(rdmp.m_pRoadmap), cc_cfg, cc);

    //attempt to connect start and goal to cc
    cout << "connecting start to CC[" << distance(ccsBegin,CC)+1 << "]" << endl;
    CanConnectToCC(_start, Stats,cd,cn,lp,dm, cc,&scvid,&sci);
    cout << "connecting goal to CC[" << distance(ccsBegin,CC)+1 << "]" << endl;
    CanConnectToCC(_goal,  Stats,cd,cn,lp,dm, cc,&gcvid,&gci);

    connected = false;
    while(IsSameCC(*(rdmp.m_pRoadmap), svid, gvid)) {
      cout << "\nStart(" << svid
	   << ") and Goal(" << gvid
	   << ") seem connected to same CC[" << distance(ccsBegin, CC)+1 
	   << "]!" << endl;
      
      //get DSSP path
      vector<pair<CFG,WEIGHT> > rp;
      FindPathDijkstra(*(rdmp.m_pRoadmap), svid, gvid, rp);
    
      //attempt to recreate path
      vector<CFG> recreatedPath;
      if(CanRecreatePath(rp, Stats, cd, lp, dm, recreatedPath)) {
	connected = true;

	//add start
	_path->push_back(_start);
	//add recreated path
	_path->insert(_path->end(), 
		      recreatedPath.begin(), recreatedPath.end());

	break;
      } else
        cout << endl << "Failed to recreate path\n";
    }
    if(connected) {
#if INTERMEDIATE_FILES
      //Print out all start, all graph nodes, goal
      //ie, *NO* "ticks" from local planners
      vector<CFG> _mapcfgs;
      for(vector<pair<CFG,WEIGHT> >::iterator I = rp.begin(); I != rp.end() ++I)
        _mapcfgs.push_back(I->first);
      WritePathConfigurations("mapnodes.path", _mapcfgs, rdmp.GetEnvironment());
#endif
      break;    
    }
  }
  
  return connected;
};

template <class CFG, class WEIGHT>
bool
Query<CFG, WEIGHT>::
CanConnectToCC(CFG _cfg, Stat_Class& Stats, CollisionDetection* cd,
	       ConnectMap<CFG, WEIGHT>* cn, LocalPlanners<CFG,WEIGHT>* lp, 
	       DistanceMetric* dm, vector<CFG> _cc, 
	       VID* _vid, LPOutput<CFG,WEIGHT>*_ci) {
   // erase previous connection attempt. 
   _ci->path.erase(_ci->path.begin(),_ci->path.end() );

   // sort the cfgs in _cc by distance from _cfg
   vector<ConnectionMethod<CFG,WEIGHT>*> selected = cn->GetDefault();
   typename vector<ConnectionMethod<CFG,WEIGHT>*>::iterator first_method = selected.begin();
   (*first_method)->SortByDistFromCfg(rdmp.GetEnvironment(),dm,_cfg,_cc);

   // try to connect _cfg to (closest) config in _cc 
   // (now try all, later only k closest)
   for (vector<CFG>::iterator I = _cc.begin(); I != _cc.end(); ++I) {
     if (!rdmp.m_pRoadmap->IsEdge(_cfg, *I) && 
	 lp->IsConnected(rdmp.GetEnvironment(), Stats, cd, dm, _cfg, *I, 
			 _ci, rdmp.GetEnvironment()->GetPositionRes(), 
			 rdmp.GetEnvironment()->GetOrientationRes(), 
			 true, true)) {
       *_vid = rdmp.m_pRoadmap->GetVID(*I);
       VID _cfgVID;
       if(rdmp.m_pRoadmap->IsVertex(_cfg))
	 _cfgVID = rdmp.m_pRoadmap->GetVID(_cfg);
       else
	 _cfgVID = rdmp.m_pRoadmap->AddVertex(_cfg);
       rdmp.m_pRoadmap->AddEdge(_cfgVID, *_vid, _ci->edge);
       return true;
     } else {
       // clear out previous (ie, "old") connection attempt
       _ci->path.erase(_ci->path.begin(), _ci->path.end());
     }     
   }
   *_vid = INVALID_VID;
   return false;
}


template <class CFG, class WEIGHT>
bool 
Query<CFG, WEIGHT>::
CanRecreatePath(vector<pair<CFG,WEIGHT> >& attemptedPath, 
		Stat_Class& Stats, CollisionDetection* cd, 
		LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric* dm, 
		vector<CFG>& recreatedPath) {
  LPOutput<CFG,WEIGHT> ci;

  for(vector<pair<CFG,WEIGHT> >::iterator I = attemptedPath.begin(); 
      (I+1) != attemptedPath.end(); ++I) {
    if(!(lp->GetPathSegment(rdmp.GetEnvironment(), Stats, cd, dm,
			    I->first, (I+1)->first, I->second, &ci,
			    rdmp.GetEnvironment()->GetPositionRes(),
			    rdmp.GetEnvironment()->GetOrientationRes(),
			    true, true))) {
      rdmp.m_pRoadmap->DeleteEdge(I->first, (I+1)->first);
      return false;
    } else {
      recreatedPath.insert(recreatedPath.end(), 
			   ci.path.begin(), ci.path.end());
    } 
  }
  return true;
}

//===================================================================
// Query class Methods: Display, Input, Output
//===================================================================

template <class CFG, class WEIGHT>
void 
Query<CFG, WEIGHT>::
ReadQuery(const char* _filename) {


  CFG tempCfg;
  
  ifstream  myifstream(_filename);
  if (!myifstream) {
    cout << endl << "In ReadQuery: can't open infile: " << _filename ;
    return;
  }
  
  
  while (1) {
    tempCfg.Read(myifstream);
    if(!myifstream) break;
    query.push_back(tempCfg);
  }
  
  myifstream.close();
};

template <class CFG, class WEIGHT>
void 
Query<CFG, WEIGHT>::
WritePath() {
  WritePath( outputPathFile );
}

template <class CFG, class WEIGHT>
void 
Query<CFG, WEIGHT>::
WritePath(char* _filename ) {
  vector<Cfg*> ppath;
  for(int i=0; i<path.size(); i++)
    ppath.push_back(&path[i]);
  WritePathConfigurations(_filename, ppath, rdmp.GetEnvironment());
}



#endif
