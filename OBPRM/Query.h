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
  for (int i=0; i < query.size()-1; i++ ){
    cout << "\nquery is ...     ";
    query[i].Write(cout);
    cout << "\n                 ";
    query[i+1].Write(cout);
    cout << "\nworking  ...     "
	 << endl;
    
    if ( !PerformQuery(query[i],query[i+1],Stats,cd,cn,lp,dm,&path) ) {
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
  
  vector< pair<int,VID> > ccs;
  GetCCStats(*(rdmp.m_pRoadmap),ccs);  
  
  bool connected = false;
  int  i, thiscc = 0;
  VID scvid, gcvid;
  
  while ( !connected && thiscc < ccs.size() ) {
    
    //get cc
    CFG t;
    t.equals(rdmp.m_pRoadmap->GetData(ccs[thiscc].second));
    vector<CFG> cc;
    GetCC(*(rdmp.m_pRoadmap) , t,cc);
    //connect start and goal to cc
    if ( CanConnectToCC(_start, Stats, cd,cn,lp,dm,cc,&scvid,&sci) && 
         CanConnectToCC(_goal,  Stats, cd,cn,lp,dm,cc,&gcvid,&gci) ) {
      
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
      typename vector<CFG>::iterator I;
      for(I=sci.path.begin(); I!=sci.path.end(); I++)
	_path->push_back(*I);

       LPOutput<CFG,WEIGHT> ci;

       if ( scvid != gcvid ) {
	 
	 vector< pair<CFG,WEIGHT> > rp;
	 FindPathDijkstra(*(rdmp.m_pRoadmap),scvid,gcvid,rp);
	 //cout<<rp.size()<<endl;
	 
#if INTERMEDIATE_FILES
	 //-----------------------------------------------------
	 // Print out all start, all graph nodes, goal
	 // ie, *NO* "ticks" from local planners
	 //-----------------------------------------------------
	 vector<CFG> _mapcfgs;
	 WritePathConfigurations("mapnodes.path", 
				 _mapcfgs, 
				 rdmp.GetEnvironment());
#endif
	 
	 for (i=0; i<rp.size()-1; i++) {
	   if (!(lp->GetPathSegment(rdmp.GetEnvironment(), Stats, cd, dm, rp[i].first, rp[i+1].first, rp[i].second, &ci, rdmp.GetEnvironment()->GetPositionRes(), rdmp.GetEnvironment()->GetOrientationRes(), true, true)) ) {
	     cout << endl << "In PerformQuery: can't recreate path" << endl;
	   } else {
	     // Add to Path rdmp cfg's & "tick"s: [ rdmp.rdmp ]
	     typename vector<CFG>::iterator I;
	     for(I=ci.path.begin(); I!=ci.path.end(); I++)
	       _path->push_back(*I);
	   }
	   
	 } // for i
	 
       } //if ( scvid != gcvid )
       
       // Add to Path "tick" cfg's: [ rdmp.goal ]
       reverse(gci.path.begin(),gci.path.end());
       typename vector<CFG>::iterator J;
       for(J=gci.path.begin(); J!=gci.path.end(); J++)
	 _path->push_back(*J);

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


template <class CFG, class WEIGHT>
bool
Query<CFG, WEIGHT>::
CanConnectToCC(CFG _cfg, Stat_Class& Stats, CollisionDetection* cd,
	       ConnectMap<CFG, WEIGHT>* cn, LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric* dm,
	       vector<CFG> _cc, VID* _vid, LPOutput<CFG,WEIGHT>*_ci) {

   // erase previous connection attempt. 
   // (when start or goal connection is sucessful.
   _ci->path.erase( _ci->path.begin(),_ci->path.end() );

   // sort the cfgs in _cc by distance from _cfg
   //cn->SortByDistFromCfg(rdmp.GetEnvironment(),dm,cn->cnInfo,_cfg,_cc);
   vector<ConnectionMethod<CFG,WEIGHT> *> selected;
   selected = cn->GetDefault();
   //this is needed to use any ConnectionMethod to access
   //function SortByDistFromCfg
   typename vector<ConnectionMethod<CFG,WEIGHT> *>::iterator first_method;
   first_method = selected.begin();
   (*first_method)->SortByDistFromCfg(rdmp.GetEnvironment(),dm,_cfg,_cc);
   // try to connect _cfg to (closest) config in _cc 
   // (now try all, later only k closest)
   for (int i=0; i < _cc.size(); i++ ) {
     if (!rdmp.m_pRoadmap->IsEdge(_cfg,_cc[i])
	 && lp->IsConnected(rdmp.GetEnvironment(), Stats, cd, dm, _cfg, _cc[i], _ci,rdmp.GetEnvironment()->GetPositionRes(), rdmp.GetEnvironment()->GetOrientationRes(), true, true) ) {
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
