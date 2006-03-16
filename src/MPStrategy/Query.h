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
         *This is useful when client needs to init data member by themself.
         */
        Query();

        /**
          *Preferred*, fills input.
          */
        Query
        (QueryCmds*);  

        /**
         * Read in query from a file and set every thing else to NULL. 
	 * Currently there is no output.
	 * This is useful when client only want to perform some queries,
	 * for example, in QueryTest.
	 */
        Query(const char* queryFileName);

        /**
         * Used given start/goal to set up query and set every thing else to NULL
	 * Currently there is no output
	 * This is useful when client only want to perform some queries,
	 * for example, in QueryTest.
	 */
        Query(CFG _start, CFG _goal);

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
	virtual
	  bool CanRecreatePath(Roadmap<CFG, WEIGHT>* rdmp, 
			       vector<pair<CFG,WEIGHT> >& attemptedPath,
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
        (Roadmap<CFG, WEIGHT>* rdmp, Stat_Class& Stats, CollisionDetection *cd, ConnectMap<CFG, WEIGHT> *cn, LocalPlanners<CFG,WEIGHT> * lp,DistanceMetric * dm);

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
			  Roadmap<CFG, WEIGHT>* rdmp,
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
	void WritePath(Roadmap<CFG, WEIGHT>* rdmp);

        ///Write path data to given file.
        virtual 
	void WritePath(Roadmap<CFG, WEIGHT>* rdmp, char* _filename);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    vector<CFG> query;                  ///< start,[intermediate nodes],goal 
    vector<CFG> path;                   ///< output paths

    char *outputPathFile;               ///< File name for path data

};


template <class CFG, class WEIGHT>
class QueryConnect : public ConnectMap<CFG,WEIGHT> {
 public:
  QueryConnect() : ConnectMap<CFG,WEIGHT>() {}
  QueryConnect(Roadmap<CFG,WEIGHT>* rm, CollisionDetection* cd,
	       DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp) :
    ConnectMap<CFG,WEIGHT>() {}
  ~QueryConnect() {
    this->selected_node_methods.clear();
    this->all_node_methods.clear();

    this->selected_component_methods.clear();
    this->all_component_methods.clear();
    
    //this->selected_roadmap_methods.clear();
    //this->all_roadmap_methods.clear();
  }

  virtual vector<NodeConnectionMethod<CFG,WEIGHT>*> GetNodeDefault() {
    vector<NodeConnectionMethod<CFG,WEIGHT>*> tmp;
    ConnectFirst<CFG,WEIGHT>* connectFirst = new ConnectFirst<CFG,WEIGHT>();
    connectFirst->SetDefault();
    tmp.push_back(connectFirst);
    return tmp;
  }

  virtual vector<ComponentConnectionMethod<CFG,WEIGHT>*> GetComponentDefault() {
    vector<ComponentConnectionMethod<CFG,WEIGHT>*> tmp;
    return tmp;
  }

  /*
  virtual vector<RoadmapConnectionMethod<CFG,WEIGHT>*> GetRoadmapDefault() {
    vector<RoadmapConnectionMethod<CFG,WEIGHT>*> tmp;
    return tmp;
  }
  */
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
Query(QueryCmds* Qinput)  
{
    ReadQuery( Qinput->queryFile.GetValue() );
    
    outputPathFile = new char[strlen(Qinput->pathFile.GetValue())+1];
    strcpy(outputPathFile,Qinput->pathFile.GetValue());
}

template <class CFG, class WEIGHT> 
Query<CFG, WEIGHT>::
Query(const char* queryFileName)  
{
    ReadQuery( queryFileName );  
    outputPathFile = NULL;  
}

template <class CFG, class WEIGHT>     
Query<CFG, WEIGHT>::
Query(CFG _start, CFG _goal)  
{
    query.push_back(_start);
    query.push_back(_goal);
    outputPathFile = NULL;  
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
PerformQuery(Roadmap<CFG, WEIGHT>* rdmp, Stat_Class& Stats, CollisionDetection* cd, ConnectMap<CFG, WEIGHT>* cn, LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric* dm) 
{
  for(typename vector<CFG>::iterator Q = query.begin(); 
      (Q+1) != query.end(); ++Q) {
    cout << "\nquery is ...     ";
    Q->Write(cout);
    cout << "\n                 ";
    (Q+1)->Write(cout);
    cout << "\nworking  ...     "
	 << endl;
    
    if ( !PerformQuery(*Q,*(Q+1),rdmp, Stats,cd,cn,lp,dm,&path) ) {
      cout << endl << "In PerformQuery(): didn't connect";
      return false;
    } 
  }
  
  return true;
};

template <class CFG, class WEIGHT>
bool 
Query<CFG, WEIGHT>::
PerformQuery(CFG _start, CFG _goal, Roadmap<CFG, WEIGHT>* rdmp, Stat_Class& Stats, CollisionDetection* cd,
	     ConnectMap<CFG, WEIGHT>* cn, LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric* dm, vector<CFG>* _path) {

  LPOutput<CFG,WEIGHT> sci, gci;   // connection info for start, goal nodes
  VID scvid, gcvid;

  //  vector<VID> cc; 
  vector< pair<int,VID> > ccs;
  GetCCStats(*(rdmp->m_pRoadmap),ccs);  
  
  VID svid;
  if(rdmp->m_pRoadmap->IsVertex(_start))
    svid = rdmp->m_pRoadmap->GetVID(_start);
  else
    svid = rdmp->m_pRoadmap->AddVertex(_start);
  VID gvid;
  if(rdmp->m_pRoadmap->IsVertex(_goal))
    gvid = rdmp->m_pRoadmap->GetVID(_goal);
  else
    gvid = rdmp->m_pRoadmap->AddVertex(_goal);

  bool connected = false;
  vector<pair<int,VID> >::const_iterator CC, ccsBegin = ccs.begin();
  for(CC = ccs.begin(); CC != ccs.end(); ++CC) {
    //get cc vids
      vector<VID> cc; 
    GetCC(*(rdmp->m_pRoadmap), CC->second, cc);

    //attempt to connect start and goal to cc
    bool addPartialEdge = false; //??
    bool addAllEdges = false; //??

    vector<VID> verticesList(1, svid);
    cout << "connecting start to CC[" << distance(ccsBegin,CC)+1 << "]";
    //    cout << "CC size = " << CC->first << endl;
    cn->ConnectNodes(rdmp, Stats, cd, dm, lp, false, false,
		     verticesList, cc);

    cout << "connecting goal to CC[" << distance(ccsBegin,CC)+1 << "]";
    //    cout << "CC size = " << CC->first << endl;
    verticesList[0] = gvid;
    cn->ConnectNodes(rdmp, Stats, cd, dm, lp, false, false,
		     verticesList, cc);

    connected = false;
    vector<pair<CFG,WEIGHT> > rp;
    while(IsSameCC(*(rdmp->m_pRoadmap), svid, gvid)) {
      //get DSSP path
      rp.clear();
      FindPathDijkstra(*(rdmp->m_pRoadmap), svid, gvid, rp);

      cout << "\nStart(" << rdmp->m_pRoadmap->GetVID(rp[1].first)
	   << ") and Goal(" << rdmp->m_pRoadmap->GetVID(rp[rp.size()-2].first)
	   << ") seem connected to same CC[" << distance(ccsBegin, CC)+1 
	   << "]!" << endl;
    
      //attempt to recreate path
      vector<CFG> recreatedPath;
      if(CanRecreatePath(rdmp, rp, Stats, cd, lp, dm, recreatedPath)) {
	connected = true;
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
      for(typename vector<pair<CFG,WEIGHT> >::iterator I = rp.begin(); 
	  I != rp.end(); ++I)
        _mapcfgs.push_back(I->first);
      WritePathConfigurations("mapnodes.path", _mapcfgs, rdmp->GetEnvironment());
#endif
      break;    
    }
  }
  
  return connected;
};


template <class CFG, class WEIGHT>
bool 
Query<CFG, WEIGHT>::
CanRecreatePath(Roadmap<CFG, WEIGHT>* rdmp, 
		vector<pair<CFG,WEIGHT> >& attemptedPath, 
		Stat_Class& Stats, CollisionDetection* cd, 
		LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric* dm, 
		vector<CFG>& recreatedPath) {
  LPOutput<CFG,WEIGHT> ci;

  recreatedPath.push_back(attemptedPath.begin()->first);
  for(typename vector<pair<CFG,WEIGHT> >::iterator I = attemptedPath.begin(); 
      (I+1) != attemptedPath.end(); ++I) {
    if(!(lp->GetPathSegment(rdmp->GetEnvironment(), Stats, cd, dm,
			    I->first, (I+1)->first, I->second, &ci,
			    rdmp->GetEnvironment()->GetPositionRes(),
			    rdmp->GetEnvironment()->GetOrientationRes(),
			    true, true))) {
      rdmp->m_pRoadmap->DeleteEdge(I->first, (I+1)->first);
      return false;
    } else {
      recreatedPath.insert(recreatedPath.end(), 
			   ci.path.begin(), ci.path.end());
      recreatedPath.push_back((I+1)->first);
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
WritePath(Roadmap<CFG, WEIGHT>* rdmp) {
  WritePath( rdmp, outputPathFile );
}

template <class CFG, class WEIGHT>
void 
Query<CFG, WEIGHT>::
WritePath(Roadmap<CFG, WEIGHT>* rdmp, char* _filename ) {
  vector<Cfg*> ppath;
  for(int i=0; i<path.size(); i++)
    ppath.push_back(&path[i]);
  WritePathConfigurations(_filename, ppath, rdmp->GetEnvironment());
}



#endif
