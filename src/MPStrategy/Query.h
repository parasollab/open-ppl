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

#include "Connector.h"

template <class CFG, class WEIGHT>
class Query {

public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
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
        virtual ~Query();

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
/*	virtual
	  bool CanRecreatePath(Roadmap<CFG, WEIGHT>* rdmp, 
			       vector<pair<CFG,WEIGHT> >& attemptedPath,
			       Stat_Class& Stats, 
			       LocalPlanners<CFG,WEIGHT>* lp, 
			       DistanceMetric* dm, 
			       vector<CFG>& recreatedPath);
*/
	virtual
	  bool CanRecreatePath(Roadmap<CFG, WEIGHT>* rdmp, 
			       vector<VID>& attemptedPath,
			       StatClass& Stats,  
						 LocalPlanners<CFG,WEIGHT>* lp, 
             string _lp_label,
			       shared_ptr< DistanceMetricMethod> dm, 
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
        virtual bool PerformQuery(
            Roadmap<CFG, WEIGHT>* rdmp, 
            StatClass& Stats, 
            Connector<CFG, WEIGHT> *cn, 
            vector<typename Connector<CFG, WEIGHT>::ConnectionPointer >* pConnections,
            LocalPlanners<CFG,WEIGHT> * lp, 
            string _lp_label,
            shared_ptr< DistanceMetricMethod > dm,
            bool intermediateFiles = false);

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
        virtual bool PerformQuery(
            CFG _start, 
            CFG _goal,
            Roadmap<CFG, WEIGHT>* rdmp,
            StatClass& Stats,
            Connector<CFG, WEIGHT>*, 
            vector<typename Connector<CFG, WEIGHT>::ConnectionPointer >* pConnections,
            LocalPlanners<CFG,WEIGHT>*, 
            string _lp_label,
            shared_ptr<DistanceMetricMethod>, 
            vector<CFG>* _path, 
            bool intermediateFiles = false);

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
class QueryConnect : public Connector<CFG,WEIGHT> {
 public:
  QueryConnect() : Connector<CFG,WEIGHT>() {}
  QueryConnect(Roadmap<CFG,WEIGHT>* rm,
               shared_ptr< DistanceMetricMethod> dm) ://, LocalPlanners<CFG,WEIGHT>* lp) :
    Connector<CFG,WEIGHT>() {}
  ~QueryConnect() {
    this->selected_node_methods.clear();
    this->all_node_methods.clear();

    this->selected_component_methods.clear();
    this->all_component_methods.clear();
    
    //this->selected_roadmap_methods.clear();
    //this->all_roadmap_methods.clear();
  }

  virtual vector<ConnectionMethod<CFG,WEIGHT>*> GetMethodDefault(){
    vector<ConnectionMethod<CFG,WEIGHT>*> tmp;
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
PerformQuery(Roadmap<CFG, WEIGHT>* rdmp, StatClass& Stats, 
        Connector<CFG, WEIGHT>* cn, vector<typename Connector<CFG, WEIGHT>::ConnectionPointer >* pConnections,
				LocalPlanners<CFG,WEIGHT>* lp, 
        string _lp_label, shared_ptr<DistanceMetricMethod> dm, bool intermediateFiles) {
  for(typename vector<CFG>::iterator Q = query.begin(); 
      (Q+1) != query.end(); ++Q) {
    cout << "\nquery is ...     ";
    Q->Write(cout);
    cout << "\n                 ";
    (Q+1)->Write(cout);
    cout << "\nworking  ...     "
	 << endl;
    
    if ( !PerformQuery(*Q,*(Q+1),rdmp, Stats,cn,pConnections,lp,
                       _lp_label,dm,&path, intermediateFiles) ) {
      cout << endl << "In PerformQuery(): didn't connect";
      return false;
    } 
  }
  
  return true;
};

template <class CFG, class WEIGHT>
bool 
Query<CFG, WEIGHT>::
PerformQuery(CFG _start, CFG _goal, Roadmap<CFG, WEIGHT>* rdmp, StatClass& Stats, 
	     Connector<CFG, WEIGHT>* cn, vector<typename Connector<CFG, WEIGHT>::ConnectionPointer >* pConnections, 
			 LocalPlanners<CFG,WEIGHT>* lp, 
       string _lp_label, shared_ptr<DistanceMetricMethod> dm, vector<CFG>* _path, bool intermediateFiles) {
  VDComment("Begin Query");
  LPOutput<CFG,WEIGHT> sci, gci;   // connection info for start, goal nodes

  //  vector<VID> cc; 
  vector< pair<size_t,VID> > ccs;
  stapl::sequential::vector_property_map< RoadmapGraph<CFG, WEIGHT>,size_t > cmap;
  get_cc_stats(*(rdmp->m_pRoadmap),cmap,ccs);  
  
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
  typename vector<pair<size_t,VID> >::const_iterator CC, ccsBegin = ccs.begin();
  for(CC = ccs.begin(); CC != ccs.end(); ++CC) {
    //store cc vids if needed
    vector<VID> cc; 


    //attempt to connect start and goal to cc
    cmap.reset();
    if(stapl::sequential::is_same_cc(*(rdmp->m_pRoadmap),cmap, svid, CC->second)) 
      cout << "start already connected to CC[" << distance(ccsBegin,CC)+1 << "]\n";
    else
    {
      cmap.reset();
      stapl::sequential::get_cc(*(rdmp->m_pRoadmap), cmap, CC->second, cc);
      vector<VID> verticesList(1, svid);
      cout << "connecting start to CC[" << distance(ccsBegin,CC)+1 << "]";

      for (typename vector<typename Connector<CFG,WEIGHT>::ConnectionPointer>::iterator itr = pConnections->begin(); itr != pConnections->end(); itr++) {
        (*itr)->Connect(rdmp, Stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }

   cmap.reset();
    if(stapl::sequential::is_same_cc(*(rdmp->m_pRoadmap),cmap, gvid, CC->second))
      cout << "goal already connected to CC[" << distance(ccsBegin,CC)+1 << "]\n";
    else
    {
      if(cc.empty()){
	cmap.reset();
        stapl::sequential::get_cc(*(rdmp->m_pRoadmap), cmap, CC->second, cc);}
      cout << "connecting goal to CC[" << distance(ccsBegin,CC)+1 << "]";
      vector<VID> verticesList(1, gvid);
      
      for (typename vector<typename Connector<CFG,WEIGHT>::ConnectionPointer>::iterator itr = pConnections->begin(); itr != pConnections->end(); itr++) {
        cmap.reset();
        (*itr)->Connect(rdmp, Stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }
    

    connected = false;
    vector<VID> _rp;
    cmap.reset();
    while(stapl::sequential::is_same_cc(*(rdmp->m_pRoadmap), cmap, svid, gvid)) {
      //get DSSP path
      _rp.clear();
      cmap.reset();
      //TO DO:: fix compilation issue in parallel
      #ifndef _PARALLEL
      stapl::sequential::find_path_dijkstra(*(rdmp->m_pRoadmap), svid, gvid, _rp, WEIGHT::MaxWeight()); 
      //cout << "FindPathDijkstra:: size of vector<VID>" << _rp.size() << endl ;
      #endif 
      cout << "\nStart(" << size_t(_rp[1]) 
	   << ") and Goal(" << size_t(_rp[_rp.size()-2]) 
	   << ") seem connected to same CC[" << distance(ccsBegin, CC)+1 
	   << "]!" << endl;
    
      //attempt to recreate path
      vector<CFG> recreatedPath;
      if(CanRecreatePath(rdmp, _rp, Stats, lp, 
                         _lp_label, dm, recreatedPath)) {
	connected = true;
       _path->insert(_path->end(),
                     recreatedPath.begin(), recreatedPath.end());
       break;
      } else
        cout << endl << "Failed to recreate path\n";
    }
    if(connected) {
      if(intermediateFiles){
        //Print out all start, all graph nodes, goal
        //ie, *NO* "ticks" from local planners
        vector<CFG> _mapcfgs;
        for(typename vector<VID>::iterator I = _rp.begin();
            I != _rp.end(); ++I)
          _mapcfgs.push_back((*(rdmp->m_pRoadmap->find_vertex(*I))).property());
        WritePathConfigurations("mapnodes.path", _mapcfgs, rdmp->GetEnvironment());
      }
      break;    
    }
  }
  VDComment("End Query");
  return connected;
};


//interface and inside modified, lantao
template <class CFG, class WEIGHT>
bool
Query<CFG, WEIGHT>::
CanRecreatePath(Roadmap<CFG, WEIGHT>* rdmp,
                vector<VID >& attemptedPath,
                StatClass& Stats,
                LocalPlanners<CFG,WEIGHT>* lp, 
                string _lp_label,
                shared_ptr< DistanceMetricMethod> dm,
                vector<CFG>& recreatedPath) {

  recreatedPath.push_back((*(rdmp->m_pRoadmap->find_vertex( *(attemptedPath.begin()) ))).property());
  for(typename vector<VID>::iterator I = attemptedPath.begin();
      (I+1) != attemptedPath.end(); ++I) {
    LPOutput<CFG,WEIGHT> ci;
    typename RoadmapGraph<CFG, WEIGHT>::vertex_iterator vi;
    typename RoadmapGraph<CFG, WEIGHT>::adj_edge_iterator ei;
    typename RoadmapGraph<CFG, WEIGHT>::edge_descriptor ed((*(rdmp->m_pRoadmap->find_vertex(*I))).descriptor(),(*(rdmp->m_pRoadmap->find_vertex(*(I+1) ))).descriptor());
    rdmp->m_pRoadmap->find_edge(ed, vi, ei);
    if(!(lp->GetMethod(_lp_label)->
         IsConnected(rdmp->GetEnvironment(), Stats, dm,
                     (*(rdmp->m_pRoadmap->find_vertex(*I))).property(), 
                     (*(rdmp->m_pRoadmap->find_vertex(*(I+1) ))).property(), 
                     &ci, rdmp->GetEnvironment()->GetPositionRes(),
                     rdmp->GetEnvironment()->GetOrientationRes(),true, true))) {
      rdmp->m_pRoadmap->delete_edge(*I, *(I+1));
      return false;
    } else {
      recreatedPath.insert(recreatedPath.end(),
                           ci.path.begin(), ci.path.end());
      recreatedPath.push_back((*(rdmp->m_pRoadmap->find_vertex(*(I+1)))).property());
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
  for(size_t i=0; i<path.size(); i++)
    ppath.push_back(&path[i]);
  WritePathConfigurations(_filename, ppath, rdmp->GetEnvironment());
}



#endif
