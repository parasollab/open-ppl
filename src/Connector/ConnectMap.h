#ifndef _ConnectMap_h_
#define _ConnectMap_h_



#include "Roadmap.h"
#include "DistanceMetrics.h"
#include "Clock_Class.h"
#include "util.h"

#include <sstream>

//node connection methods
#include "NodeConnectionMethod.h"
#include "NeighborhoodConnection.h"
#include "Closest.h"
//#include "ClosestUnconnected.h"
#include "ClosestSF.h"
//#include "AllPairsNodeConnection.h"
//#include "UnconnectedClosest.h"
//#include "RandomConnect.h"

// look these over before updating
//#include "ModifiedLM.h"
//#include "ObstBased.h"
//#include "ClosestVE.h"
//#include "RRTexpand.h"
//#include "RayTracer.h"
#include "ConnectFirst.h"

//component connection methods
#include "ComponentConnectionMethod.h"
#include "ConnectCCs.h"
#include "ConnectkCCs.h"

// do these last
//#include "RRTcomponents.h"
//#include "RayTracer.h"
//#include "Disconnect.h"

// MPRegion is used by region combination methods
///\todo Fix this include mess
#include "MPRegion.h"
//class MPRegion;
#include "MPProblem.h"
#include "util.h"
#include "PMPL_Container_Base.h"

// region connection methods
//#include "RegionConnectionMethod.h"
//#include "NaiveRegionConnect.h"
//#include "RegionOverlapMapCombine.h"


namespace pmpl_detail { //hide NeighborhoodFinderMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
      NeighborhoodConnection<CfgType,WeightType>
//    Closest<CfgType,WeightType>
//    ,ClosestUnconnected<CfgType,WeightType>
//    ,ClosestSF<CfgType,WeightType>
    > NodeConnectorMethodList;

  typedef boost::mpl::list<
    ConnectCCs<CfgType,WeightType>
    > ComponentConnectorMethodList;
}


//#############################################################################
// A collection of component connection methods
template <class CFG, class WEIGHT>
class ConnectMap : public PMPL_Container_Base< NodeConnectionMethod<CFG,WEIGHT>, 
                    pmpl_detail::NodeConnectorMethodList>, 
              public PMPL_Container_Base< ComponentConnectionMethod<CFG,WEIGHT>, 
                    pmpl_detail::ComponentConnectorMethodList >, 
              public MPBaseObject{
  private:
  typedef PMPL_Container_Base< NodeConnectionMethod<CFG,WEIGHT>, 
       pmpl_detail::NodeConnectorMethodList> NodeConnectionContainer;
  typedef PMPL_Container_Base< ComponentConnectionMethod<CFG,WEIGHT>, 
       pmpl_detail::ComponentConnectorMethodList > ComponentConnectionContainer;
 typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  public:
    typedef typename NodeConnectionContainer::method_pointer NodeConnectionPointer;
    typedef typename ComponentConnectionContainer::method_pointer ComponentConnectionPointer;

  

  //////////////////////
  // Constructors and destructor
  ConnectMap();
  ConnectMap(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ConnectMap(Roadmap<CFG,WEIGHT>*, 
	     DistanceMetric*, LocalPlanners<CFG,WEIGHT>*);
  ~ConnectMap();

  //////////////////////
  // Access methods



  NodeConnectionPointer GetNodeMethod(string& in_strLabel) {
    NodeConnectionPointer to_return = 
                            NodeConnectionContainer::GetMethod(in_strLabel);
    if(to_return.get() == NULL) {
      cerr << "ConnectMap::GetNodeMethod - ERROR: " 
           << in_strLabel << " not found." << endl;
      exit(-1);
    }
    return to_return;
  }

  ComponentConnectionPointer GetComponentMethod(string& in_strLabel) {
    ComponentConnectionPointer to_return = 
                        ComponentConnectionContainer::GetMethod(in_strLabel);
    if(to_return.get() == NULL) {
      cerr << "ConnectMap::GetComponentMethod - ERROR: " 
           << in_strLabel << " not found." << endl;
      exit(-1);
    }
    return to_return;
  }

  void AddNodeMethod(string in_strLabel, NodeConnectionPointer in_ptr) {
          NodeConnectionContainer::AddMethod(in_strLabel, in_ptr);
  }
  void AddComponentMethod(string in_strLabel, ComponentConnectionPointer in_ptr) {
        ComponentConnectionContainer::AddMethod(in_strLabel, in_ptr);
  }
  

  void SetNodeConnectionMethods(vector<NodeConnectionMethod<CFG,WEIGHT>*>& methods)
  {
    selected_node_methods = methods;
  }
  void SetComponentConnectionMethods(vector<ComponentConnectionMethod<CFG,WEIGHT>*>& methods)
  {
    selected_component_methods = methods;
  }
  //void SetRegionConnectionMethods(vector<RegionConnectionMethod<CFG,WEIGHT>*>& methods)
  //{
  //  selected_region_methods = methods;
  //}

  //////////////////////
  // I/O methods
  void PrintOptions(ostream& out_os);

  //////////////////////
  // Core: Connection methods


///
///
/// Begin new ConnectNodes interface
///
///

  void ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges) {
      //cout << "ConnectMap::ConnectNodes() - Roadmap only" << endl;
 }

  template<typename InputIterator>
  void ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last) {
      //cout << "ConnectMap::ConnectNodes() - 1 pair InputIterator" << endl;
      _ConnectNodes(selected, _rm, Stats, dm, lp,addPartialEdge,addAllEdges,
                    _itr1_first, _itr1_last,
                    typename NodeConnectionContainer::MethodTypes_begin(), 
                    typename NodeConnectionContainer::MethodTypes_end());
 }


  template<typename InputIterator>
  void ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
		    LocalPlanners<CFG,WEIGHT>* lp,
		    bool addPartialEdge, bool addAllEdges,
		    InputIterator _itr1_first, InputIterator _itr1_last,
        InputIterator _itr2_first, InputIterator _itr2_last) {
      //cout << "ConnectMap::ConnectNodes() - 2 pairs InputIterator" << endl;
      _ConnectNodes(selected, _rm, Stats, dm, lp,addPartialEdge,addAllEdges,
                    _itr1_first, _itr1_last, _itr2_first, _itr2_last,
                    typename NodeConnectionContainer::MethodTypes_begin(), 
                    typename NodeConnectionContainer::MethodTypes_end());
 }



  private:
  
  //implements the function call dispatching (b/c no support for templated virtual functions)
  template <typename InputIterator, typename First, typename Last>
  void 
  _ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        InputIterator _itr2_first, InputIterator _itr2_last, 
           First, Last); 
  template <typename InputIterator, typename Last>
  void
  _ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        InputIterator _itr2_first, InputIterator _itr2_last, 
          Last, Last) {
    cerr << "ERROR, dynamic_cast of NodeConnectionMethod failed, method type not found!\n\n";
    exit(-1);
    }


  //implements the function call dispatching (b/c no support for templated virtual functions)
  template <typename InputIterator, typename First, typename Last>
  void 
  _ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        First, Last); 

  template <typename InputIterator, typename Last>
  void
  _ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        Last, Last) {
    cerr << "ERROR, dynamic_cast of NodeConnectionMethod failed, method type not found!\n\n";
    exit(-1);
    }

///
///
/// End new ConnectNodes interface
///
///


///
///
/// Begin new ConnectComponents interface
///
///

public:
  void ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges) {
    //cout << "ConnectMap::ConnectComponents() - Roadmap only" << endl;
       _ConnectComponents(selected, rm, Stats,  dm, lp, addPartialEdge, addAllEdges,
          typename ComponentConnectionContainer::MethodTypes_begin(),
          typename ComponentConnectionContainer::MethodTypes_end());
  }

  template<typename InputIterator>
  void ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges,
       InputIterator _itr1_first, InputIterator _itr1_last) {
    //cout << "ConnectMap::ConnectComponents()" << endl;
      _ConnectComponents(selected, rm, Stats,  dm, lp, addPartialEdge, addAllEdges,
        _itr1_first, _itr1_last,
          typename ComponentConnectionContainer::MethodTypes_begin(),
          typename ComponentConnectionContainer::MethodTypes_end());
  }

  template<typename InputIterator>
  void ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
			 DistanceMetric* dm,
			 LocalPlanners<CFG,WEIGHT>* lp,
			 bool addPartialEdge, bool addAllEdges,
			 InputIterator _itr1_first, InputIterator _itr1_last,
       InputIterator _itr2_first, InputIterator _itr2_last) {
    //cout << "ConnectMap::ConnectComponents()" << endl;
      _ConnectComponents(selected, rm, Stats, dm, lp, addPartialEdge, addAllEdges,
        _itr1_first, _itr1_last, _itr2_first, _itr2_last,
          typename ComponentConnectionContainer::MethodTypes_begin(),
          typename ComponentConnectionContainer::MethodTypes_end());
  }

private:
  //implements the function call dispatching (b/c no support for templated virtual functions)
  template <typename First, typename Last>
  void
  _ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges,
       First, Last);


  template <typename Last>
  void
  _ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges,
       Last, Last){
    cerr << "ERROR, dynamic_cast of ComponentConnectionMethod failed, method type not found!\n\n";
    exit(-1);
    }

  //implements the function call dispatching (b/c no support for templated virtual functions)
  template <typename InputIterator, typename First, typename Last>
  void
  _ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges,
       InputIterator _itr1_first, InputIterator _itr1_last,
       InputIterator _itr2_first, InputIterator _itr2_last, 
       First, Last);


  template <typename InputIterator, typename Last>
  void
  _ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges,
       InputIterator _itr1_first, InputIterator _itr1_last,
       InputIterator _itr2_first, InputIterator _itr2_last, 
       Last, Last){
    cerr << "ERROR, dynamic_cast of ComponentConnectionMethod failed, method type not found!\n\n";
    exit(-1);
    }

  //implements the function call dispatching (b/c no support for templated virtual functions)
  template <typename InputIterator, typename First, typename Last>
  void
  _ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges,
       InputIterator _itr1_first, InputIterator _itr1_last,
       First, Last);


  template <typename InputIterator, typename Last>
  void
  _ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
       DistanceMetric* dm,
       LocalPlanners<CFG,WEIGHT>* lp,
       bool addPartialEdge, bool addAllEdges,
       InputIterator _itr1_first, InputIterator _itr1_last,
       Last, Last){
    cerr << "ERROR, dynamic_cast of ComponentConnectionMethod failed, method type not found!\n\n";
    exit(-1);
    }


///
///
/// End new ConnectComponents interface
///
///


 protected:
  //////////////////////
  // Data
  vector<NodeConnectionMethod<CFG,WEIGHT> *> all_node_methods;
  vector<NodeConnectionMethod<CFG,WEIGHT> *> selected_node_methods;

  vector<ComponentConnectionMethod<CFG,WEIGHT> *> all_component_methods;
  vector<ComponentConnectionMethod<CFG,WEIGHT> *> selected_component_methods;

  void ParseXML(XMLNodeReader& in_Node);
  
 public:
  CDInfo cdInfo;
  static double connectionPosRes, ///< Position resolution for node connection
         connectionOriRes; ///< Orientation resolution for node connection
};

template <class CFG, class WEIGHT>
double ConnectMap<CFG, WEIGHT>::connectionPosRes = 0.05;

template <class CFG, class WEIGHT>
double ConnectMap<CFG, WEIGHT>::connectionOriRes = 0.05;


////////////////////////////////////////////////////////////////////////////
// ConnectMap: Methods
template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
ConnectMap() {

  NeighborhoodConnection<CFG,WEIGHT>* neighborhoodconn = new NeighborhoodConnection<CFG,WEIGHT>();
  all_node_methods.push_back(neighborhoodconn);

  //setup component connection methods
  selected_component_methods.clear();
  all_component_methods.clear();

  ConnectCCs<CFG,WEIGHT>* connectccs = new ConnectCCs<CFG,WEIGHT>();
  all_component_methods.push_back(connectccs);

  ConnectkCCs<CFG,WEIGHT>* connectkccs = new ConnectkCCs<CFG,WEIGHT>();
  all_component_methods.push_back(connectkccs);


/*   RRTexpand<CFG,WEIGHT>* rrtexpand = new RRTexpand<CFG,WEIGHT>(); */
/*   all_component_methods.push_back(rrtexpand); */

/*   RRTcomponents<CFG,WEIGHT>* rrtcomp = new RRTcomponents<CFG,WEIGHT>(); */
/*   all_component_methods.push_back(rrtcomp); */

  //RayTracer<CFG,WEIGHT>* rt = new RayTracer<CFG,WEIGHT>();
  //all_component_methods.push_back(rt);


}

template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
ConnectMap(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
  MPBaseObject(in_Node, in_pProblem){
  LOG_DEBUG_MSG("ConnectMap::ConnectMap()");
  ParseXML(in_Node);
  
  
  if(selected_node_methods.size() < 1)
    LOG_WARNING_MSG("No Connection Methods selected!");

  LOG_DEBUG_MSG("~ConnectMap::ConnectMap()");
}


template <class CFG, class WEIGHT>
void ConnectMap<CFG,WEIGHT>::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("ConnectMap::ParseXML()");
  
  connectionPosRes = GetMPProblem()->GetEnvironment()->GetPositionRes();
  cout << "connectionPosRes = " << connectionPosRes << endl;
  connectionOriRes = GetMPProblem()->GetEnvironment()->GetOrientationRes();     
  cout << "connectionOriRes = " << connectionOriRes << endl;

  XMLNodeReader::childiterator citr;

  //Iterate over child nodes
  
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if (citr->getName() == "NeighborhoodConnection") {
      cout << "ConnectMap found NeighborhoodConnection" << endl;
      NeighborhoodConnection<CFG,WEIGHT>* neighborhoodconn
                      = new NeighborhoodConnection<CFG,WEIGHT>(*citr,GetMPProblem());
      AddNodeMethod(neighborhoodconn->GetLabel(),NodeConnectionPointer(neighborhoodconn));
      neighborhoodconn->cdInfo = &cdInfo;
      neighborhoodconn->connectionPosRes = connectionPosRes;
      neighborhoodconn->connectionOriRes = connectionOriRes;
    } else if(citr->getName() == "ConnectCCs") {
      cout << "ConnectMap found ConnectCCs" << endl;
      ConnectCCs<CFG,WEIGHT>* connectccs = new ConnectCCs<CFG,WEIGHT>(*citr,GetMPProblem());
      AddComponentMethod(connectccs->GetLabel(),ComponentConnectionPointer(connectccs));
      connectccs->cdInfo = &cdInfo;
      connectccs->connectionPosRes = connectionPosRes;
      connectccs->connectionOriRes = connectionOriRes; 
    } 
  }
  
  LOG_DEBUG_MSG("~ConnectMap::ParseXML()");
}

template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
ConnectMap(Roadmap<CFG,WEIGHT> * rdmp, 
	   DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp) {
  ConnectMap();
}


template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
~ConnectMap() {
  selected_node_methods.clear();
  all_node_methods.clear();

  selected_component_methods.clear();
  all_component_methods.clear();

}


  
  


template <class CFG, class WEIGHT>
void
ConnectMap<CFG,WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "  Connection Methods" << endl;
  typename vector<NodeConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I = all_node_methods.begin(); I != all_node_methods.end(); ++I)
    (*I)->PrintOptions(out_os);

  typename vector<ComponentConnectionMethod<CFG,WEIGHT>*>::iterator J;
  for(J = all_component_methods.begin(); J != all_component_methods.end(); ++J)
    (*J)->PrintOptions(out_os);

}



template <class CFG, class WEIGHT>
template <typename InputIterator, typename First, typename Last>
void 
ConnectMap<CFG,WEIGHT>::
_ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
      Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
      DistanceMetric * dm,
      LocalPlanners<CFG,WEIGHT>* lp,
      bool addPartialEdge, bool addAllEdges,
      InputIterator _itr1_first, InputIterator _itr1_last,
      InputIterator _itr2_first, InputIterator _itr2_last, 
          First, Last) {

  typedef typename boost::mpl::deref<First>::type MethodType;
  if(MethodType* finder = dynamic_cast<MethodType*>(selected.get()))
  { 
    //cout << "ConnectMap::_ConnectNodes 2 sets of InputIterator- "
    //     << finder->GetLabel() << endl << flush;
    finder->ConnectNodes(_rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                         _itr1_first, _itr1_last,
                         _itr2_first, _itr2_last);
    return;
  }
  else 
  {
    typedef typename boost::mpl::next<First>::type Next;
    _ConnectNodes(selected, _rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                    _itr1_first, _itr1_last,
                    _itr2_first, _itr2_last,
                    Next(), Last());
    return;
    
  }
}


template <class CFG, class WEIGHT>
template <typename InputIterator, typename First, typename Last>
void 
ConnectMap<CFG,WEIGHT>::
_ConnectNodes(shared_ptr<NodeConnectionMethod<CFG,WEIGHT> > selected, 
      Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
      DistanceMetric * dm,
      LocalPlanners<CFG,WEIGHT>* lp,
      bool addPartialEdge, bool addAllEdges,
      InputIterator _itr1_first, InputIterator _itr1_last,
      First, Last) {

  typedef typename boost::mpl::deref<First>::type MethodType;
  if(MethodType* finder = dynamic_cast<MethodType*>(selected.get()))
  {
    //cout << "ConnectMap::_ConnectNodes 1 set1 of InputIterator- " << finder->GetLabel() << endl;
    finder->ConnectNodes(_rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                         _itr1_first, _itr1_last);
    return;
  }
  else 
  {
    typedef typename boost::mpl::next<First>::type Next;
    _ConnectNodes(selected, _rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                    _itr1_first, _itr1_last,
                    Next(), Last());
    return;
    
  }
}


template <class CFG, class WEIGHT>
template <typename First, typename Last>
void
ConnectMap<CFG,WEIGHT>::
_ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
      Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
      DistanceMetric* dm,
      LocalPlanners<CFG,WEIGHT>* lp,
      bool addPartialEdge, bool addAllEdges,
      First, Last) {


  typedef typename boost::mpl::deref<First>::type MethodType;
  if(MethodType* finder = dynamic_cast<MethodType*>(selected.get()))
  {
    //cout << "ConnectMap::_ConnectComponents 0 sets of InputIterator- " << finder->GetLabel() << endl;
    finder->Connect(rm, Stats, dm, lp, addPartialEdge, addAllEdges);
    return;
  }
  else 
  {
    typedef typename boost::mpl::next<First>::type Next;
    _ConnectComponents(selected, rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                    Next(), Last());
    return;
  }
}


template <class CFG, class WEIGHT>
template <typename InputIterator, typename First, typename Last>
void
ConnectMap<CFG,WEIGHT>::
_ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
      Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
      DistanceMetric* dm,
      LocalPlanners<CFG,WEIGHT>* lp,
      bool addPartialEdge, bool addAllEdges,
      InputIterator _itr1_first, InputIterator _itr1_last,
      InputIterator _itr2_first, InputIterator _itr2_last, 
      First, Last) {


  typedef typename boost::mpl::deref<First>::type MethodType;
  if(MethodType* finder = dynamic_cast<MethodType*>(selected.get()))
  {
    //cout << "ConnectMap::_ConnectComponents 2 sets of InputIterator- " << finder->GetLabel() << endl;
    finder->Connect(rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                    _itr1_first, _itr1_last, _itr2_first, _itr2_last);
    return;
  }
  else 
  {
    typedef typename boost::mpl::next<First>::type Next;
    _ConnectComponents(selected, rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                    _itr1_first, _itr1_last,
                    _itr2_first, _itr2_last,
                    Next(), Last());
    return;
  }
}


template <class CFG, class WEIGHT>
template <typename InputIterator, typename First, typename Last>
void
ConnectMap<CFG,WEIGHT>::
_ConnectComponents(shared_ptr<ComponentConnectionMethod<CFG,WEIGHT> > selected, 
      Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
      DistanceMetric* dm,
      LocalPlanners<CFG,WEIGHT>* lp,
      bool addPartialEdge, bool addAllEdges,
      InputIterator _itr1_first, InputIterator _itr1_last,
      First, Last) {

  typedef typename boost::mpl::deref<First>::type MethodType;
  if(MethodType* finder = dynamic_cast<MethodType*>(selected.get()))
  {
    //cout << "ConnectMap::_ConnectComponents 1 set of InputIterator- " << finder->GetLabel() << endl;
    finder->Connect(rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                    _itr1_first, _itr1_last);
    return;
  }
  else 
  {
    typedef typename boost::mpl::next<First>::type Next;
    _ConnectComponents(selected, rm, Stats, dm, lp, addPartialEdge, addAllEdges,
                    _itr1_first, _itr1_last,
                    Next(), Last());
    return;    
  }
}

#endif /*_ConnectMap_h_*/
