//////////////////////////////////
//HEADER RadialSubdivisionRRT.h
/////////////////////////////////

#ifndef RADIALSUBDIVISIONRRT_H_
#define RADIALSUBDIVISIONRRT_H_

#include "ParallelSBMPHeader.h"
#include "WorkFunctions/RadialRRT.h"

//typedef typename stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, RadialRegion, WeightType> RadialRegionGraph;

template<class MPTraits>
class RadialSubdivisionRRT : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef graph_view<typename GraphType::GRAPH>  GraphView;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;
    typedef typename stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, 
            RadialRegion<MPTraits>, WeightType> RadialRegionGraph;
    typedef stapl::counter<stapl::default_timer> staplTimer;

    RadialSubdivisionRRT(MPProblemType* _problem, XMLNodeReader& _node);
    RadialSubdivisionRRT();
    virtual ~RadialSubdivisionRRT();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual double ComputeRandomRayLength(Boundary& _cBoundary);
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

    // Run Methods

    void RegionVertex (graph_view<RadialRegionGraph> _regionView, MPProblemType*_problem, CfgType _root);

    void RegionEdge(graph_view<RadialRegionGraph> _regionView, MPProblemType* _problem);

    void BuildRRT(graph_view<RadialRegionGraph> _regionView, MPProblemType* _problem, CfgType _root);

    void ConnectRegions(graph_view<RadialRegionGraph> _regionView, MPProblemType* _problem);

    void RemoveCycles(RoadmapGraph<CfgType, WeightType>* _rmg);

  protected:
    size_t m_numRegions, m_numNeighbors,m_numNodes,m_runs,m_numAttempts; 
    double m_radius, m_delta,m_minDist;  
    string m_vcLabel, m_dmLabel, m_nfLabel, m_connectorLabel;
    bool m_strictBranching;
    double m_overlap;
    MPProblemType* m_problem;

};

template<class MPTraits>
RadialSubdivisionRRT<MPTraits>::RadialSubdivisionRRT(MPProblemType* _problem, 
XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
  this->SetName("RadialSubdivisionRRT");
    ParseXML(_node);
}

template<class MPTraits>
RadialSubdivisionRRT<MPTraits>::RadialSubdivisionRRT() {
  this->SetName("RadialSubdivisionRRT");      
}

template<class MPTraits>
RadialSubdivisionRRT<MPTraits>::~RadialSubdivisionRRT() { }

template<class MPTraits>
void RadialSubdivisionRRT<MPTraits>::ParseXML(XMLNodeReader& _node){
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "region_constr") {
      m_numRegions = citr->numberXMLParameter("numRegions", true, 1, 0, MAX_INT, "Number of Regions");
      m_radius = citr->numberXMLParameter("rayLength", true, 0.0, 0.0, MAX_DBL, "Random Ray Length");
      m_numNeighbors = citr->numberXMLParameter("numNeighbors", true,1,0, MAX_INT, "Number of Adjacent Regions");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "rrt_constr") {
      m_numNodes = citr->numberXMLParameter("numNodes", true, 1, 0, MAX_INT, "Number of samples per region");
      m_delta = citr->numberXMLParameter("delta", true, 0.0, 0.0, MAX_DBL, "Delta Variable for ExtendTree method");
      m_minDist = citr->numberXMLParameter("minDist", true, 0.0, 0.0, MAX_DBL, "Minimum Distance to see if new node is too close to closet cfg");
      m_numAttempts = citr->numberXMLParameter("numAttempts", true, 1, 0, MAX_INT,  "Number of samples to attempt per region");
      m_strictBranching = citr->boolXMLParameter("strictBranching", true, false, "if true, root only has 'regions' number of edges");
      m_overlap = citr->numberXMLParameter("overlap", false, 0.0, 0.0, 0.99, "percentage of the overlap of regions"); 
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "num_runs") {
      m_runs = citr->numberXMLParameter(string("nRuns"), true, 
          int(1), int(0), MAX_INT, string("Runs number"));
      citr->warnUnrequestedAttributes();
    }else if (citr->getName() == "vc_method") {
      m_vcLabel = citr->stringXMLParameter("vcm", true,"", "Validity Checker Method");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "connectionMethod") {
      m_connectorLabel = citr->stringXMLParameter("Label", true, "", "Region connection method");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "dm_method") {
      m_dmLabel = citr->stringXMLParameter("Method", true, "", "Distance Metric method");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "nf_method") {
      m_nfLabel = citr->stringXMLParameter("Method", true, "", "Neighborhood Finder method");
      citr->warnUnrequestedAttributes();
    }else {
      citr->warnUnknownNode();
    }
  }
};

template<class MPTraits>
void RadialSubdivisionRRT<MPTraits>::Initialize() {
  cout << "RadialSubdivisionRRT::Initialize()" <<endl;
}


///Compute approximate ray lenght from boundary if not given by user  
template<class MPTraits>
double RadialSubdivisionRRT<MPTraits>::ComputeRandomRayLength(Boundary& _cBoundary) {
  double xrange = (_cBoundary.GetRange(0).second - _cBoundary.GetRange(0).first);
  double yrange = (_cBoundary.GetRange(1).second - _cBoundary.GetRange(1).first);
  double zrange = 0.0;
  if (_cBoundary.GetPosDOFs() > 2) 
    zrange = (_cBoundary.GetRange(2).second - _cBoundary.GetRange(2).first);
  double branges[] = {xrange,yrange,zrange};
  return ((5*(*max_element(branges,branges+_cBoundary.GetPosDOFs()))));
}

template<class MPTraits>
void 
RadialSubdivisionRRT<MPTraits>::RegionVertex (graph_view<RadialRegionGraph> _regionView, MPProblemType*_problem, CfgType _root) {

  DistanceMetricPointer dmm = _problem->GetDistanceMetric(m_dmLabel);
  Environment* env = _problem->GetEnvironment();
  CfgType point = _root;
  point.GetRandomRay(m_radius, env, dmm);

  if (_root.PosDOF() == 2) {
    RadialRegionVertex2D<MPTraits> wf(m_numRegions,point,m_radius);  
    new_algorithms::for_each(_regionView,wf);
  } else {
    RadialRegionVertex<MPTraits> wf(_problem, _root, m_radius, m_dmLabel);  
    new_algorithms::for_each(_regionView,wf);
  }
}

template<class MPTraits>
void 
RadialSubdivisionRRT<MPTraits>::RegionEdge(graph_view<RadialRegionGraph> _regionView, MPProblemType* _problem) {
  RadialRegionEdge<MPTraits> wf(_problem,m_numNeighbors,m_dmLabel);
  map_func(wf,_regionView,repeat_view(_regionView));
}

template<class MPTraits>
void 
RadialSubdivisionRRT<MPTraits>::BuildRRT(graph_view<RadialRegionGraph> _regionView, 
    MPProblemType* _problem, CfgType _root) {
  // no expansion type or CCconnection labels required
  BuildRadialRRT<MPTraits> wf(_problem,m_numNodes,m_dmLabel,m_vcLabel,m_nfLabel,m_delta,m_minDist,
      _root,m_numAttempts, m_overlap, m_strictBranching);

//  MPStrategyPointer strategy = this->GetMPProblem()->GetMPStrategy("BlindRRT");
//  BuildRadialBlindRRT<MPTraits> wf(strategy, _root, m_radius, m_strictBranching, m_overlap); 
  new_algorithms::for_each(_regionView,wf);
}

template<class MPTraits>
void 
RadialSubdivisionRRT<MPTraits>::ConnectRegions(graph_view<RadialRegionGraph> _regionView, 
    MPProblemType* _problem) {

  ConnectorPointer pConnection;
  pConnection = _problem->GetMPStrategy()->GetConnector()->GetMethod(m_connectorLabel);
  ConnectRegion<MPTraits> wf(_problem, pConnection);
  map_func(wf, _regionView, repeat_view(_regionView));
}

template<class MPTraits>
void 
RadialSubdivisionRRT<MPTraits>::RemoveCycles(RoadmapGraph<CfgType, WeightType>* _rmg) {
  GraphView rmView(*_rmg);
  remove_cycles(rmView);
}

template<class MPTraits>
void RadialSubdivisionRRT<MPTraits>::Run() {
  cout << "RadialSubdivisionRRT:: Run()" << endl;
  //Set up variables 
  MPProblemType* problem = this->GetMPProblem();
  StatClass* regionStats = problem->GetStatClass();
  Environment* env = problem->GetEnvironment();

  ///If random ray length is not given, then compute approximate value from given boundary
  if(m_radius <= 0.0) m_radius = ComputeRandomRayLength(*(env->GetBoundary()));

  RadialRegionGraph radialRegion(m_numRegions);
  graph_view<RadialRegionGraph> regionView(radialRegion);
  GraphType* pMap = problem->GetRoadmap()->GetGraph();
  rmi_fence();

  CfgType root;
  ///Assumption: root is at the center and is valid
  ///delayed check for validity- if root is not valid, delete it and then connect all edges
  ///from it to one of its target (still need to check those edges are valid
  /// randomize root generated by location 0, can be anywhere or native state 

  if(stapl::get_location_id() == 0){ 
    pMap->add_vertex(root);
    // TODO VIZMO DEBUG
    VDAddNode(root);
  }

  PrintOnce("ROOT  : ", root);
  PrintOnce("LENGTH  : ", m_radius); 
  rmi_fence();

  staplTimer t0,t1,t2,t3,t4,t5;
  cout << "STEP 1: Randomly generate n points with  m_radius length from the center" << endl; 
  t0.start();
  t1.start();
  RegionVertex(regionView, problem, root);
  t1.stop();
  rmi_fence(); 

  cout << "STEP 2: Make edge between k-closest regions " << endl;
  ///For each vertex v find k closest to v in a map_reduce fashion
  t2.start();
  RegionEdge(regionView, problem);
  t2.stop();
  rmi_fence();

  cout << "STEP 3: Construct RRT in each region " << endl;
  t3.start();
  BuildRRT(regionView, problem, root);
  t3.stop();
  rmi_fence();
  PrintOnce("Num of Edges before: ", pMap->num_edges());
  rmi_fence();

  cout << "STEP 4 : Connect branches in each region " << endl;
  t4.start();
  //  ConnectRegions(regionView, problem);
  t4.stop();
  rmi_fence();

  PrintOnce("Num of Edges after: ", pMap->num_edges());
  cout << "STEP 5 : Remove Cycles " << endl;
  t5.start();
  //  RemoveCycles(pMap);
  t5.stop();
  t0.stop();
  PrintOnce("Num of Edges after Remove Cycle: ", pMap->num_edges());
  rmi_fence();



  if(get_location_id() == 0){

    //  DEBUGGIN REGIONS 
    /*
       typename RoadmapGraph<CfgType, WeightType>::vertex_iterator vi = pMap->find_vertex(0);
       typename RoadmapGraph<CfgType, WeightType>::adj_edge_iterator ei = (*vi).begin();

       for(; ei != (*vi).end(); ++ei){
       VID target = ((*ei).target()); 
       cout << "Finding Vertex::" << target << endl;
       CfgType cfg1 = (*(pMap->find_vertex(target))).property();
       VDRemoveEdge(cfg1, root);
       VDRemoveEdge(root, cfg1);
       } 
       */
    pMap->delete_vertex(0);

  }


  /*PrintOnce("REGIONGRAPH VERTEX TIME: ", t1.value());
    PrintOnce("REGIONGRAPH EDGE TIME: ", t2.value());
    PrintOnce("BUILDRRT TIME: ", t3.value());
    PrintOnce("REGIONCONNECT TIME: ", t4.value());
    PrintOnce("REMOVE CYCLE TIME: ", t5.value());
    PrintOnce("TOTAL TIME : ", t0.value());
    PrintOnce("Tree size :" , pMap->size());
    PrintOnce("Num of Edges after : ", pMap->num_edges());*/

  //STATS
  stapl::rmi_fence();
  if(get_location_id() == 0){
    stringstream basefname;
    basefname << this->GetBaseFilename() << ".N" << m_numNodes << ".R" << m_numRegions << ".p" << get_num_locations() << ".run" << m_runs;
    ofstream stat_out((basefname.str() + ".stats").c_str());

    stat_out << "#P \t RGV  \t RGE  \t BRRT \t RCON  \t CYCLE \t TOT \t VERT \t EDGES "  << endl;

    stat_out << get_num_locations() << "\n" << t1.value() << "\n" << t2.value() << "\n" << t3.value()
      << "\n" << t4.value() <<  "\n" << t5.value()  << t0.value() 
      << "\n" << pMap->size() << "\n" << pMap->num_edges() << endl;

    stat_out.close();	   
  }

  rmi_fence();
  ///DEBUG
  cout << "Write Region Graph " << endl;
  write_graph(regionView, "radialRegion.out");
  
  rmi_fence(); 

}


template<class MPTraits>
void RadialSubdivisionRRT<MPTraits>::Finalize(){
  string str;
  stringstream basefname;
  MPProblemType* problem = this->GetMPProblem();

  basefname << this->GetBaseFilename() << ".p" << stapl::get_num_locations() << ".r" << m_numRegions;
  ofstream osMap((basefname.str() + ".map").c_str());
  if(!osMap){
    cout << "RadialSubdivisionRRT::Finalize(): can't open outfile: ";
    exit(-1);
  }else{
     problem->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
    osMap.close();
  }
  stapl::rmi_fence();
  cout << "location [" << stapl::get_location_id() <<"] ALL FINISHED" << endl;
}

template<class MPTraits>
void RadialSubdivisionRRT<MPTraits>::PrintOptions(ostream& _os){
  if(this->m_debug) _os << "RadialSubdivisionRRT:: PrintOptions \n";
}


#endif 
