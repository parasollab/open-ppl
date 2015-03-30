#ifndef RADIAL_SUBDIVISION_RRT_H_
#define RADIAL_SUBDIVISION_RRT_H_

#include "ParallelSBMPHeader.h"
#include "WorkFunctions/RadialRRT.h"

///TODO check correctness of region creation (uniformly divided regions)
///TODO Resurrect remove_cycles somehow
template<class MPTraits>
class RadialSubdivisionRRT : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef graph_view<GraphType>  GraphView;

    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
            RadialRegion<MPTraits>, WeightType> RadialRegionGraph;
    typedef graph_view<RadialRegionGraph> RegionGraphView;
    typedef stapl::counter<stapl::default_timer> staplTimer;

    RadialSubdivisionRRT(MPProblemType* _problem, XMLNodeReader& _node);
    RadialSubdivisionRRT();
    virtual ~RadialSubdivisionRRT();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual double ComputeRandomRayLength(Boundary& _cBoundary);
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

    // Run Methods

    void RegionVertex(RegionGraphView _regionView, CfgType _root);

    void RegionEdge(RegionGraphView _regionView);

    void BuildRRT(RegionGraphView _regionView, CfgType _root);

    void ConnectRegions(RegionGraphView _regionView);

    void RemoveCycles(RoadmapGraph<CfgType, WeightType>* _rmg);

  protected:
    size_t m_numRegions, m_numNeighbors,m_numNodes,m_runs,m_numAttempts;
    double m_radius, m_delta,m_minDist;
    string m_vcLabel, m_dmLabel, m_nfLabel, m_eLabel, m_connectorLabel;
    bool m_strictBranching;
    double m_overlap;
    MPProblemType* m_problem;
};

template<class MPTraits>
RadialSubdivisionRRT<MPTraits>::
RadialSubdivisionRRT(MPProblemType* _problem,
    XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("RadialSubdivisionRRT");
    ParseXML(_node);
  }

template<class MPTraits>
RadialSubdivisionRRT<MPTraits>::
RadialSubdivisionRRT() {
  this->SetName("RadialSubdivisionRRT");
}

template<class MPTraits>
RadialSubdivisionRRT<MPTraits>::
~RadialSubdivisionRRT() { }

template<class MPTraits>
void RadialSubdivisionRRT<MPTraits>::ParseXML(XMLNodeReader& _node){
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "region_constr") {
      m_numRegions = citr->numberXMLParameter("numRegions", true, 1, 0, MAX_INT, "Number of Regions");
      m_radius = citr->numberXMLParameter("rayLength", true, 0.0, 0.0, MAX_DBL, "Random Ray Length");
      m_numNeighbors = citr->numberXMLParameter("numNeighbors", true,1,0, MAX_INT, "Number of Adjacent Regions");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "rrt_constr") {
      m_numNodes = citr->numberXMLParameter("numNodes", true, 1, 0, MAX_INT, "Number of samples per region");
      m_delta = citr->numberXMLParameter("delta", true, 0.0, 0.0, MAX_DBL, "Delta Variable for ExtendTree method");
      m_minDist = citr->numberXMLParameter("minDist", true, 0.0, 0.0, MAX_DBL, "Minimum Distance to see if new node is too close to closet cfg");
      m_numAttempts = citr->numberXMLParameter("numAttempts", true, 1, 0, MAX_INT,  "Number of samples to attempt per region");
      m_strictBranching = citr->boolXMLParameter("strictBranching", true, false, "if true, root only has 'regions' number of edges");
      m_overlap = citr->numberXMLParameter("overlap", false, 0.0, 0.0, 0.99, "percentage of the overlap of regions");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "num_runs") {
      m_runs = citr->numberXMLParameter(string("nRuns"), true,
          int(1), int(0), MAX_INT, string("Runs number"));
      citr->warnUnrequestedAttributes();
    }
    else if (citr->getName() == "vc_method") {
      m_vcLabel = citr->stringXMLParameter("vcm", true,"", "Validity Checker Method");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "connectionMethod") {
      m_connectorLabel = citr->stringXMLParameter("Label", true, "", "Region connection method");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "dm_method") {
      m_dmLabel = citr->stringXMLParameter("Method", true, "", "Distance Metric method");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "nf_method") {
      m_nfLabel = citr->stringXMLParameter("Method", true, "", "Neighborhood Finder method");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "e_method") {
      m_eLabel = citr->stringXMLParameter("Method", true, "", "Extender method");
      citr->warnUnrequestedAttributes();
    }
    else {
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
  return 2*_cBoundary.GetMaxDist();
}

template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
RegionVertex(RegionGraphView _regionView, CfgType _root) {
  DistanceMetricPointer dmm = this->GetDistanceMetric(m_dmLabel);
  Environment* env = this->GetEnvironment();

  if(_root.PosDOF() == 2) {
    CfgType point;
    point.GetRandomRay(m_radius, env, dmm);
    RadialRegionVertex2D<MPTraits> wf(m_numRegions, point, m_radius);
    stapl::for_each(_regionView, wf);
  }
  else {
    RadialRegionVertex<MPTraits> wf(this->GetMPProblem(), _root, m_radius, m_dmLabel);
    stapl::for_each(_regionView, wf);
  }
}

template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
RegionEdge(RegionGraphView _regionView) {
  RadialRegionEdge<MPTraits> wf(this->GetMPProblem(),
      m_numRegions > m_numNeighbors ? m_numNeighbors : m_numRegions-1, m_dmLabel);
  map_func(wf, _regionView, make_repeat_view(_regionView));
}

template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
BuildRRT(RegionGraphView _regionView, CfgType _root) {
  // no expansion type or CCconnection labels required
  BuildRadialRRT<MPTraits> wf(this->GetMPProblem(), m_numNodes,
      m_dmLabel, m_vcLabel, m_nfLabel, m_eLabel, m_delta, m_minDist,
      _root, m_numAttempts, m_overlap, m_strictBranching);
  stapl::for_each(_regionView,wf);
}

template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
ConnectRegions(RegionGraphView _regionView) {
  ConnectorPointer pConnection = this->GetConnector(m_connectorLabel);
  ConnectRegion<MPTraits> wf(this->GetMPProblem(), pConnection);
  map_func(wf, _regionView, make_repeat_view(_regionView));
}

template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
RemoveCycles(RoadmapGraph<CfgType, WeightType>* _rmg) {
  GraphView rmView(*_rmg);
  ///TODO resurrect this function?
  //remove_cycles(rmView);
}

template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
Run() {
  cout << "RadialSubdivisionRRT:: Run()" << endl;
  //Set up variables
  MPProblemType* problem = this->GetMPProblem();
  Environment* env = problem->GetEnvironment();

  ///If random ray length is not given, then compute approximate value from given boundary
  if(m_radius <= 0.0) m_radius = ComputeRandomRayLength(*(env->GetBoundary()));

  RadialRegionGraph radialRegion;
  radialRegion.add_vertex(RadialRegion<MPTraits>(CfgType()));

  RegionGraphView regionView(radialRegion);
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
  t0.start();

  cout << "STEP 1: Randomly generate n points with  m_radius length from the center" << endl;
  t1.start();

  RegionVertex(regionView, root);

  t1.stop();
  rmi_fence();

  cout << "STEP 2: Make edge between k-closest regions " << endl;
  ///For each vertex v find k closest to v in a map_reduce fashion
  t2.start();

  if(regionView.size() > 1)
    RegionEdge(regionView);

  t2.stop();
  rmi_fence();

  cout << "STEP 3: Construct RRT in each region " << endl;
  t3.start();

  BuildRRT(regionView, root);

  t3.stop();
  rmi_fence();

  PrintOnce("Num of Edges before: ", pMap->num_edges());
  rmi_fence();

  cout << "STEP 4 : Connect branches in each region " << endl;
  t4.start();

  ConnectRegions(regionView);

  t4.stop();
  rmi_fence();

  PrintOnce("Num of Edges after: ", pMap->num_edges());
  cout << "STEP 5 : Remove Cycles " << endl;

  t5.start();

  RemoveCycles(pMap);

  t5.stop();

  t0.stop();

  PrintOnce("Num of Edges after Remove Cycle: ", pMap->num_edges());
  rmi_fence();

  //if(get_location_id() == 0){
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
  //  pMap->delete_vertex(0);
  //}

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
  write_adj_list(regionView, "radialRegion.out");

  rmi_fence();
}


template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
Finalize() {
  stringstream basefname;
  basefname << this->GetBaseFilename();// << ".p" << stapl::get_num_locations() << ".r" << m_numRegions;
  this->GetMPProblem()->GetRoadmap()->Write(basefname.str() + ".map", this->GetMPProblem()->GetEnvironment());
  stapl::rmi_fence();
  cout << "location [" << stapl::get_location_id() <<"] ALL FINISHED" << endl;
}

template<class MPTraits>
void
RadialSubdivisionRRT<MPTraits>::
Print(ostream& _os) const {
  _os << "RadialSubdivisionRRT\n";
}


#endif
