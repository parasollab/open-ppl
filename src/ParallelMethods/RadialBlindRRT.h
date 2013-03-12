//////////////////////////////////
//HEADER RadialBlindRRT.h
/////////////////////////////////

#ifndef RADIALBLINDRRT_H_
#define RADIALBLINDRRT_H_

#include "ParallelSBMPHeader.h"
#include "WorkFunctions/RadialRRT.h"
#include "RadialSubdivisionRRT.h"


template<class MPTraits>
class RadialBlindRRT : public RadialSubdivisionRRT<MPTraits> {
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


    RadialBlindRRT(MPProblemType* _problem, XMLNodeReader& _node);
    RadialBlindRRT();
    virtual ~RadialBlindRRT();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

    // Run Methods

    void BuildRRT(graph_view<RadialRegionGraph> _regionView, MPProblemType* _problem, CfgType _root);

    void ConnectRegions(graph_view<RadialRegionGraph> _regionView, MPProblemType* _problem);

    void RemoveCycles(RoadmapGraph<CfgType, WeightType>* _rmg);

  protected:
    string m_expansionType;
    string m_CCconnection;
    size_t m_numCCIters;
};

template<class MPTraits>
RadialBlindRRT<MPTraits>::RadialBlindRRT(MPProblemType* _problem, 
    XMLNodeReader& _node) :
  RadialSubdivisionRRT<MPTraits>(_problem, _node) {
    this->SetName("RadialBlindRRT");
    ParseXML(_node);
  }

template<class MPTraits>
RadialBlindRRT<MPTraits>::RadialBlindRRT() {
  this->SetName("RadialBlindRRT");      
}

template<class MPTraits>
RadialBlindRRT<MPTraits>::~RadialBlindRRT() { }

template<class MPTraits>
void RadialBlindRRT<MPTraits>::ParseXML(XMLNodeReader& _node){
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "rrt_constr") {
      m_CCconnection = citr->stringXMLParameter("CCconnection",true,"","CC connection strategy");
      m_expansionType = citr->stringXMLParameter("expansionType", true, "", "Normal, Blind, Witness");
      m_numCCIters = citr->numberXMLParameter("numCCIters", true, 1, 0, MAX_INT, "Number of iteration for the local CC connection");
      citr->warnUnrequestedAttributes();
    }
  }
};

template<class MPTraits>
void RadialBlindRRT<MPTraits>::Initialize() {
  cout << "RadialBlindRRT::Initialize()" <<endl;
}

template<class MPTraits>
void 
RadialBlindRRT<MPTraits>::BuildRRT(graph_view<RadialRegionGraph> _regionView, 
    MPProblemType* _problem, CfgType _root) {
  ///adding this for strong scaling, replace with m_numNodes if interested in weak scaling
  size_t nodesPerRegion = this->m_numNodes/stapl::get_num_locations();
  BuildRadialBlindRRT<MPTraits> wf(_problem,nodesPerRegion,this->m_dmLabel,this->m_vcLabel,
      this->m_nfLabel,m_CCconnection,m_expansionType,this->m_delta,this->m_minDist,
      _root,this->m_numAttempts,m_numCCIters,this->m_overlap, this->m_strictBranching);

  //  MPStrategyPointer strategy = this->GetMPProblem()->GetMPStrategy("BlindRRT");
  //  BuildRadialBlindRRT<MPTraits> wf(strategy, _root, m_radius, m_strictBranching, m_overlap); 
  new_algorithms::for_each(_regionView,wf);
}

template<class MPTraits>
void 
RadialBlindRRT<MPTraits>::ConnectRegions(graph_view<RadialRegionGraph> _regionView, 
    MPProblemType* _problem) {

  ConnectorPointer pConnection;
  pConnection = _problem->GetConnector(this->m_connectorLabel);
  ConnectGlobalCCs<MPTraits> wf(_problem, pConnection);
  map_func(wf, _regionView, repeat_view(_regionView));
}

template<class MPTraits>
void 
RadialBlindRRT<MPTraits>::RemoveCycles(RoadmapGraph<CfgType, WeightType>* _rmg) {
  GraphView rmView(*_rmg);
  remove_cycles(rmView);
}

template<class MPTraits>
void RadialBlindRRT<MPTraits>::Run() {
  cout << "RadialBlindRRT:: Run()" << endl;
  //Set up variables 
  MPProblemType* problem = this->GetMPProblem();
  Environment* env = problem->GetEnvironment();

  ///If random ray length is not given, then compute approximate value from given boundary
  if(this->m_radius <= 0.0) this->m_radius = this->ComputeRandomRayLength(*(env->GetBoundary()));

  RadialRegionGraph radialRegion(this->m_numRegions);

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
    //VDAddNode(root);
  }
  //PrintOnce("ROOT  : ", root);
  //PrintOnce("LENGTH  : ", this->m_radius); 
  rmi_fence();

  staplTimer t0,t1,t2,t3,t4,t5;
  //cout << "STEP 1: Randomly generate n points with  m_radius length from the center" << endl; 
  t0.start();
  t1.start();
  this->RegionVertex(regionView, problem, root);
  t1.stop();
  PrintOnce("STEP 1 REGION VERTEX (s) : ", t1.value());
  rmi_fence(); 
  //cout << "STEP 2: Make edge between k-closest regions " << endl;
  ///For each vertex v find k closest to v in a map_reduce fashion
  t2.start();
  if (regionView.size() >1) this->RegionEdge(regionView, problem);
  t2.stop();
  PrintOnce("STEP 2 REGION EDGE (s) : ", t2.value());
  rmi_fence();
  //cout << "STEP 3: Construct Blind RRT in each region " << endl;
  t3.start();
  BuildRRT(regionView, problem, root);
  t3.stop();
  PrintOnce("STEP 3 BUILD RRT (s) : ", t3.value());
  rmi_fence();
 // cout << "STEP 4 : Global CC Connect " << endl;
  t4.start();
  if(regionView.size() > 1) ConnectRegions(regionView, problem);
  t4.stop();
  rmi_fence();

  //t5.start();
  //  RemoveCycles(pMap);
  //t5.stop();
  t0.stop();
  PrintOnce("STEP 4 CONNECT REGIONS (s) : ", t4.value());
  PrintOnce("TOTAL TIME (s) : ", t0.value());
  PrintOnce("#VERTICES  : ", pMap->num_vertices());
  PrintOnce("#EDGES  : ", pMap->num_edges());
  rmi_fence();



 // if(get_location_id() == 0){

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
   // pMap->delete_vertex(0);

 // }


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
    basefname << this->GetBaseFilename() << ".N" << this->m_numNodes << ".R" << this->m_numRegions << ".p" << get_num_locations() << ".run" << this->m_runs;
    ofstream stat_out((basefname.str() + ".stats").c_str());

    stat_out << "#P \t RGV  \t RGE  \t BRRT \t RCON   \t TOT \t VERT \t EDGES "  << endl;

    stat_out << get_num_locations() << "\n" << t1.value() << "\n" << t2.value() << "\n" << t3.value()
      << "\n" << t4.value() <<  "\n" <<  t0.value(); 
    stat_out << "\n" << pMap->size() << "\n" << pMap->num_edges() << endl;

    stat_out.close();	   
  }

  rmi_fence();
  ///DEBUG
  //cout << "Write Region Graph " << endl;
  //write_graph(regionView, "radialRegion.out");

  rmi_fence(); 

}


template<class MPTraits>
void RadialBlindRRT<MPTraits>::Finalize(){
  string str;
  stringstream basefname;
  MPProblemType* problem = this->GetMPProblem();

  basefname << this->GetBaseFilename() << ".p" << stapl::get_num_locations() << ".r" << this->m_numRegions;
  ofstream osMap((basefname.str() + ".map").c_str());
  if(!osMap){
    cout << "RadialBlindRRT::Finalize(): can't open outfile: ";
    exit(-1);
  }else{
    problem->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
    osMap.close();
  }
  stapl::rmi_fence();
  cout << "location [" << stapl::get_location_id() <<"] ALL FINISHED" << endl;
}

template<class MPTraits>
void RadialBlindRRT<MPTraits>::PrintOptions(ostream& _os){
  if(this->m_debug) _os << "RadialBlindRRT:: PrintOptions \n";
}


#endif 
