//////////////////////////////////
//HEADER RegularSubdivisionMethod.h
/////////////////////////////////

#ifndef REGULARSUBDIVISIONMETHOD_H_
#define REGULARSUBDIVISIONMETHOD_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "RegionDecomposition/Region.h"
#include "MPProblem/BoundingBox.h"
#include "ParallelMethods/WorkFunctions/BasicDecomposition.h"
#include "ParallelMethods/WorkFunctions/ConstructRegionMap.h"
#include "ParallelMethods/WorkFunctions/RegionMapConnect.h"


/*class XMLNodeReader;
class MPProblem;
template<class CFG, class WEIGHT> class ConnectCCs;*/

template<class MPTraits>
class RegularSubdivisionMethod : public MPStrategyMethod<MPTraits>{
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;
   // typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    
    typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region<BoundingBox,MPTraits>, WeightType> RRGraph;
    typedef typename RRGraph::vertex_descriptor RVID; 
    typedef typename RRGraph::vertex_iterator RVI;
    
    //non-xml/empty constructor
    RegularSubdivisionMethod(const vector<pair<string, int> >& _vecStrNodeGenLabels = vector<pair<string, int> >(), 
       const vector<string>& _vecStrNodeConnectionLabels = vector<string>(),
       const vector<string>& _strategiesLabels = vector<string>(),
       string _ccc= "", int _row = 0, int _col = 0);

    RegularSubdivisionMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~RegularSubdivisionMethod(){ };

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize(){ };
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

  private:
    vector<string> m_regionConnectionLabels;
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_strategiesLabels;
    int m_row,m_col,m_runs, m_k1, m_k2;
    double m_xEpsilon, m_yEpsilon, m_zEpsilon;
    string m_nf, m_ccc, m_lp;
    CCsConnector<MPTraits>* m_ccConnector;
   // MPProblem* m_problem; 
};

template<class MPTraits>
RegularSubdivisionMethod<MPTraits>::RegularSubdivisionMethod(const vector<pair<string, int> >& _vecStrNodeGenLabels, 
  const vector<string>& _vecStrNodeConnectionLabels, const vector<string>& _strategiesLabels,
  string _ccc, int _row, int _col)
  :m_vecStrNodeGenLabels(_vecStrNodeGenLabels), m_vecStrNodeConnectionLabels(_vecStrNodeConnectionLabels), 
   m_strategiesLabels(_strategiesLabels),m_ccc(_ccc), m_row(_row), m_col(_col){
  this->SetName("RegularSubdivisionMethod");
}

template<class MPTraits>
RegularSubdivisionMethod<MPTraits>::RegularSubdivisionMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MPStrategyMethod<MPTraits>(_problem,_node) {
 // m_problem = _problem;
  ParseXML(_node);
}


//RegularSubdivisionMethod::~RegularSubdivisionMethod() { }
template<class MPTraits>
void RegularSubdivisionMethod<MPTraits>::ParseXML(XMLNodeReader& _node){
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "sequential_strategy") {
      string strategy_string = citr->stringXMLParameter(string("Strategy"), true,
        string(""), string("Sequential Strategy"));
      m_strategiesLabels.push_back(strategy_string);
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "node_generation_method") {
      string node_gen_method = citr->stringXMLParameter(string("Method"), true,
        string(""), string("Node Generation Method"));
      int numPerIteration = citr->numberXMLParameter(string("Number"), true, 
        int(1), int(0), MAX_INT, string("Number of samples"));
      m_vecStrNodeGenLabels.push_back(pair<string, int>(node_gen_method, numPerIteration));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "node_connection_method") {
      string connect_method = citr->stringXMLParameter(string("Method"), true,
        string(""), string("Node Connection Method"));
      m_vecStrNodeConnectionLabels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "region_connection_method"){
      string connectRegionMethod = citr->stringXMLParameter(string("Method"), true, 
        string(""), string("Region Connection Method"));
      m_regionConnectionLabels.push_back(connectRegionMethod);
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "num_row") {
      m_row = citr->numberXMLParameter(string("nRow"), true, 
        int(1), int(1), MAX_INT, string("number of partition on x"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "n_col") {
      m_col = citr->numberXMLParameter(string("nCol"), true, 
        int(1), int(1), MAX_INT, string("number of partition on y"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "num_runs") {
      m_runs = citr->numberXMLParameter(string("nRuns"), true, 
        int(1), int(0), MAX_INT, string("Runs number"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "overlap") {
      // All 3 epsilon values are doubles between 0.0 to 1.0 with default value 0.0.
      m_xEpsilon = citr->numberXMLParameter("xeps", true, 0.0, 0.0, 1.0, "x overlap percentage");
      m_yEpsilon = citr->numberXMLParameter("yeps", true, 0.0, 0.0, 1.0, "y overlap percentage");
      m_zEpsilon = citr->numberXMLParameter("zeps", true, 0.0, 0.0, 1.0, "z overlap percentage");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "region_connect_k") {
      m_k1 = citr->numberXMLParameter("k1", true, 
        1, 0, MAX_INT, "K Largest CC from source region");
      m_k2 = citr->numberXMLParameter("k2", true, 
	1, 0, MAX_INT, "K Largest CC from target region");
      m_nf = citr->stringXMLParameter("nf", true,
	"", "Neighborhood Finder for Region Connect");
      m_lp = citr->stringXMLParameter("lp", true,
        "", "Local Planner for Region Connect");
      m_ccc = citr->stringXMLParameter("type", true,
	"", "CC connection strategy option");

      citr->warnUnrequestedAttributes();
    } else {
      citr->warnUnknownNode();
    }
  }
};


/*void RegularSubdivisionMethod::Initialize() {
  cout << "RegularSubdivisionMethod::Initialize()" <<endl;
}*/

template<class MPTraits>
void RegularSubdivisionMethod<MPTraits>::Run() {
  
  cout << "RegularSubdivisionMethod:: Run()" << endl;
  
  Environment* env = this->GetMPProblem()->GetEnvironment();
  GraphType* rmg = this->GetMPProblem()->GetRoadmap()->GetGraph(); 
  BasicDecomposition* decomposer = new BasicDecomposition();
  //shared_ptr<DistanceMetricMethod> dmm = this->GetMPProblem()->GetDistanceMetric()->GetMethod("scaledEuclidean");
  //shared_ptr<ValidityCheckerMethod> vcm = this->GetMPProblem()->GetValidityChecker()->GetMethod("cd1");
 // typedef typename Connector<MPTraits> NC;
  //typedef NC::ConnectionPointer NCP;
 // NC* nc = this->GetMPProblem()->GetConnector();
 
  m_ccConnector = new CCsConnector<MPTraits>(this->GetMPProblem(), m_lp,m_nf);
 // m_ccConnector->SetMPProblem(this->GetMPProblem());
  
   ///TIMER STUFF
  
  stapl::counter<stapl::default_timer> t0,t1,t2,t3;
  double constr_tm=0.0, cc_tm=0, rg_constr_tm=0.0, total_tm=0.0;
   
  typedef vector<pair<string, int> >::iterator I;
  typedef vector<string>::iterator J;
  typedef CCsConnector<MPTraits>* CCP;
  typedef std::tr1::tuple<CCP,string, int> ccConnectParam;
  typedef std::tr1::tuple<ConnectorPointer,string, int> ncConnectParam;
  typedef array< BoundingBox > arrayBbox;
  typedef array_view <arrayBbox> viewBbox;
  
  int mesh_size = m_row * m_col;
  int num_samples;
  
  BoundingBox* bbox = dynamic_cast<BoundingBox*>(&*env->GetBoundary());
  
  arrayBbox pArrayBbox(mesh_size, *bbox);
  
  ///MAKE A MESH GRAPH
  RRGraph regularRegion;
  stapl::generators::mesh<RRGraph> meshGraph(regularRegion,m_row, m_col);
  meshGraph.add_vertices();
  rmi_fence();
  meshGraph.add_edges();
  rmi_fence();
  
  graph_view<RRGraph> regionView(regularRegion);
  
  rmi_fence();
  
  ////DECOMPOSE SPACE TO REGIONS
  if( stapl::get_location_id() == 0){
    decomposer->DecomposeWS(env,bbox, m_row, m_col,1, pArrayBbox.begin(), m_xEpsilon,m_yEpsilon, m_zEpsilon);
  }
  rmi_fence();
  
  viewBbox arrView(pArrayBbox);
  rmi_fence();
  
  t1.start();
  PrintValue("Region graph size " , regularRegion.size());
  //PrintValue("Mesh Graph size " , meshGraph.size());
  
  
  if (m_strategiesLabels.size() != 0){
  
    ///CONSTRUCT REGION MAP---COARSE GRAINED -- takes the whole sequential strategy
    for(J itr1 = m_strategiesLabels.begin(); itr1 != m_strategiesLabels.end(); ++itr1) {
    PrintValue("View size " , regionView.size());
    MPStrategyPointer strategy = this->GetMPProblem()->GetMPStrategy(*itr1); 
    ConstructRoadmap<MPTraits> constrRegionMap(strategy);
    map_func(constrRegionMap, regionView,arrView);
    }
  }else {
  
    ///CONSTRUCT REGION MAP --- FINE GRAINED-- node gen first then connect  
    ////GENERATE NODES IN REGIONS
    for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr){
      num_samples = itr->second;  
      SamplerPointer sp = this->GetMPProblem()->GetSampler(itr->first);
      NodeGenerator<MPTraits> nodeGen(this->GetMPProblem(),sp,num_samples);
      map_func(nodeGen,arrView,regionView);
    }
  
    ///REDISTRIBUTE/MIGRATE - based on RegionVIDs and RegionWeight() 
  
    ///CONNECT NODES IN REGIONS
    for(J itr1 = m_vecStrNodeConnectionLabels.begin(); itr1 != m_vecStrNodeConnectionLabels.end(); ++itr1){
      ConnectorPointer cp = this->GetMPProblem()->GetConnector(*itr1);  
      NodeConnector<MPTraits> nodeCon(this->GetMPProblem(), cp);
      new_algorithms::for_each(regionView,nodeCon);
    }
  }
  rmi_fence();

  ///DEBUG
  constr_tm = t1.stop();
  PrintValue("CNSTR : " , constr_tm);
  PrintOnce("RUN::# of regions ", regularRegion.num_vertices());
  PrintOnce("RUN::# of region edges: ", regularRegion.num_edges());
  PrintOnce("RUN::roadmap graph size ", rmg->num_vertices());
  PrintOnce("RUN::roadmap graph edges before: ", rmg->num_edges());
  rmi_fence();
  
  /// COMPUTE CCs AND SET REGION CCs
  typedef graph_view<GraphType> view_type;
  view_type rmView(*rmg);
  rmi_fence();
  typedef static_array<cc_color_property> property_storage_type;
  typedef graph_external_property_map<view_type, cc_color_property, property_storage_type> property_map_type;
				      
  ///TODO: proper fix by making cc_color_property derived from cfg class
  /// and then use internal_property_map
  property_storage_type prop_storage(2*num_samples*mesh_size);
  property_map_type     map(rmView, &prop_storage);
  
  connected_components(rmView, map);
  rmi_fence();
  
  std::vector<pair<VID,size_t> > ccVec1 = cc_stats(rmView,map);
  rmi_fence();
  
  PrintOnce("cc count before region con:", ccVec1.size());
  rmi_fence();
  if(m_ccc == "largest"){ 
    array_view<std::vector<pair<VID,size_t> > > ccView1(ccVec1);
    map_func(SetRegionCC<MPTraits>(), regionView, balance_view(ccView1,regionView.size()));
    rmi_fence();
  
    ///CONNECT REGIONS ROADMAP 
       ccConnectParam conParam1 = std::tr1::make_tuple(m_ccConnector,m_ccc,m_k1);
   
    RegionCCConnector<RRGraph, Region<BoundingBox, MPTraits>, property_map_type, MPTraits> regionCCCon(this->GetMPProblem(), &regularRegion, map, conParam1);  
    new_algorithms::for_each(regionView,regionCCCon);
  }else if(m_ccc== "random") { 
    for(J itr2 = m_regionConnectionLabels.begin(); itr2 != m_regionConnectionLabels.end(); ++itr2) {
      ConnectorPointer ncp = this->GetMPProblem()->GetConnector(*itr2); 
      ncConnectParam conParam2 = std::tr1::make_tuple(ncp,m_ccc,m_k1);

      RegionRandomConnector<RRGraph, Region<BoundingBox, MPTraits>, MPTraits > regionRandomCon(this->GetMPProblem(), &regularRegion, conParam2);
      new_algorithms::for_each(regionView,regionRandomCon);
    }
  }else {

    cerr << "ERROR::Please choose an existing k_closest_cc connection type "<< endl;
    cerr << "Reference this error on line "<< __LINE__ << " of file " << __FILE__ << endl;
    exit(-1);

  }
  
  ///DEBUG
  PrintOnce("RUN::roadmap graph edges after: ", rmg->num_edges());
  rmi_fence();
  
  map.reset();
  connected_components(rmView, map);
  rmi_fence();
  std::vector<pair<VID,size_t> > ccVec2 = cc_stats(rmView,map);
  rmi_fence();
  
  PrintOnce("cc count after region con: ", ccVec2.size());
  rmi_fence();
   
  ///WRITE REGION GRAPH :: DEBUG
  //TODO write_graph(regionView,"rgFile.out");
  
  rmi_fence();
  
  //bbox->Clear();
}

template<class MPTraits>
void RegularSubdivisionMethod<MPTraits>::Finalize(){
  ///Write graph here :: DEBUG
  string str;
  stringstream basefname;
  basefname << this->GetBaseFilename() << ".p" << stapl::get_num_locations() << ".it" << m_runs;
  ofstream osMap((basefname.str() + ".map").c_str());
  if(!osMap){
     cout << "RegularSubdivisionMethod::Finalize(): can't open outfile: ";
     exit(-1);
  }else{
     this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
     osMap.close();
  }
  rmi_fence();
  cout << "location [" << get_location_id() <<"] ALL FINISHED" << endl;
}

template<class MPTraits>
void RegularSubdivisionMethod<MPTraits>::PrintOptions(ostream& _os){
   _os << "RegularSubdivisionMethod:: PrintOptions \n";
}

#endif 
