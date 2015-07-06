#ifndef REGULARSUBDIVISIONMETHOD_H_
#define REGULARSUBDIVISIONMETHOD_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "RegionDecomposition/Region.h"
#include "ParallelMethods/WorkFunctions/BasicDecomposition.h"
#include "ParallelMethods/WorkFunctions/ConstructRegionMap.h"
#include "ParallelMethods/WorkFunctions/RegionMapConnect.h"


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RegularSubdivisionMethod : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    typedef typename stapl::dynamic_graph<
      stapl::DIRECTED, stapl::NONMULTIEDGES,
      Region<BoundingBox,MPTraits>, WeightType
        > RRGraph;
    typedef typename RRGraph::vertex_descriptor RVID;
    typedef typename RRGraph::vertex_iterator RVI;

    //non-xml/empty constructor
    RegularSubdivisionMethod(const vector<pair<string, int> >&
        _vecStrNodeGenLabels = vector<pair<string, int> >(),
        const vector<string>& _vecStrNodeConnectionLabels = vector<string>(),
        const vector<string>& _strategiesLabels = vector<string>(),
        string _ccc= "", int _row = 0, int _col = 0);

    RegularSubdivisionMethod(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    virtual ~RegularSubdivisionMethod(){ };

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize(){ };
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  private:
    vector<string> m_regionConnectionLabels;
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_strategiesLabels;
    string m_nf, m_ccc, m_lp;
    size_t m_row,m_col,m_runs, m_k1, m_k2;
    double m_xEpsilon, m_yEpsilon, m_zEpsilon;
    CCsConnector<MPTraits>* m_ccConnector;
};

template<class MPTraits>
RegularSubdivisionMethod<MPTraits>::
RegularSubdivisionMethod(const vector<pair<string, int> >& _vecStrNodeGenLabels,
    const vector<string>& _vecStrNodeConnectionLabels, const vector<string>& _strategiesLabels,
    string _ccc, int _row, int _col) :
  m_vecStrNodeGenLabels(_vecStrNodeGenLabels), m_vecStrNodeConnectionLabels(_vecStrNodeConnectionLabels),
  m_strategiesLabels(_strategiesLabels), m_ccc(_ccc), m_row(_row), m_col(_col){
    this->SetName("RegularSubdivisionMethod");
}

template<class MPTraits>
RegularSubdivisionMethod<MPTraits>::
RegularSubdivisionMethod(typename MPTraits::MPProblemType* _problem,
    XMLNode& _node) : MPStrategyMethod<MPTraits>(_problem,_node) {
  ParseXML(_node);
  this->SetName("RegularSubdivisionMethod");
}

template<class MPTraits>
void
RegularSubdivisionMethod<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node) {
    if(child.Name() == "sequential_strategy")
      m_strategiesLabels.push_back(child.Read("Strategy", true, "",
            "Sequential Strategy"));
    else if(child.Name() == "node_generation_method") {
      string node_gen_method = child.Read("Method", true, "",
          "Node Generation Method");
      int numPerIteration = child.Read("Number", true, 1, 0, MAX_INT,
          "Number of samples");
      m_vecStrNodeGenLabels.push_back(
          make_pair(node_gen_method, numPerIteration));
    }
    else if(child.Name() == "node_connection_method")
      m_vecStrNodeConnectionLabels.push_back(
          child.Read("Method", true, "", "Node Connection Method"));
    else if(child.Name() == "region_connection_method")
      m_regionConnectionLabels.push_back(
          child.Read("Method", true, "", "Region Connection Method"));
    else if(child.Name() == "num_row")
      m_row = child.Read("nRow", true, 1, 1, MAX_INT,
          "number of partition on x");
    else if(child.Name() == "n_col")
      m_col = child.Read("nCol", true, 1, 1, MAX_INT,
          "number of partition on y");
    else if(child.Name() == "num_runs")
      m_runs = child.Read("nRuns", true, 1, 0, MAX_INT, "Runs number");
    else if(child.Name() == "overlap") {
      // All 3 epsilon values are doubles between 0.0 to 1.0 with default value 0.0.
      m_xEpsilon = child.Read("xeps", true, 0.0, 0.0, 1.0, "x overlap percentage");
      m_yEpsilon = child.Read("yeps", true, 0.0, 0.0, 1.0, "y overlap percentage");
      m_zEpsilon = child.Read("zeps", true, 0.0, 0.0, 1.0, "z overlap percentage");
    }
    else if(child.Name() == "region_connect_k") {
      m_k1 = child.Read("k1", true,
          1, 0, MAX_INT, "K Largest CC from source region");
      m_k2 = child.Read("k2", true,
          1, 0, MAX_INT, "K Largest CC from target region");
      m_nf = child.Read("nf", true,
          "", "Neighborhood Finder for Region Connect");
      m_lp = child.Read("lp", true,
          "", "Local Planner for Region Connect");
      m_ccc = child.Read("type", true,
          "", "CC connection strategy option");
    }
  }
};


template<class MPTraits>
void
RegularSubdivisionMethod<MPTraits>::
Print(ostream& _os) const {
  _os << "RegularSubdivisionMethod:: Print \n";
  _os << "\trows: " << m_row << endl;
  _os << "\tcols: " << m_col << endl;
  _os << "\tregion connector type: " << m_ccc << endl;
  _os << "\tregion k: " << m_k1 << endl;

  typedef vector<pair<string, int> >::const_iterator VIter;
  typedef vector<string>::const_iterator StringIter;

  _os<<"\nSamplers\n";
  for(VIter vIter=m_vecStrNodeGenLabels.begin();
      vIter!=m_vecStrNodeGenLabels.end(); vIter++){
    _os<<"\t"<<vIter->first<<"\tNumber:"<<vIter->second;
  }

  _os<<"\nNodeConnectors\n";
  for(StringIter sIter=m_vecStrNodeConnectionLabels.begin(); sIter!=m_vecStrNodeConnectionLabels.end(); sIter++){
    _os<<"\t"<<*sIter;
  }

  _os<<"\nSequential Strategy\n";
  for(StringIter sIter=m_strategiesLabels.begin(); sIter!=m_strategiesLabels.end(); sIter++){
    _os<<"\t"<<*sIter;
  }
}

template<class MPTraits>
void
RegularSubdivisionMethod<MPTraits>::
Run() {
  cout << "RegularSubdivisionMethod:: Run()" << endl;

  Environment* env = this->GetMPProblem()->GetEnvironment();
  GraphType* rmg = this->GetMPProblem()->GetRoadmap()->GetGraph();
  BasicDecomposition* decomposer = new BasicDecomposition();

  m_ccConnector = new CCsConnector<MPTraits>(m_nf, m_lp);

  ///TIMER STUFF
  stapl::counter<stapl::default_timer> t0;
  double constr_tm=0.0;

  typedef vector<pair<string, int> >::iterator I;
  typedef vector<string>::iterator J;
  typedef CCsConnector<MPTraits>* CCP;
  typedef std::tuple<CCP,string, int> ccConnectParam;
  typedef std::tuple<ConnectorPointer,string, int> ncConnectParam;
  typedef stapl::array< BoundingBox > arrayBbox;
  typedef array_view <arrayBbox> viewBbox;

  size_t mesh_size = m_row * m_col;
  int num_samples;
  CfgType cfg;

  BoundingBox* bbox = dynamic_cast<BoundingBox*>(&*env->GetBoundary());

  arrayBbox pArrayBbox(mesh_size, *bbox);

  ///MAKE A MESH GRAPH
  typedef graph_view<RRGraph> RRGraphView;
  RRGraph regularRegion;
  rmi_fence();
  RRGraphView regionView(regularRegion);
  regionView = stapl::generators::make_mesh<RRGraphView>(regionView,m_row,m_col);
  rmi_fence();

  ////DECOMPOSE SPACE TO REGIONS
  if( stapl::get_location_id() == 0){
    decomposer->DecomposeWS(env,cfg.PosDOF(), m_row, m_col,1, pArrayBbox.begin(), m_xEpsilon,m_yEpsilon, m_zEpsilon);
  }
  rmi_fence();

  viewBbox arrView(pArrayBbox);
  rmi_fence();

  if (this->m_debug)
    t0.start();

  if(this->m_debug)
    PrintValue("View size " , regionView.size());

  if (m_strategiesLabels.size() != 0) {
    for(J itr1 = m_strategiesLabels.begin(); itr1 != m_strategiesLabels.end(); ++itr1) {
      if(this->m_debug) PrintValue("View size " , regionView.size());
      MPStrategyPointer strategy = this->GetMPProblem()->GetMPStrategy(*itr1);
      ConstructRoadmap<MPTraits> constrRegionMap(strategy);
      map_func(constrRegionMap, regionView,arrView);
    }
  }
  else {

    for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr){
      num_samples = itr->second;
      SamplerPointer sp = this->GetMPProblem()->GetSampler(itr->first);
      NodeGenerator<MPTraits> nodeGen(this->GetMPProblem(),sp,num_samples);
      map_func(nodeGen,arrView,regionView);
    }

    for(J itr1 = m_vecStrNodeConnectionLabels.begin();
        itr1 != m_vecStrNodeConnectionLabels.end(); ++itr1) {
      ConnectorPointer cp = this->GetMPProblem()->GetConnector(*itr1);
      NodeConnector<MPTraits> nodeCon(this->GetMPProblem(), cp);
      stapl::for_each(regionView,nodeCon);
    }
  }
  rmi_fence();

  if (this->m_debug) {
    constr_tm = t0.stop();
    PrintValue("CNSTR : " , constr_tm);

    PrintOnce("RUN::# of regions ", regularRegion.num_vertices());
    PrintOnce("RUN::# of region edges: ", regularRegion.num_edges());
    PrintOnce("RUN::roadmap graph size ", rmg->num_vertices());
    PrintOnce("RUN::roadmap graph edges before: ", rmg->num_edges());
  }
  rmi_fence();

  /// COMPUTE CCs AND SET REGION CCs
  typedef graph_view<GraphType> view_type;
  view_type rmView(*rmg);
  rmi_fence();
  /*typedef static_array<cc_color_property> property_storage_type;
    typedef property_storage_type::size_type  size_type;
    typedef graph_external_property_map<view_type, cc_color_property, property_storage_type> property_map_type;

  ///TODO: proper fix by making cc_color_property derived from cfg class
  /// and then use internal_property_map
  size_type mapSz = 2*rmg->num_vertices();
  //property_storage_type prop_storage(2*rmg->num_vertices());
  property_storage_type prop_storage(mapSz);
  property_map_type     map(rmView, &prop_storage);

  connected_components(rmView, map);*/
  connected_components(rmView);
  rmi_fence();
  //std::vector<pair<VID,size_t> > ccVec1;
  //std::vector<pair<VID,size_t> > ccVec1 = cc_stats(rmView,map);
  std::vector<pair<VID,size_t> > ccVec1 = cc_stats(rmView);
  rmi_fence();

  //TODO : Danger, potential to overide region VIDs for fine grained approach, so we check that fine grained is not set
  if (m_strategiesLabels.size() != 0 && (m_vecStrNodeGenLabels.size() == 0
        || m_vecStrNodeConnectionLabels.size() ==0)) {
    array_view<std::vector<pair<VID,size_t> > > ccView1(ccVec1);
    map_func(SetRegionCCVIDS<MPTraits>(), regionView, balance_view(ccView1,regionView.size()));
    rmi_fence();
  }

  if(this->m_debug)
    ///DEBUG
    PrintOnce("cc count before region con:", ccVec1.size());
  rmi_fence();
  if(m_ccc == "largest") {
    array_view<std::vector<pair<VID,size_t> > > ccView1(ccVec1);
    map_func(SetRegionCC<MPTraits>(), regionView, balance_view(ccView1,regionView.size()));
    rmi_fence();

    ///CONNECT REGIONS ROADMAP
    ccConnectParam conParam1 = std::make_tuple(m_ccConnector,m_ccc,m_k1);

    RegionCCConnector<RRGraph, Region<BoundingBox, MPTraits>, MPTraits> regionCCCon(this->GetMPProblem(), &regularRegion, conParam1);
    stapl::for_each(regionView,regionCCCon);
  }
  else if(m_ccc== "random") {
    for(J itr2 = m_regionConnectionLabels.begin(); itr2 != m_regionConnectionLabels.end(); ++itr2) {
      ConnectorPointer ncp = this->GetMPProblem()->GetConnector(*itr2);
      ncConnectParam conParam2 = std::make_tuple(ncp,m_ccc,m_k1);

      RegionRandomConnector<RRGraph, Region<BoundingBox, MPTraits>, MPTraits >
        regionRandomCon(this->GetMPProblem(), &regularRegion, conParam2);
      stapl::for_each(regionView,regionRandomCon);
    }
  }
  else {
    cerr << "ERROR::Please choose an existing k_closest_cc connection type "<< endl;
    cerr << "Reference this error on line "<< __LINE__ << " of file " << __FILE__ << endl;
    exit(-1);
  }

  ///DEBUG
  if(this->m_debug)
    PrintOnce("RUN::roadmap graph edges after: ", rmg->num_edges());
  rmi_fence();
}

template<class MPTraits>
void RegularSubdivisionMethod<MPTraits>::Finalize() {
  stringstream basefname;
  basefname << this->GetBaseFilename() << ".p" << stapl::get_num_locations() << ".it" << m_runs;
  this->GetMPProblem()->GetRoadmap()->Write(basefname.str() + ".map", this->GetMPProblem()->GetEnvironment());
  rmi_fence();
  cout << "location [" << get_location_id() <<"] ALL FINISHED" << endl;
}

#endif
