#ifndef BULK_RRT_H_
#define BULK_RRT_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "ParallelMethods/ParallelSBMPHeader.h"
#include "ParallelMethods/WorkFunctions/MapReduceNF.h"

using namespace psbmp;
using namespace stapl;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class BulkWF {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;
    typedef pair<pair<VID,CfgType>, double> NFType;
    typedef vector<NFType> NFResultType;

    typedef void result_type;

    BulkWF(MPProblemType* _problem, string _dm, string _eLabel, double _minDist) {
      m_problem = _problem;
      m_dm = _dm;
      m_minDist = _minDist;
    }

    void define_type(stapl::typer& _t) {
      _t.member(m_problem);
      _t.member(m_dm);
      _t.member(m_eLabel);
      _t.member(m_minDist);
    }

    template<typename View, typename GraphView>
      result_type operator()(const View& _view, GraphView& _gview){

        ///Setup global variables
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric(m_dm);
        Environment* env = m_problem->GetEnvironment();
        NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder("BFNF");
        ExtenderPointer e = m_problem->GetExtender(m_eLabel);

        for(typename View::iterator vit = _view.begin(); vit != _view.end(); ++vit) {

          CfgType newCfg, nearest, dir;
          VID nearestVID;
          //int weight;

          //We want to find a random configuration to attempt expansion that way.
          dir.GetRandomCfg(env);

          NFMapFunc<MPTraits> nfMap(m_problem, dir, 1, m_dm);
          NFReduceFunc<MPTraits> nfReduce(1);
          NFResultType nfresult = map_reduce<skeletons::tags::with_coarsened_wf>(nfMap, nfReduce, _gview);
          nearestVID = nfresult[0].first.first;
          nearest = nfresult[0].first.second;

          LPOutput<MPTraits> lpOut;
          if(nearestVID != -999 &&
              e->Extend(nearest, dir, newCfg, lpOut) &&
              dmm->Distance(newCfg, nearest) >= m_minDist)   {

            VID newVID = _gview.add_vertex(newCfg);

            typename GraphType::edge_descriptor ed1(nearestVID, newVID);
            typename GraphType::edge_descriptor ed2(newVID, nearestVID);
            _gview.add_edge_async(ed1);
            _gview.add_edge_async(ed2);
          }
          //cout << "VECTOR VID size " : << kClosest.size() << endl;
        }
      }

  private:
    MPProblemType* m_problem;
    string m_dm;
    string m_eLabel;
    double m_minDist;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class BulkRRT : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::VID VID;
    typedef graph_view<GraphType>  gviewType;

    BulkRRT(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    BulkRRT();
    virtual ~BulkRRT();

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;


  private:
    //Query<MPTraits>* m_query;
    string m_eLabel;
    int m_kNodes;
    string m_vcMethod;
    double m_minDist;
    string m_dm;
};

template<class MPTraits>
BulkRRT<MPTraits>::
BulkRRT() {
  this->SetName("BulkRRT");
}

template<class MPTraits>
BulkRRT<MPTraits>::
BulkRRT(typename MPTraits::MPProblemType* _problem, XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    ParseXML(_node);
    this->SetName("BulkRRT");
  }

template<class MPTraits>
BulkRRT<MPTraits>::
~BulkRRT() {
  /*if(m_query != NULL)
    delete m_query;*/
}

template<class MPTraits>
void
BulkRRT<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node) {
    if(child.Name() == "k_nodes")
      m_kNodes = child.Read("kNodes", true, 1, 0, MAX_INT,
          "Number of new nodes made before upgrading tree");
    else if(child.Name() == "vc_method")
      m_vcMethod = child.Read("vcm", true, "", "Validity Checker Method");
    else if(child.Name() == "dm_method")
      m_dm = child.Read("Method", true, "", "Distance Metric method");
    else if(child.Name() == "e_method")
      m_eLabel = child.Read("Method", true, "", "Extender method");
    else if(child.Name() == "min_distance")
      m_minDist = child.Read("minDist", true, 0.0, 0.0, MAX_DBL,
          "Minimum Distance to see if new node is too close to closet cfg");
    else if(child.Name() == "evaluation_method")
      this->m_meLabels.push_back(child.Read("Method", true, "",
          "Evaluation Method"));
    /*
    else if(child.Name() == "query") {
      //optionally read in a query and create a Query object.
      string query = child.Read("query", false, "", "Query Filename");
      if(query != ""){
        m_query = new Query<MPTraits>(query);
        m_query->SetMPProblem(this->GetMPProblem());
        m_query->SetDebug(this->m_debug);
      }
    }
    */

  }
};

template<class MPTraits>
void BulkRRT<MPTraits>::Initialize() {
  cout << "BulkRRT::Initialize()" <<endl;
}

template<class MPTraits>
void BulkRRT<MPTraits>::Run() {

  cout << "BulkRRT:: Run()" << endl;

  //Set up variables needed for BulkRRT
  MPProblemType* m_problem = this->GetMPProblem();

  GraphType* pMap = m_problem->GetRoadmap()->GetGraph();
  CfgType root;

  ///TODO When Empty Base Container bug is fixed in STAPL do this to add root on
  ///only one location. Until then, do code below comment section.
  //if(stapl::get_location_id() == 0){
    //keep looping until a valid root node is made
    /*
       StatClass* stat = m_problem->GetStatClass();
       Environment* env = m_problem->GetEnvironment();
       ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vcMethod);
       string callee = "BulkRRT::";
       CDInfo cdInfo;
       int condition(0);
       while(condition== 0){

       CfgType root;
       root.GetRandomCfg(env);

       if (root.InBoundary(env) &&
       vc->IsValid(root, env, *stat, cdInfo, &callee)){
       PrintOnce("ROOT " , root);
       pMap[0].property() = root;
       condition = 1;
       break;

       }
       }*/
  //  cout << "k: " << m_kNodes << endl;
  //  pMap->add_vertex(root);
  //}
  ///temporary fix to Empty Base Container bug
  stapl::array<VID> roots(get_num_locations());
  size_t myID = get_location_id();
  VID vid = pMap->add_vertex(root);
  roots[myID] = vid;
  PrintValue("VID ", vid);
  PrintOnce("ROOT " , root);
  stapl::rmi_fence();
  if(myID < get_num_locations() - 1) {
    pMap->add_edge_async(roots[myID], roots[myID + 1]);
    pMap->add_edge_async(roots[myID + 1], roots[myID]);
  }
  else {
    pMap->add_edge_async(roots[myID], roots[0]);
    pMap->add_edge_async(roots[0], roots[myID]);
  }


  stapl::rmi_fence();

  //WORK
  BulkWF<MPTraits> wf(m_problem, m_dm, m_eLabel, m_minDist);
  typedef stapl::array<VID> vidArray;
  typedef stapl::array_view<vidArray> viewVidArray;
  stapl::counter<stapl::default_timer> t1;
  t1.start();

  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    vidArray PA(m_kNodes*get_num_locations());
    viewVidArray v(PA);
    gviewType rmView(*pMap);
    map_func(wf, balance_view(v,get_num_locations()), make_repeat_view(rmView));
    mapPassedEvaluation = this->EvaluateMap();
  }

  stapl::rmi_fence(); //do I really need to fence?

  t1.stop();

  //STATS
  PrintOnce("TOTAL TIME: ", t1.value());
  PrintOnce("Graph size: ", pMap->get_num_vertices());
  stapl::rmi_fence();
}

template<class MPTraits>
void
BulkRRT<MPTraits>::
Finalize() {
  stringstream basefname;
  //basefname << this->GetBaseFilename() << ".N" << this->m_numNodes << ".R" << this->m_numRegions << ".p" << get_num_locations() ;
  basefname << this->GetBaseFilename() ;
  this->GetRoadmap()->Write(basefname.str() + ".map", this->GetEnvironment());
  stapl::rmi_fence();
  cout << "location [" << stapl::get_location_id() <<"] ALL FINISHED" << endl;
}

template<class MPTraits>
void
BulkRRT<MPTraits>::
Print(ostream& _os) const {
  _os << "BulkRRT:: PrintOptions \n";
}

#endif
