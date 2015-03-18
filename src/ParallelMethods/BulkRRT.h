#ifndef BULK_RRT_H_
#define BULK_RRT_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "ParallelMethods/ParallelSBMPHeader.h"
#include "ParallelMethods/WorkFunctions/MapReduceNF.h"

using namespace psbmp;
using namespace stapl;

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

        for(typename View::iterator vit = _view.begin(); vit != _view.end(); ++vit){

          CfgType newCfg, nearest, dir;
          VID nearestVID;
          //int weight;

          //We want to find a random configuration to attempt expansion that way.
          //CfgType dir = SelectDirection(env);
          dir.GetRandomCfg(env);

          NFMapFunc<MPTraits> nfMap;
          NFResultType nfresult = nfMap.FindNeighbor(env, dmm, _gview.begin(), _gview.end(), dir, 1);
          nearestVID = nfresult[0].first.first;
          nearest = nfresult[0].first.second;

          vector<CfgType> inner;
          if(nearestVID != -999 &&
              e->Extend(nearest, dir, newCfg, inner) &&
              dmm->Distance(newCfg, nearest) >= m_minDist)   {

            VID newVID = _gview.add_vertex(newCfg);

            int tmpweight=2.0;
            pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", tmpweight), WeightType("RRTExpand", tmpweight));

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

    BulkRRT(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    BulkRRT();
    virtual ~BulkRRT();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;


  private:
    vector<string> m_evaluatorLabels;
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
BulkRRT(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    ParseXML(_node);
    this->SetName("BulkRRT");
  }

template<class MPTraits>
BulkRRT<MPTraits>::~BulkRRT() {
  /*if(m_query != NULL)
    delete m_query;*/
}

template<class MPTraits>
void BulkRRT<MPTraits>::ParseXML(XMLNodeReader& _node){
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "k_nodes") {
      m_kNodes = citr->numberXMLParameter(string("kNodes"), true,
          int(1), int(0), MAX_INT, string("Number of new nodes made before upgrading tree"));
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "vc_method") {
      m_vcMethod = citr->stringXMLParameter("vcm", true,
          "", "Validity Checker Method");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "dm_method") {
      m_dm = citr->stringXMLParameter("Method", true,
          "", "Distance Metric method");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "e_method") {
      m_eLabel = citr->stringXMLParameter("Method", true,
          "", "Distance Metric method");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "min_distance") {
      m_minDist = citr->numberXMLParameter("minDist", true, 0.0, 0.0, MAX_DBL, "Minimum Distance to see if new node is too close to closet cfg");
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "",
          "Evaluation Method");
      m_evaluatorLabels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    /*else if(citr->getName() == "query"){
    //optionally read in a query and create a Query object.
    string query = citr->stringXMLParameter("query", false, "", "Query Filename");
    if(query != ""){
    m_query = new Query<MPTraits>(query);
    m_query->SetMPProblem(this->GetMPProblem());
    m_query->SetDebug(this->m_debug);
    citr->warnUnrequestedAttributes();
    }*/
    else {
      citr->warnUnknownNode();
    }

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
  gviewType rmView(*pMap);
  CfgType root;


  if(stapl::get_location_id() == 0){
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
    pMap->add_vertex(root);
  }

  PrintValue("ROOT " , root);
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
    map_func(wf, balance_view(v,get_num_locations()), make_repeat_view(rmView));
    mapPassedEvaluation = this->EvaluateMap(m_evaluatorLabels);
  }
  stapl::rmi_fence(); //do I really need to fence?

  t1.stop();

  //STATS
  PrintOnce("TOTAL TIME: ", t1.value());
  PrintOnce("Graph size: ", rmView.size());
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
