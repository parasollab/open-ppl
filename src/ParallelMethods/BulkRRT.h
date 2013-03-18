//////////////////////////////////
//HEADER BulkRRT.h
/////////////////////////////////

#ifndef BULKRRT_H_
#define BULKRRT_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "ParallelMethods/ParallelSBMPHeader.h"
#include "MPStrategies/BasicRRTStrategy.h"
//#include "BulkNFRRT.h"
#include <stapl/containers/graph/views/graph_view.hpp>


using namespace psbmp;
using namespace stapl;



class XMLNodeReader;

 
template<class MPTraits>
class BulkWF {

  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::GraphType GraphType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
  typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
  private:
  
  MPProblemType* m_problem;
  string m_dm;
  string m_vcm;
  double m_delta;
  double m_minDist;
  
  public:
  BulkWF(MPProblemType* _problem, string _dm, string _vcm, 
    double _delta, double _minDist) {
    
    m_problem = _problem;
    m_dm = _dm;
    m_delta = _delta;
    m_vcm = _vcm;
    m_minDist = _minDist;
  }
  
  void define_type(stapl::typer& _t) {
      _t.member(m_problem);
      _t.member(m_dm);
      _t.member(m_delta);
      _t.member(m_minDist);
  }

  template<typename View, typename GraphView> 
  void operator()(const View& _view, GraphView& _gview){  
    
    ///Setup global variables
    DistanceMetricPointer dmm = m_problem->GetDistanceMetric("scaledEuclidean");
    Environment* env = m_problem->GetEnvironment();
    NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder("BruteForce");
      for(typename View::iterator vit = _view.begin(); vit != _view.end(); ++vit){  
            
        CfgType newCfg, nearest, dir;
        VID nearestVID;
        CDInfo cdInfo;
        int weight;  
        vector<VID> kClosest(1);
        
        //We want to find a random configuration to attempt expansion that way.
        //CfgType dir = SelectDirection(env);
        dir.GetRandomCfg(env);
         
        nf->KClosest(m_problem->GetRoadmap(), dir, 1, back_inserter(kClosest));
        /*NFMapFunc nfMap;
        NFResultType nfresult = nfMap.FindNeighbors(env, dmm, _gview.begin(), _gview.end(), dir, 1);
        nearestVID = nfresult[0].first;*/
	//nearest = (*(_gview.find_vertex(nearestVID))).property();
	nearest = (*(_gview.find_vertex(kClosest[0]))).property();
	
    //if(this->m_debug) cout << "RRT could not expand!" << endl; 
        if((nearestVID != -999) && RRTExpand<MPTraits>(m_problem, m_vcm, m_dm, nearest, dir, newCfg, m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())
		    && (dmm->Distance(env, newCfg, nearest) >= m_minDist))   { 
            //do not change the root       
            if((*vit).descriptor() != 0) (*vit).property() = newCfg;
            int tmpweight=2.0;
            pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", tmpweight), WeightType("RRTExpand", tmpweight));
            typename GraphType::edge_descriptor ed1(nearestVID,(*vit).descriptor());
            typename GraphType::edge_descriptor ed2((*vit).descriptor(),nearestVID);
            _gview.add_edge_async(ed1);

        }

      }
    }
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
  void InitializeTree();
  virtual void Finalize();
  virtual void PrintOptions(ostream& _os);
    
    
  private:
  ///////////////////////////////////////////////
  //general variables
  ///////////////////////////////////////////////
  int m_runs;
  int m_totalNodes;
  ///////////////////////////////////////////////
  //variables for ExtendTree method
  ///////////////////////////////////////////////
  int m_kNodes;
  string m_vcMethod;
  double m_minDist;
  string m_dm;
  double m_delta;
  int m_weight;
  bool m_parallelNF;

};

template<class MPTraits>
BulkRRT<MPTraits>::BulkRRT(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MPStrategyMethod<MPTraits>(_problem, _node) {
  ParseXML(_node);
  this->SetName("BulkRRT");
}

template<class MPTraits>
BulkRRT<MPTraits>::BulkRRT()
{ 
this->SetName("BulkRRT");
}


template<class MPTraits>
BulkRRT<MPTraits>::~BulkRRT() { }

template<class MPTraits>
void BulkRRT<MPTraits>::ParseXML(XMLNodeReader& _node){
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "num_runs") {
      m_runs = citr->numberXMLParameter(string("nRuns"), true, 
        int(1), int(0), MAX_INT, string("Runs number"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "total_nodes") {
      m_totalNodes = citr->numberXMLParameter(string("totalNodes"), true, 
        int(1), int(0), MAX_INT, string("Total number of nodes to be created"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "k_nodes") {
      m_kNodes = citr->numberXMLParameter(string("kNodes"), true, 
        int(1), int(0), MAX_INT, string("Number of new nodes made before upgrading tree"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "vc_method") {
      m_vcMethod = citr->stringXMLParameter("vcm", true,
	"", "Validity Checker Method");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "dm_method") {
      m_dm = citr->stringXMLParameter("Method", true,
	"", "Distance Metric method");
      citr->warnUnrequestedAttributes();
    }else if (citr->getName() == "parallel_nf"){
      m_parallelNF = citr->boolXMLParameter("parallelNF",true,true,"If true, parallelize NF");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "delta_var") {
      m_delta = citr->numberXMLParameter("delta", true, 0.0, 0.0, MAX_DBL, "Delta Variable for ExtendTree method");
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "min_distance") {
      m_minDist = citr->numberXMLParameter("minDist", true, 0.0, 0.0, MAX_DBL, "Minimum Distance to see if new node is too close to closet cfg");
      citr->warnUnrequestedAttributes();
    }else {
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
  StatClass* stat = m_problem->GetStatClass();
  Environment* env = m_problem->GetEnvironment();
  ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vcMethod);
  string callee = "BulkRRT::";  
  CDInfo cdInfo;
  //GraphType pMap(m_totalNodes+1);
  GraphType pMap;
  gviewType rmView(pMap);
  
  if(stapl::get_location_id() == 0){ 
    
    //keep looping until a valid root node is made
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
    }

  }
  stapl::rmi_fence(); 
  

  //WORK
  BulkWF<MPTraits> wf(m_problem, m_dm, m_vcMethod, m_delta, m_minDist); 
  int partition = m_totalNodes/m_kNodes;
  stapl::map_func(wf,stapl::balance_view(rmView,partition),repeat_view(rmView));
  stapl::rmi_fence(); //do I really need to fence?
  //t1.stop();
  
  //STATS
  /*PrintValue("DIR TIME: ", t2.value());
  PrintValue("NN TIME : ", t3.value());
  PrintValue("EXPAND TIME : ", t4.value());
  PrintOnce("TOTAL TIME: ", t1.value());*/
  PrintOnce("Graph size: ", rmView.size());
  stapl::rmi_fence();
}

template<class MPTraits>
void BulkRRT<MPTraits>::Finalize(){
  ///Write graph here :: DEBUG

 /* string str;
  stringstream basefname;
  MPProblem* m_problem = GetMPProblem();

  
  basefname << GetBaseFilename() << ".p" << stapl::get_num_locations() << ".it" << m_runs <<".m"<<m_kNodes;
  ofstream osMap((basefname.str() + ".map").c_str());
  if(!osMap){
     cout << "BulkRRt::Finalize(): can't open outfile: ";
     exit(-1);
  }else{
    m_problem->GetMPRegion(_regionID)->WriteRoadmapForVizmo(osMap);
    osMap.close();
  }
  stapl::rmi_fence();*/
  cout << "location [" << stapl::get_location_id() <<"] ALL FINISHED" << endl;
  stapl::rmi_fence();
}

template<class MPTraits>
void BulkRRT<MPTraits>::PrintOptions(ostream& _os){
   _os << "BulkRRT:: PrintOptions \n";
}

#endif 
