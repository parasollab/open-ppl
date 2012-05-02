/*
#include "Roadmap.h"



#include "MetricUtils.h"
#include "CollisionDetection.h"
#include "Connector.h"
#include "LocalPlanners.h"
#include "Sampler.h"
#include "MPProblem.h"
#include "MPStrategy/MPStrategyMethod.h"
#include "runtime.h"
#include "runtime/counter/default_counters.h"
#include "pRange.h"
#include "p_array.h"
#include "views/view_base.hpp"
#include "views/balance_view.hpp"
#include "RoadmapGraph.h"
#include "pContainers/pgraph/view/p_graph_view.h"
#include "views/native_view.hpp"
#include "views/replicated_view.hpp"
*/

#include "BasicParallelPRM.h"
#include <stapl/views/repeated_view.hpp>
using namespace std;


BasicParallelPRM::BasicParallelPRM(XMLNodeReader& _node, MPProblem* in_pProblem) :
  MPStrategyMethod(_node,in_pProblem) {
    if (m_debug) cout << "BasicParallelPRM::BasicParallelPRM" << endl;
    ParseXML(_node);    
    this->SetLabel("BasicParallelPRM");
  }

void 
BasicParallelPRM::ParseXML(XMLNodeReader& _node) {
  if (m_debug) cout << "BasicParallelPRM::ParseXML" << endl;

  m_numIterations = _node.numberXMLParameter("iterations", true, int(1), int(0), MAX_INT, "iterations of strategy");
  
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "node_generation_method") {
      string node_gen_method = citr->stringXMLParameter(string("Method"), true,
          string(""), string("Node Generation Method"));
      int numPerIteration = citr->numberXMLParameter(string("Number"), true, 
          int(1), int(0), MAX_INT, string("Number of samples"));
      m_vecStrNodeGenLabels.push_back(pair<string, int>(node_gen_method, numPerIteration));
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "node_connection_method") {
      string connect_method = citr->stringXMLParameter(string("Method"), true,
          string(""), string("Node Connection Method"));
      m_vecStrNodeConnectionLabels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "node_connection_method") {
      string connect_method = citr->stringXMLParameter(string("Method"), true,
          string(""), string("Node Connection Method"));
      m_vecStrNodeConnectionLabels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    } else {

      citr->warnUnknownNode();
    }
  }



  if (m_debug) cout << "BasicParallelPRM::ParseXML()" << endl;

}

void 
BasicParallelPRM::Run(int in_RegionID) {
  if (m_debug) cout << "BasicParallelPRM::Run()" << endl;
  region = GetMPProblem()->GetMPRegion(in_RegionID);
  pStatClass = region->GetStatClass();
  CollisionDetection *  pCd = GetMPProblem()->GetCollisionDetection();
  Environment * pEnv = GetMPProblem()->GetEnvironment();
  LocalPlanners<CfgType, WeightType>* pLp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();

  RoadmapGraph<CfgType,WeightType> * rmg = region->GetRoadmap()->m_pRoadmap; 


  stapl::counter<stapl::default_timer> t1,t2;

  double sample_timer=0.0, connect_timer=0.0 ;
  for(int it =1; it<= m_numIterations; ++it)
  {       

    typedef vector<pair<string, int> >::iterator I;

    //---------------------------
    // Generate roadmap nodes
    //---------------------------

    cout << "Generation Phase" << endl;
    for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
    {
 
      Sampler<CfgType>::SamplerPointer pNodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(itr->first);
      typedef stapl::array<CfgType> cfgArray;
      typedef stapl::array_view<cfgArray> viewCfgArray;
      cfgArray PA(itr->second); // ########
      //####stapl::array_1D_view<stapl::p_array<CfgType> > v(PA);
      viewCfgArray v(PA);

      t1.start();
      p_sample(v,pNodeGen,region,pEnv);
      cout << "P_sample" << endl;
      sample_timer = t1.stop();

      if (m_debug) 
        cout<<"\n processor #----->["<<stapl::get_location_id()<<"] NodeGeneration time  = "  << sample_timer << endl; 
    }

    //---------------------------
    // Connect roadmap nodes
    //---------------------------
    cout << "Connecting Phase" << endl;
    typedef vector<string>::iterator J;
    for(J itr = m_vecStrNodeConnectionLabels.begin(); itr != m_vecStrNodeConnectionLabels.end(); ++itr)
    {      
      if (m_debug) cout << "ParallelPRMStrategy::graph size " << rmg->size() << endl;

      Connector<CfgType, WeightType>::ConnectionPointer pConnection;
      pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*itr);
      typedef stapl::graph_view<RoadmapGraph<CfgType,WeightType> >   VType;
      VType g_view(*rmg);
      
      //stapl::repeat_view<stapl::replicated_container<VType> > voverlap = stapl::repeat_view(g_view);
      //stapl::repeat_view<stapl::replicated_container<VType> > voverlap = stapl::repeat_view(g_view);

      t2.start();
      //p_connect(vnative,voverlap,pConnection,region,pLp);
      connect_wf connWf(pConnection,region,pLp);
      stapl::map_func(connWf, stapl::native_view(g_view), stapl::repeat_view(g_view));
      connect_timer = t2.stop();
      if (m_debug) {
        //for (VI vi=region->GetRoadmap()->m_pRoadmap->begin(); vi!= region->GetRoadmap()->m_pRoadmap->end(); ++vi){  
          //cout<<"\n processor #["<<stapl::get_location_id()<<"] NodeConnection time  = "  << connect_timer << endl; 
        
      }
    }
  }
  
}

void 
BasicParallelPRM::Finalize(int in_RegionID){

  if (m_debug) cout << "ParallelPRMStrategy::Finalize()";
  //---------------------------
  // Write roadmap to file
  //---------------------------

  if( stapl::get_location_id() == 0){
    string str;
    str = GetBaseFilename() + ".map";
    ofstream osMap(str.c_str());
    if(!osMap){
      if (m_debug) cout << "ParallelPRMStrategy::Finalize(): can't open outfile: " << endl;
      exit(-1);
    }else{
      region->WriteRoadmapForVizmo(osMap);
      osMap.close();
    }
  }
  if (m_debug) cout << "!!ALL FINISHED!!"<< endl;
}




