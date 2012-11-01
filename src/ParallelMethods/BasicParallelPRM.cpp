//////////////////////////////////////
//class BasicParallelPRM.cpp
/////////////////////////////////////

#include "BasicParallelPRM.h"
#include <stapl/views/repeated_view.hpp>

using namespace std;

BasicParallelPRM::BasicParallelPRM(XMLNodeReader& _node, MPProblem* _problem) :
MPStrategyMethod(_node, _problem) {
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
BasicParallelPRM::Run() {
  if (m_debug) cout << "BasicParallelPRM::Run()" << endl;

  RoadmapGraph<CfgType,WeightType>* rmg = GetMPProblem()->GetRoadmap()->m_pRoadmap; 

  stapl::counter<stapl::default_timer> t1,t2;

  double sample_timer=0.0, connect_timer=0.0 ;
  for(int it =1; it<= m_numIterations; ++it) {       
    typedef vector<pair<string, int> >::iterator I;

    //---------------------------
    // Generate roadmap nodes
    //---------------------------

    cout << "Generation Phase" << endl;
    for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr) {
      Sampler<CfgType>::SamplerPointer nodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(itr->first);
      typedef stapl::array<CfgType> cfgArray;
      typedef stapl::array_view<cfgArray> viewCfgArray;
      cfgArray PA(itr->second); 
      viewCfgArray v(PA);

      t1.start();
      SampleWF SampleWF(nodeGen, GetMPProblem());
      stapl::map_func(SampleWF,stapl::balance_view(v,stapl::get_num_locations()));
      sample_timer = t1.stop();

      if (m_debug) 
        cout<<"\n processor #----->["<<stapl::get_location_id()<<"] NodeGeneration time  = "  << sample_timer << endl; 
    }

    //---------------------------
    // Connect roadmap nodes
    //---------------------------
    cout << "Connecting Phase" << endl;
    typedef vector<string>::iterator J;
    for(J itr = m_vecStrNodeConnectionLabels.begin(); itr != m_vecStrNodeConnectionLabels.end(); ++itr) {      
      if (m_debug) cout << "ParallelPRMStrategy::graph size " << rmg->size() << endl;

      Connector<CfgType, WeightType>::ConnectionPointer connector;
      connector = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*itr);
      typedef stapl::graph_view<RoadmapGraph<CfgType,WeightType> >   VType;
      VType g_view(*rmg);
      
      t2.start();
      ConnectWF connWf(connector, GetMPProblem());
      stapl::map_func(connWf, stapl::native_view(g_view), stapl::repeat_view(g_view));
      connect_timer = t2.stop();
      if (m_debug) {
          cout<<"\n processor #["<<stapl::get_location_id()<<"] NodeConnection time  = "  << connect_timer << endl; 
        
      }
    }
  }
}

void 
BasicParallelPRM::Finalize() {
  if (m_debug) cout << "ParallelPRMStrategy::Finalize()";
  //---------------------------
  // Write roadmap to file
  //---------------------------

  if(stapl::get_location_id() == 0) {
    string str;
    str = GetBaseFilename() + ".map";
    ofstream osMap(str.c_str());
    if(!osMap) {
      if (m_debug){
       // cout << "ParallelPRMStrategy::Finalize(): can't open outfile: " << endl;
        
        cerr << "ERROR::Can't open outfile. "<< endl;
        cerr << "Reference this error on line "<< __LINE__ << " of file " << __FILE__ << endl;
      }
      exit(-1);

    }else {
      GetMPProblem()->WriteRoadmapForVizmo(osMap);
      osMap.close();
    }
  }
  if (m_debug) cout << "!!ALL FINISHED!!"<< endl;
}




