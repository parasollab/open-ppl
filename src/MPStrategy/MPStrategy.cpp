#include "MPStrategy.h"

#ifdef _PARALLEL
#include "ParallelPRMStrategy.h"
#else
#include "MPRegionComparerMethod.h"
#include "BasicPRM.h"
#include "BasicRRTStrategy.h"
#include "MARRTStrategy.h"
#include "OBRRTStrategy.h"
#include "TogglePRMStrategy.h"
#include "RoadmapToolsStrategy.h"
#include "HybridPRM.h"
#include "ExpanderStats.h"
#include "TimingStats.h"
#include "NFComparer.h"
#include "BandsStrategy.h"
#include "QueryStrategy.h"
#include "SmoothQueryStrategy.h"
#include "ResamplePointStrategy.h"
#include "EvaluateMapStrategy.h"
#include "UAStrategy.h"
#endif

#include "Sampler.h"
//#include "LocalPlanners.h"

MPStrategy::
MPStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml) : MPBaseObject(in_Node,in_pProblem) {
  ///\todo Find a home for "addPartialEdge" and "addAllEdges" or remove all together
  addPartialEdge=true;
  addAllEdges=false;

  if(parse_xml) {
    in_Node.verifyName("MPStrategy");

    m_pNodeGeneration = NULL;
    m_pConnection = NULL;
    m_pLocalPlanners = NULL;
#ifndef _PARALLEL
    m_Evaluator = NULL;
    m_pCharacterizer = NULL;
#endif 
    XMLNodeReader::childiterator citr;
    cout << "input node label MP Strategy = " << in_Node.getName() << endl;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(citr->getName() == "node_generation_methods") {
        // m_pNodeGeneration = new GenerateMapNodes<CfgType>(*citr, GetMPProblem());
        m_pNodeGeneration = new Sampler<CfgType>(*citr, GetMPProblem());
      } else if(citr->getName() == "connection_methods") {
        m_pConnection = new ConnectMap<CfgType, WeightType>(*citr, GetMPProblem());
      } else if(citr->getName() == "lp_methods") {
        m_pLocalPlanners = new LocalPlanners<CfgType, WeightType>(*citr, GetMPProblem());
      } else if(citr->getName() == "MPStrategyMethod") {
        ParseStrategyMethod(*citr);
#ifndef _PARALLEL
      } else if (citr->getName() == "MPEvaluator_methods") {
        m_Evaluator = new MapEvaluator<CfgType, WeightType>(*citr, GetMPProblem());
        /*} else if(citr->getName() == "MPStrategyMethod") {
          ParseStrategyMethod(*citr);*/
    } else if(citr->getName() == "MPCharacterizer") {
      m_pCharacterizer = new MPCharacterizer<CfgType, WeightType>(*citr, GetMPProblem());
    } 
    else if(citr->getName() == "features"){
      m_Features = new Features(*citr, GetMPProblem());
    }
    else if(citr->getName() == "partitioning_methods"){
      m_PartitioningMethods = new PartitioningMethods(*citr, GetMPProblem());
    }
    else if(citr->getName() == "partitioning_evaluators"){
      m_PartitioningEvaluators = new PartitioningEvaluators(*citr, GetMPProblem());
#endif
    }
    else {
      citr->warnUnknownNode();
    }
    }
  }
}


void MPStrategy::
PrintOptions(ostream& out_os)
{
  out_os << "MPStrategy" << endl;
  if(m_pNodeGeneration)
    m_pNodeGeneration->PrintOptions(out_os);
  if(m_pConnection)
    m_pConnection->PrintOptions(out_os);
  if(m_pLocalPlanners)
    m_pLocalPlanners->PrintOptions(out_os);
#ifndef _PARALLEL
  if (m_Evaluator)
    m_Evaluator->PrintOptions(out_os);
#endif
}

void MPStrategy::
ParseStrategyMethod(XMLNodeReader& in_Node) {
  in_Node.verifyName(string("MPStrategyMethod"));

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    pair<MPStrategyMethod*, XMLNodeReader*> mpsm(CreateMPStrategyMethod(*citr), &*citr);
    all_MPStrategyMethod.push_back(mpsm);
  }

  m_strController_MPStrategyMethod = in_Node.stringXMLParameter("Controller",true,"","Controller Method");
}

MPStrategyMethod* MPStrategy::CreateMPStrategyMethod(XMLNodeReader& citr){
  MPStrategyMethod* mpsm = NULL;
#ifdef _PARALLEL
  if(citr.getName() == "ParallelPRMRoadmap"){
    mpsm = new ParallelPRMRoadmap(citr, GetMPProblem());
  }
#else
  if(citr.getName() == "BasicPRM"){
    mpsm = new BasicPRM(citr, GetMPProblem());
  } else if(citr.getName() == "BasicRRTStrategy"){
    mpsm = new BasicRRTStrategy(citr, GetMPProblem()); 
  }  else if(citr.getName() == "MARRTStrategy"){
    mpsm = new MARRTStrategy(citr, GetMPProblem());
  }  else if(citr.getName() == "OBRRTStrategy"){
    mpsm = new OBRRTStrategy(citr, GetMPProblem());
  }   
  else if(citr.getName() == "TogglePRMStrategy") {
    mpsm = new TogglePRMStrategy(citr,GetMPProblem());
  } else if(citr.getName() == "Compare") {
    mpsm = new MPComparer(citr,GetMPProblem());
  } else if(citr.getName() == "RoadmapClear") {
    mpsm = new RoadmapClear(citr,GetMPProblem());
  } else if(citr.getName() == "RoadmapInput") {
    mpsm = new RoadmapInput(citr,GetMPProblem());
  } else if(citr.getName() == "MPMultiStrategy") {
    mpsm = new MPMultiStrategy(citr,GetMPProblem());
  } else if(citr.getName() == "HybridPRM") {
    mpsm = new HybridPRM(citr,GetMPProblem());
  } else if(citr.getName() == "NFUnionRoadmap") {
    mpsm = new NFUnionRoadmap(citr,GetMPProblem());
  } else if(citr.getName() == "NFRoadmapCompare") {
    mpsm = new NFRoadmapCompare(citr,GetMPProblem());
  } else if(citr.getName() == "ExpanderStats") {
    mpsm = new EdgeExpanderStats(citr,GetMPProblem());
  } else if(citr.getName() == "TimingStats") {
    mpsm = new RoadmapTimingStats(citr,GetMPProblem());
  } else if(citr.getName() == "BandsIncrementalRoadmap") {
    mpsm = new BandsIncrementalRoadmap(citr, GetMPProblem());
  } else if(citr.getName() == "BandsStats") {
    mpsm = new BandsStats(citr, GetMPProblem());
  } else if(citr.getName() == "QueryStrategy") {
    mpsm = new QueryStrategy(citr,GetMPProblem());
  } else if(citr.getName() == "SmoothQueryStrategy") {
    mpsm = new SmoothQueryStrategy(citr,GetMPProblem());
  } else if(citr.getName() == "EvaluateMapStrategy") {
    mpsm = new EvaluateMapStrategy(citr,GetMPProblem());
  } else if(citr.getName() == "ResamplePointStrategy") {
    mpsm = new ResamplePointStrategy(citr,GetMPProblem());
  } else if(citr.getName() == "UAStrategy") {
    mpsm = new UAStrategy(citr, GetMPProblem());
  } 
#endif
  else {
    citr.warnUnknownNode();
  }
  return mpsm;
}

MPStrategyMethod* MPStrategy::
GetMPStrategyMethod(string& in_strLabel) {
  vector<pair<MPStrategyMethod*, XMLNodeReader*> >::iterator I;
  for(I = all_MPStrategyMethod.begin(); 
      I != all_MPStrategyMethod.end(); ++I) {
    if(I->first->GetLabel() == in_strLabel) {
      return I->first;
    }
  }
  return NULL;
}

XMLNodeReader* MPStrategy::
GetXMLNodeForStrategy(string& in_strLabel) {
  vector<pair<MPStrategyMethod*, XMLNodeReader*> >::iterator I;
  for(I = all_MPStrategyMethod.begin(); I != all_MPStrategyMethod.end(); ++I) {
    if(I->first->GetLabel() == in_strLabel) {
      return I->second;
    }
  }
  return NULL;
}

void MPStrategy::
Solve() {
  // (*(GetMPStrategyMethod(m_strController_MPStrategyMethod)))();
  cout<<m_strController_MPStrategyMethod<<endl;
  if(GetMPStrategyMethod(m_strController_MPStrategyMethod)==NULL){
    cout<<"ISNULL"<<flush;
  }
  GetMPStrategyMethod(m_strController_MPStrategyMethod)->operator()();
};



MPComparer::
MPComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node,in_pProblem) {
    ParseXML(in_Node);    
  }

void 
MPComparer::
PrintOptions(ostream& out_os) { 
}

void 
MPComparer::
ParseXML(XMLNodeReader& in_Node) {

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "input") {
      string strategy = citr->stringXMLParameter("Method",true,"","Method");
      m_input_methods.push_back(strategy);    
    } else if (citr->getName() == "comparer_method") {
      string evaluator_method = citr->stringXMLParameter("Method",true,"","Method");
      m_comparer_methods.push_back(evaluator_method);
    } else {
      citr->warnUnknownNode();
    }
  }
}

void 
MPComparer::
operator()(int in_RegionID) { 
}

// @todo make the parameter be a vector<int> in_region_ids and loop through it to map regions

void 
MPComparer::
operator()(int in_RegionID_1, int in_RegionID_2) { 

#ifndef _PARALLEL
  // mapping region 1
  MPStrategyMethod* input1 = GetMPProblem()->GetMPStrategy()->
    GetMPStrategyMethod(m_input_methods[0]);
  (*input1)(in_RegionID_1);

  // mapping region 2
  MPStrategyMethod* input2 = GetMPProblem()->GetMPStrategy()->
    GetMPStrategyMethod(m_input_methods[1]);
  (*input2)(in_RegionID_2); 

  // comparing region 1 to region 2 with each comparer
  typedef vector<string>::iterator Itrtr;
  for (Itrtr itrtr = m_comparer_methods.begin(); itrtr < m_comparer_methods.end(); itrtr++) {
    GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetComparerMethod(*itrtr)->Compare(in_RegionID_1, in_RegionID_2);
  } 
#endif
}


void 
MPComparer::
operator()() {
  int Input1RegionId = GetMPProblem()->CreateMPRegion();
  int Input2RegionId = GetMPProblem()->CreateMPRegion();

  (*this)(Input1RegionId, Input2RegionId);
}


MPMultiStrategy::
MPMultiStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node,in_pProblem) {
    ParseXML(in_Node);    
  }

void 
MPMultiStrategy::
PrintOptions(ostream& out_os) { 
}

void 
MPMultiStrategy::
ParseXML(XMLNodeReader& in_Node) {
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if (citr->getName() == "strategy") {
      string strategy = citr->stringXMLParameter("Method",true,"","Method");
      m_strategy_methods.push_back(strategy);
    } else {
      citr->warnUnknownNode();
    }
  }
}

void 
MPMultiStrategy::
operator()(int in_RegionID) { 
  // initializing region from input
  typedef vector< string >::iterator VITRTR;
  for (VITRTR s_itrtr = m_strategy_methods.begin(); s_itrtr < m_strategy_methods.end(); s_itrtr++) { 
    MPStrategyMethod* strategy = GetMPProblem()->GetMPStrategy()->
      GetMPStrategyMethod(*s_itrtr);
    (*strategy)(in_RegionID);
  }
}

void 
MPMultiStrategy::
operator()() {
  int RegionId = GetMPProblem()->CreateMPRegion();    
  (*this)(RegionId);
}

