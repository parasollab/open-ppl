#include "MPStrategy.h"
#include "MPRegionComparerMethod.h"

MPStrategy::
MPStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) : MPBaseObject(in_Node,in_pProblem) {
  LOG_DEBUG_MSG( "MPStrategy::MPStrategy()");
  ///\todo Find a home for "addPartialEdge" and "addAllEdges" or remove all together
  addPartialEdge=true;
  addAllEdges=false;
  
  in_Node.verifyName("MPStrategy");

  m_pNodeGeneration = NULL;
  m_pConnection = NULL;
  m_pLocalPlanners = NULL;
  m_Evaluator = NULL;
  m_pCharacterizer = NULL;
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "node_generation_methods") {
      m_pNodeGeneration = new GenerateMapNodes<CfgType>(*citr, GetMPProblem());
    } else if(citr->getName() == "connection_methods") {
      m_pConnection = new ConnectMap<CfgType, WeightType>(*citr, GetMPProblem());
    } else if(citr->getName() == "lp_methods") {
      m_pLocalPlanners = new LocalPlanners<CfgType, WeightType>(*citr, GetMPProblem());
    } else if (citr->getName() == "MPEvaluator_methods") {
      m_Evaluator = new MapEvaluator<CfgType, WeightType>(*citr, GetMPProblem());
    } else if(citr->getName() == "MPStrategyMethod") {
      ParseStrategyMethod(*citr);
    } else if(citr->getName() == "MPCharacterizer") {
      m_pCharacterizer = new MPCharacterizer<CfgType, WeightType>(*citr, GetMPProblem());
    } else {
      citr->warnUnknownNode();
    }
  }

  LOG_DEBUG_MSG( "~MPStrategy::MPStrategy()");
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
  if (m_Evaluator)
    m_Evaluator->PrintOptions(out_os);
}

void MPStrategy::
ParseStrategyMethod(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG( "MPStrategy::ParseStrategyMethod()");
  in_Node.verifyName(string("MPStrategyMethod"));
  
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "PRMRoadmap") {
      PRMRoadmap* prm = new PRMRoadmap(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( prm );
    } else if(citr->getName() == "PRMOriginalRoadmap") {
      PRMOriginalRoadmap* comp = new PRMOriginalRoadmap(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( comp );
    } else if(citr->getName() == "Compare") {
      MPComparer* comp = new MPComparer(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( comp );
    } else if(citr->getName() == "RoadmapClear") {
      RoadmapClear* rmpclear = new RoadmapClear(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( rmpclear );
    } else if(citr->getName() == "RoadmapInput") {
      RoadmapInput* rmpinput = new RoadmapInput(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( rmpinput );
    } else if(citr->getName() == "MPMultiStrategy") {
      MPMultiStrategy* multistrategy = new MPMultiStrategy(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( multistrategy );
    } else if(citr->getName() == "PRMIncrementalStrategy") {
      PRMIncrementalStrategy* prminc = new PRMIncrementalStrategy(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( prminc );
    } else if(citr->getName() == "HybridPRM") {
      HybridPRM* hprm = new HybridPRM(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( hprm );
    } else if(citr->getName() == "QueryStrategy") {
      QueryStrategy* query = new QueryStrategy(*citr,GetMPProblem());
      all_MPStrategyMethod.push_back( query );
    } else {
      citr->warnUnknownNode();
    }
  }

  m_strController_MPStrategyMethod = in_Node.stringXMLParameter("Controller",true,"","Controller Method");
  LOG_DEBUG_MSG( "~MPStrategy::ParseStrategyMethod()");
}

MPStrategyMethod* MPStrategy::
GetMPStrategyMethod(string& in_strLabel) {
  vector<MPStrategyMethod*>::iterator I;
  for(I = all_MPStrategyMethod.begin(); 
      I != all_MPStrategyMethod.end(); ++I) {
        if((*I)->GetLabel() == in_strLabel) {
    LOG_DEBUG_MSG("MPStrategyMethod::GetMPStrategyMethod(): found " << in_strLabel);
          return (*I);
        }
      }
}

void MPStrategy::
Solve() {
  LOG_DEBUG_MSG("MPStrategy::Solve()")
      LOG_DEBUG_MSG("MPStrategy::Solve() -- about to run " << m_strController_MPStrategyMethod);
      (*(GetMPStrategyMethod(m_strController_MPStrategyMethod)))();
  LOG_DEBUG_MSG("~MPStrategy::Solve()")
};



MPComparer::
MPComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node,in_pProblem) {
  LOG_DEBUG_MSG("MPComparer::MPComparer()");
  ParseXML(in_Node);    
  LOG_DEBUG_MSG("~MPComparer::MPComparer()");
}
  
void 
MPComparer::
PrintOptions(ostream& out_os) { 
}
  
void 
MPComparer::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("MPComparer::ParseXML()");
  
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
  
  LOG_DEBUG_MSG("~MPComparer::ParseXML()");
}

void 
MPComparer::
operator()(int in_RegionID) { 
}
   
// @todo make the parameter be a vector<int> in_region_ids and loop through it to map regions
void 
MPComparer::
operator()(int in_RegionID_1, int in_RegionID_2) { 
  OBPRM_srand(getSeed()); 
  // mapping region 1
  LOG_DEBUG_MSG("MPComparer::() -- executing "<< m_input_methods[0]);
  MPStrategyMethod* input1 = GetMPProblem()->GetMPStrategy()->
    GetMPStrategyMethod(m_input_methods[0]);
  (*input1)(in_RegionID_1);
  
  // mapping region 2
  LOG_DEBUG_MSG("MPComparer::() -- executing "<< m_input_methods[1]);
  MPStrategyMethod* input2 = GetMPProblem()->GetMPStrategy()->
    GetMPStrategyMethod(m_input_methods[1]);
  (*input2)(in_RegionID_2); 
  
  // comparing region 1 to region 2 with each comparer
  typedef vector<string>::iterator Itrtr;
  for (Itrtr itrtr = m_comparer_methods.begin(); itrtr < m_comparer_methods.end(); itrtr++) {
    MPRegionComparerMethod< CfgType, WeightType > * region_comparer;
    region_comparer = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetComparerMethod(*itrtr);
    region_comparer->Compare(in_RegionID_1, in_RegionID_2);
  }  
}

void 
MPComparer::
operator()() {
  LOG_DEBUG_MSG("MPComparer::()");
  
  int Input1RegionId = GetMPProblem()->CreateMPRegion();
  int Input2RegionId = GetMPProblem()->CreateMPRegion();
  
  (*this)(Input1RegionId, Input2RegionId);
  
  LOG_DEBUG_MSG("MPComparer::()");  
}


MPMultiStrategy::
MPMultiStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node,in_pProblem) {
  LOG_DEBUG_MSG("MPMultiStrategy::MPMultiStrategy()");
  ParseXML(in_Node);    
  LOG_DEBUG_MSG("~MPMultiStrategy::MPMultiStrategy()");
}
  
void 
MPMultiStrategy::
PrintOptions(ostream& out_os) { 
}
  
void 
MPMultiStrategy::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("MPMultiStrategy::ParseXML()");
  
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if (citr->getName() == "strategy") {
      string strategy = citr->stringXMLParameter("Method",true,"","Method");
      m_strategy_methods.push_back(strategy);
    } else {
      citr->warnUnknownNode();
    }
  }
  
  LOG_DEBUG_MSG("~MPMultiStrategy::ParseXML()");
}

void 
MPMultiStrategy::
operator()(int in_RegionID) { 
  // initializing region from input
  typedef vector< string >::iterator VITRTR;
  for (VITRTR s_itrtr = m_strategy_methods.begin(); s_itrtr < m_strategy_methods.end(); s_itrtr++) { 
    LOG_DEBUG_MSG("MPMultiStrategy::() -- executing "<< (*s_itrtr));
    MPStrategyMethod* strategy = GetMPProblem()->GetMPStrategy()->
      GetMPStrategyMethod(*s_itrtr);
    (*strategy)(in_RegionID);
  }
}
   
void 
MPMultiStrategy::
operator()() {
  LOG_DEBUG_MSG("MPMultiStrategy::()");
  
  int RegionId = GetMPProblem()->CreateMPRegion();    
  (*this)(RegionId);
  
  LOG_DEBUG_MSG("MPMultiStrategy::()");
}
