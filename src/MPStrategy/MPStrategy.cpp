#include "MPStrategy.h"
#include "MPRegionComparerMethod.h"

MPStrategy::
MPStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem) : MPBaseObject(in_pNode,in_pProblem) {
  LOG_DEBUG_MSG( "MPStrategy::MPStrategy()");
  ///\todo Find a home for "addPartialEdge" and "addAllEdges" or remove all together
  addPartialEdge=true;
  addAllEdges=false;
  if(!in_pNode) {
    LOG_ERROR_MSG("MPStrategy::MPStrategy() error xml input"); exit(-1);
  }
  if(string(in_pNode->Value()) != "MPStrategy") {
    LOG_ERROR_MSG("MPStrategy::MPStrategy() error xml input"); exit(-1);
  }

  m_Evaluator = NULL;     
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
    if(string(pChild->Value()) == "node_generation_methods") {
      m_pNodeGeneration = new GenerateMapNodes<CfgType>(pChild, GetMPProblem());
    } else if(string(pChild->Value()) == "connection_methods") {
      m_pConnection = new ConnectMap<CfgType, WeightType>(pChild, GetMPProblem());
    } else if(string(pChild->Value()) == "lp_methods") {
      m_pLocalPlanners = new LocalPlanners<CfgType, WeightType>(pChild, GetMPProblem());
    } else if (string(pChild->Value()) == "MPEvaluator_methods") {
      m_Evaluator = new MapEvaluator<CfgType, WeightType>(pChild, GetMPProblem());

    } else if(string(pChild->Value()) == "MPStrategyMethod") {
      ParseStrategyMethod(pChild);
    } else if(string(pChild->Value()) == "MPCharacterizer") {
      m_pCharacterizer = new MPCharacterizer<CfgType, WeightType>(pChild, GetMPProblem());
    } else {
      LOG_WARNING_MSG("MPStrategy::  I don't know: "<< endl << *pChild);
    }
  }

  LOG_DEBUG_MSG( "~MPStrategy::MPStrategy()");
}


void MPStrategy::
PrintOptions(ostream& out_os)
{
  out_os << "MPStrategy" << endl;
  m_pNodeGeneration->PrintOptions(out_os);
  m_pConnection->PrintOptions(out_os);
  m_pLocalPlanners->PrintOptions(out_os);
  if (m_Evaluator)
    m_Evaluator->PrintOptions(out_os);
}

void MPStrategy::
ParseStrategyMethod(TiXmlNode* in_pNode) {
  if(!in_pNode) {
    LOG_ERROR_MSG("MPStrategy::ParseStrategyMethod() error xml input"); exit(-1);
  }
  if(string(in_pNode->Value()) != "MPStrategyMethod") {
    LOG_ERROR_MSG("MPStrategy::ParseStrategyMethod() require tag <MPStrategyMethod>");
    exit(-1);
  }
  
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
    if(string(pChild->Value()) == "PRMRoadmap") {
      PRMRoadmap* prm = new PRMRoadmap(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( prm );
    } else if(string(pChild->Value()) == "PRMOriginalRoadmap") {
      PRMOriginalRoadmap* comp = new PRMOriginalRoadmap(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( comp );
    } else if(string(pChild->Value()) == "Compare") {
      MPComparer* comp = new MPComparer(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( comp );
    } else if(string(pChild->Value()) == "RoadmapClear") {
      RoadmapClear* rmpclear = new RoadmapClear(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( rmpclear );
    } else if(string(pChild->Value()) == "RoadmapInput") {
      RoadmapInput* rmpinput = new RoadmapInput(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( rmpinput );
    } else if(string(pChild->Value()) == "MPMultiStrategy") {
      MPMultiStrategy* multistrategy = new MPMultiStrategy(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( multistrategy );
    } else if(string(pChild->Value()) == "PRMIncrementalStrategy") {
      PRMIncrementalStrategy* prminc = new PRMIncrementalStrategy(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( prminc );
    } else if(string(pChild->Value()) == "HybridPRM") {
      HybridPRM* hprm = new HybridPRM(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( hprm );
    } else {
      LOG_WARNING_MSG("MPStrategy::  I don't know: "<< endl << *pChild);
    }
  }
  
  
  m_strController_MPStrategyMethod = "";
  
  if(in_pNode->Type() == TiXmlNode::ELEMENT) {
    const char* carLabel = in_pNode->ToElement()->Attribute("Controller");
    if(carLabel) {
      m_strController_MPStrategyMethod =  string(carLabel);
    }
  } 
  
  
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
MPComparer(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
  MPStrategyMethod(in_pNode,in_pProblem) {
  LOG_DEBUG_MSG("MPComparer::MPComparer()");
  ParseXML(in_pNode);    
  LOG_DEBUG_MSG("~MPComparer::MPComparer()");
}
  
void 
MPComparer::
PrintOptions(ostream& out_os) { 
}
  
void 
MPComparer::
ParseXML(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("MPComparer::ParseXML()");
  
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
    if(string(pChild->Value()) == "input") {
      const char* in_char = pChild->ToElement()->Attribute("Method");
      if(in_char) {
	string strategy(in_char);
	m_input_methods.push_back(strategy);
      }
    } else if (string(pChild->Value()) == "comparer_method") {
      const char* in_char = pChild->ToElement()->Attribute("Method");
      LOG_DEBUG_MSG("MPComparer::ParseXML() -- comparer_method");
      if (in_char) {
	string evaluator_method(in_char);
	m_comparer_methods.push_back(evaluator_method);
      }
    } else {
      LOG_WARNING_MSG("MPComparer::  I don't know: "<< endl << *pChild);
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
MPMultiStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
  MPStrategyMethod(in_pNode,in_pProblem) {
  LOG_DEBUG_MSG("MPMultiStrategy::MPMultiStrategy()");
  ParseXML(in_pNode);    
  LOG_DEBUG_MSG("~MPMultiStrategy::MPMultiStrategy()");
}
  
void 
MPMultiStrategy::
PrintOptions(ostream& out_os) { 
}
  
void 
MPMultiStrategy::
ParseXML(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("MPMultiStrategy::ParseXML()");
  
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
    if (string(pChild->Value()) == "strategy") {
      const char* in_char = pChild->ToElement()->Attribute("Method");
      LOG_DEBUG_MSG("MPMultiStrategy::ParseXML() -- strategy_method");
      if (in_char) {
	string strategy(in_char);
	m_strategy_methods.push_back(strategy);
      }
    } else {
      LOG_WARNING_MSG("MPMultiStrategy::  I don't know: "<< endl << *pChild);
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
