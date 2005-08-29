#include "MPStrategy.h"


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
     
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
    if(string(pChild->Value()) == "node_generation_methods") {
      m_pNodeGeneration = new GenerateMapNodes<CfgType>(pChild, GetMPProblem());
    } else if(string(pChild->Value()) == "connection_methods") {
      m_pConnection = new ConnectMap<CfgType, WeightType>(pChild, GetMPProblem());
    } else if(string(pChild->Value()) == "lp_methods") {
      m_pLocalPlanners = new LocalPlanners<CfgType, WeightType>(pChild, GetMPProblem());
    } else if(string(pChild->Value()) == "MPStrategyMethod") {
      ParseStrategyMethod(pChild);
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
    } else if(string(pChild->Value()) == "Compare") {
      MPCompare* comp = new MPCompare(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( comp );
    } else if(string(pChild->Value()) == "RoadmapInput") {
      RoadmapInput* rmpinput = new RoadmapInput(pChild,GetMPProblem());
      all_MPStrategyMethod.push_back( rmpinput );
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

