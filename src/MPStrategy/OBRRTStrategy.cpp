#include "OBRRTStrategy.h"
#include "MPRegion.h"

void
OBRRTStrategy::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } 
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 0.05, 0.0, 1.0, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, 1.0, "Minimum Distance");
  m_numRoots = _node.numberXMLParameter("numRoots", false, 1, 0, MAX_INT, "Number of Roots");
  m_growthFocus = _node.numberXMLParameter("growthFocus", false, 0.0, 0.0, 1.0, "#GeneratedTowardsGoal/#Generated");
  m_sampler = _node.stringXMLParameter("sampler", true, "", "Sampler Method");
  m_vc = _node.stringXMLParameter("vc", true, "", "Validity Test Method");
  m_nf = _node.stringXMLParameter("nf", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dm",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lp", true, "", "Local Planning Method");
  m_query = _node.stringXMLParameter("query", false, "", "Query Filename"); 
  m_g0 = _node.numberXMLParameter("g0", false, 0.1, 0.0, 1.0, "g0 Growth Method");
  m_g1 = _node.numberXMLParameter("g1", false, 0.1, 0.0, 1.0, "g1 Growth Method");
  m_g2 = _node.numberXMLParameter("g2", false, 0.1, 0.0, 1.0, "g2 Growth Method");
  m_g3 = _node.numberXMLParameter("g3", false, 0.1, 0.0, 1.0, "g3 Growth Method");
  m_g4 = _node.numberXMLParameter("g4", false, 0.1, 0.0, 1.0, "g4 Growth Method");
  m_g5 = _node.numberXMLParameter("g5", false, 0.1, 0.0, 1.0, "g5 Growth Method");
  m_g6 = _node.numberXMLParameter("g6", false, 0.1, 0.0, 1.0, "g6 Growth Method");
  m_g7 = _node.numberXMLParameter("g7", false, 0.1, 0.0, 1.0, "g7 Growth Method"); 
  m_g8 = _node.numberXMLParameter("g8", false, 0.1, 0.0, 1.0, "g8 Growth Method");

  //MAPRM values
  m_exact = _node.boolXMLParameter("exact", false, "", "Exact Medial Axis Calculation");
  m_rayCount = _node.numberXMLParameter("rays", false, 20, 0, 50, "Number of Clearance Rays");
  m_penetration = _node.numberXMLParameter("penetration", false, 5, 0, 50, "Pentration");
  m_useBbx = _node.boolXMLParameter("useBBX", true, "", "Use Bounding Box");
  m_hLen = _node.numberXMLParameter("hLen", false, 5, 0, 20, "History Length");
  m_positional = _node.boolXMLParameter("positional", true, "", "Use Position in MA Calculations");
  m_debug = _node.boolXMLParameter("debug", false, "", "Debug Mode");

  _node.warnUnrequestedAttributes();

  //Normalize probabilities
  double total = m_g0 + m_g1 + m_g2 + m_g3 + m_g4 + m_g5 + m_g6 + m_g7 + m_g8;
  m_g0 = m_g0/total;
  m_g1 = m_g1/total;
  m_g2 = m_g2/total;
  m_g3 = m_g3/total;
  m_g4 = m_g4/total;
  m_g5 = m_g5/total;
  m_g6 = m_g6/total;
  m_g7 = m_g7/total;
  m_g8 = m_g8/total;

  if(m_debug) PrintOptions(cout);
}

OBRRTStrategy::VID
OBRRTStrategy::ExpandTree(int _regionID, CfgType& _dir){
  VID recentVID = INVALID_VID;
  return recentVID;
}

//Standard RRT Expand
CfgType OBRRTStrategy::g0(int _regionID, CfgType& _dir){
  return _dir;  
}

//Random position, same orientation
CfgType OBRRTStrategy::g1(int _regionID, CfgType& _dir){
  MPRegion<CfgType, WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  Environment* env = region->GetRoadmap()->GetEnvironment();
  CfgType newCfg;
  Point3d pt = env->GetBoundingBox()->GetRandomPoint();
  //Set random position
  newCfg.SetSingleParam(0, pt[0]);
  newCfg.SetSingleParam(1, pt[1]);
  newCfg.SetSingleParam(2, pt[2]);
  //Copy over orientation information
  for(int i = 3; i < _dir.DOF(); i++){
    newCfg.SetSingleParam(i, _dir.GetData()[i]);
  }
  return newCfg;
}

CfgType OBRRTStrategy::g8(int _regionID, CfgType& _dir){
  StatClass stats;
  MPRegion<CfgType, WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  Environment* env = region->GetRoadmap()->GetEnvironment();

  CfgType newCfg = _dir;
  if(!PushToMedialAxis(GetMPProblem(), env, newCfg, stats, m_vc, m_dm, m_exact, m_rayCount, m_exact, m_penetration, m_useBbx, .0001, m_hLen, m_debug, m_positional)){
    return CfgType(); //Error out   
  }
  else{
    return newCfg;     
  }
}



