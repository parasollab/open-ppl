#include "MPProblem.h"
#include "MPStrategy.h"
#include "DistanceMetrics.h"
#include "ValidityChecker.hpp"

MPProblem::
MPProblem(XMLNodeReader& in_Node, bool parse_xml) : MPBaseObject(in_Node, this) {
  if(parse_xml)
    ParseXML(in_Node);
}

MPProblem::
MPProblem(Environment* _m_pEnvironment, DistanceMetric* _m_pDistanceMetric, ValidityChecker* _m_pValidityChecker, NeighborhoodFinder* _m_pNeighborhoodFinder) : m_pEnvironment(_m_pEnvironment), m_pDistanceMetric(_m_pDistanceMetric), m_pValidityChecker(_m_pValidityChecker), m_pNeighborhoodFinder(_m_pNeighborhoodFinder) {
  m_roadmap = new Roadmap<CfgType, WeightType>();
  m_roadmap->SetEnvironment(m_pEnvironment);
  m_blockRoadmap = new Roadmap<CfgType, WeightType>();
  m_blockRoadmap->SetEnvironment(m_pEnvironment);
  m_colRoadmap = new Roadmap<CfgType, WeightType>();
  m_colRoadmap->SetEnvironment(m_pEnvironment);

  m_stats = new StatClass();
};


bool MPProblem::
ParseChild(XMLNodeReader::childiterator citr)
{
  if(citr->getName() == "environment") {
    m_pEnvironment = new Environment(*citr, this);
    return true;
  } else  if(citr->getName() == "distance_metrics") {
    m_pDistanceMetric = new DistanceMetric(*citr, this);
    return true;
  } else  if(citr->getName() == "validity_test") {
    m_pValidityChecker = new ValidityChecker(*citr, this);
    return true;
  } else  if(citr->getName() == "NeighborhoodFinder") {
    m_pNeighborhoodFinder = new NeighborhoodFinder(*citr,this);
    return true;
  } else
    return false;
}


void MPProblem::
ParseXML(XMLNodeReader& in_Node) { 
  in_Node.verifyName("MPProblem");

  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(!ParseChild(citr))
      citr->warnUnknownNode();
  }

  vector<cd_predefined> cdtypes = m_pValidityChecker->GetSelectedCDTypes();
  for(vector<cd_predefined>::iterator C = cdtypes.begin(); C != cdtypes.end(); ++C)
    m_pEnvironment->buildCDstructure(*C);

  m_roadmap = new Roadmap<CfgType, WeightType>();
  m_roadmap->SetEnvironment(m_pEnvironment);
  m_blockRoadmap = new Roadmap<CfgType, WeightType>();
  m_blockRoadmap->SetEnvironment(m_pEnvironment);
  m_colRoadmap = new Roadmap<CfgType, WeightType>();
  m_colRoadmap->SetEnvironment(m_pEnvironment);

  m_stats = new StatClass();
}


void MPProblem::
PrintOptions(ostream& out_os)
{
  out_os << "MPProblem" << endl;
  m_pMPStrategy->PrintOptions(out_os);
  m_pDistanceMetric->PrintOptions(out_os);
  m_pValidityChecker->PrintOptions(out_os);
  m_pEnvironment->PrintOptions(out_os);
}


vector<RoadmapGraph<CfgType, WeightType>::VID>
MPProblem::
AddToRoadmap(vector<CfgType>& _cfgs) {
  vector<RoadmapGraph<CfgType, WeightType>::VID> returnVec;
  for(vector<CfgType>::iterator I = _cfgs.begin(); I != _cfgs.end(); I++) {
    if((*I).IsLabel("VALID")) {
      if((*I).GetLabel("VALID")) {//Add to Free roadmap
        returnVec.push_back(m_roadmap->m_pRoadmap->AddVertex((*I)));
      } else {  //Add to Coll Roadmap 
        //LOG_DEBUG_MSG("MPRegion::AddToRoadmap() -- Adding Coll CfgType");
        
        // commented by Bryan on 5/4/08, we only want one roadmap
        //col_roadmap.m_pRoadmap->AddVertex((*I));
      }
    } else {
      if(m_debug) 
        cout << "MPRegion::AddToRoadmap() -- UNLABELED!!!!!!!" << endl;
    }
  }
  return returnVec;
}


void MPProblem::
WriteRoadmapForVizmo(ostream& _os, vector<shared_ptr<Boundary> >* _boundaries, bool _block) {
  _os << "Roadmap Version Number " << RDMPVER_CURRENT_STR;
  _os << endl << "#####PREAMBLESTART#####";
  _os << endl << "../obprm -f " << GetEnvironment()->GetEnvFileName() << " ";//commandLine;
  _os << " -bbox "; GetEnvironment()->GetBoundary()->Print(_os, ',', ',');
  if(_boundaries != NULL) {
    typedef vector<shared_ptr<Boundary> >::iterator BIT;
    for(BIT bit = _boundaries->begin(); bit!=_boundaries->end(); bit++) {
      _os << " -bbox "; (*bit)->Print(_os, ',', ',');
    }
  }
  _os << endl << "#####PREAMBLESTOP#####";

  _os << endl << "#####ENVFILESTART#####";
  _os << endl << GetEnvironment()->GetEnvFileName();
  _os << endl << "#####ENVFILESTOP#####";
  _os << endl;

  ///TODO: fix so vizmo can understand the following 3 lines instead of the explicit printouts below
  /*
  GetMPStrategy()->GetLocalPlanners()->WriteLPsForVizmo(myofstream);
  GetCollisionDetection()->WriteCDsForVizmo(myofstream);
  GetDistanceMetric()->WriteDMsForVizmo(myofstream);
  */
  _os << "#####LPSTART#####" << endl << "0" << endl << "#####LPSTOP#####" << endl;
  _os << "#####CDSTART#####" << endl << "0" << endl << "#####CDSTOP#####" << endl;
  _os << "#####DMSTART#####" << endl << "0" << endl << "#####DMSTOP#####";
  GetRoadmap()->WriteRNGseed(_os);
  _os << endl;
  
  #ifndef _PARALLEL
  if(!_block)
    stapl::sequential::write_graph(*(GetRoadmap()->m_pRoadmap), _os);         // writes verts & adj lists
  else
    stapl::sequential::write_graph(*(GetBlockRoadmap()->m_pRoadmap), _os);        // writes verts & adj lists
  #else
  stapl::write_graph(*(GetRoadmap()->m_pRoadmap), _os);
  #endif
}

void 
MPProblem::SetMPProblem(){
  m_pMPStrategy->SetMPProblem(this);
  m_pEnvironment->SetMPProblem(this);
  m_pEnvironment->GetBoundary()->SetMPProblem(this);
  m_pDistanceMetric->SetMPProblem(this);
  m_pValidityChecker->SetMPProblem(this);
  m_pNeighborhoodFinder->SetMPProblem(this);
}

