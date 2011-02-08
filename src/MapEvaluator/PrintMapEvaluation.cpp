#include "PrintMapEvaluation.h"
#include "MPRegion.h"


PrintMapEvaluation::
PrintMapEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
 : MapEvaluationMethod(in_Node, in_pProblem) 
{
  base_name = in_Node.stringXMLParameter("base_name", true, "", "base filename for map output");
}


PrintMapEvaluation::
~PrintMapEvaluation() 
{}


void
PrintMapEvaluation::
PrintOptions(ostream& out_os)
{
  out_os << GetName() << "::  base_name = " << base_name << endl;
}
  

bool
PrintMapEvaluation::
operator() (int in_RegionID) 
{
  PrintOptions(cout);

  int num_nodes = GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap()->m_pRoadmap->get_num_vertices();
  int num_edges = GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap()->m_pRoadmap->get_num_edges();
  int num_coll_nodes = GetMPProblem()->GetMPRegion(in_RegionID)->GetBlockRoadmap()->m_pRoadmap->get_num_vertices();
  int num_coll_edges = GetMPProblem()->GetMPRegion(in_RegionID)->GetBlockRoadmap()->m_pRoadmap->get_num_edges();
  
  ostringstream os_name;
  os_name << base_name << "." << num_nodes << "." << num_edges << ".map";
  ostringstream os_coll_name;
  os_coll_name << base_name << "." << num_coll_nodes << "." << num_coll_edges << ".block.map";

  ofstream os_map(os_name.str().c_str());
  GetMPProblem()->GetMPRegion(in_RegionID)->WriteRoadmapForVizmo(os_map);
  os_map.close();
  ofstream os_coll_map(os_coll_name.str().c_str());
  GetMPProblem()->GetMPRegion(in_RegionID)->WriteRoadmapForVizmo(os_coll_map, NULL, true);
  os_coll_map.close();

  return true;
}

