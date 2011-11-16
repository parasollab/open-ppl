
#ifndef _ST_NEIGHBORHOOD_FINDER_H_
#define _ST_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "util.h"
#include "MPProblem.h"

#include "Clock_Class.h"
#include <vector>
#include <functional>
#include <spillTree.hpp>
#include "Graph.h"
#include <math.h>

class Cfg;
class MultiBody;
class Input;
class Environment;
class n_str_param;
class MPProblem;
template <class CFG, class WEIGHT> class Roadmap;

using namespace std;


template<typename CFG, typename WEIGHT>
class STNF: public NeighborhoodFinderMethod {
public:

  STNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(in_Node, in_pProblem) {

    overlapDistance = in_Node.numberXMLParameter("overlapDistance", false, double(0.0),
						 double(0.0), double(100.0),
						 "overlap distance for spilltree");
    m_cur_roadmap_version = -1;
    spillTreeNotCreated=true;
    //spillTree<CFG, WEIGHT> sTree(5000,.1,.6,true,false);
    //spillTreePtr=&sTree;
  }

  STNF(shared_ptr<DistanceMetricMethod> _dmm, std::string _strLabel) :
    NeighborhoodFinderMethod(_strLabel) {
    dmm = _dmm;
    m_epsilon = 0.0;
    m_cur_roadmap_version = -1;
    spillTreeNotCreated=true;
    //spillTree<CFG, WEIGHT> sTree(5000,.1,.6,true,false);
    //spillTreePtr=&sTree;
    overlapDistance=.05;
  }

  

  virtual const std::string GetName () const {
    return STNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "STNF";
  }
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << ":: overlapDistance = " << overlapDistance << std::endl;
  }  

  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int k,
    OutputIterator _out);
  
  // do the work here, and have the function above obtain the CFG and call this one
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k,
    OutputIterator _out);
  
  
  // KClosest that operate over the entire roadmap to find the kclosest to a VID or CFG
  //
  // NOTE: These are the prefered methods for kClosest computations
  template <typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    VID _v, int k, OutputIterator _out);
  
  template <typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, int k, OutputIterator _out);
  

  // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
  // represent the kclosest pairs of VIDs between the two ranges.
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
    InputIterator _in1_first, InputIterator _in1_last, 
    InputIterator _in2_first, InputIterator _in2_last, 
    int k, OutputIterator _out);

  void UpdateInternalModel( Roadmap<CFG,WEIGHT>* _rmp );
    
    
private:
  //spillTree<CFG, WEIGHT> *spillTreePtr;
  double m_epsilon; // appr
  //Treem_tree;
  int m_cur_roadmap_version; // used when updating internal model
  bool spillTreeNotCreated;
  spillTree<CFG, WEIGHT> *spillTreePtr;
  double overlapDistance;

};



template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
STNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v,
    int k, OutputIterator _out) {
  //cout<<"in spill tree k closest fun"<<endl;
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = pMap->GetData(_v);
  return KClosest(_rmp, _input_first, _input_last, _v_cfg, k, _out);
}


 
template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
STNF<CFG,WEIGHT>::KClosest( Roadmap<CFG,WEIGHT>* _rmp, InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k, OutputIterator _out) {
  list<VID> verticies;    
  InputIterator v1;
  StartTotalTime();
  for(v1 = _input_first; v1 != _input_last; ++v1) {
    VID vid=*v1;
    verticies.push_back(vid);
  }
  spillTree<CFG, WEIGHT> sTree(100,overlapDistance,.6,true,false,false); 
  spillTree<CFG, WEIGHT> *sTreePtr = &sTree;
    //sTreePtr = &sTree;
  sTreePtr->setDistanceMetric(dmm);
  sTreePtr->setRoadmap(_rmp);
  sTreePtr->create(verticies);
  vector<VID> *closest = new vector<VID>();
  StartQueryTime();
  closest=sTreePtr->query(_cfg, k);
  EndQueryTime();
  for(std::vector<VID>::iterator iter = closest->begin(); iter != closest->end(); ++iter) {
    //cout<<"closest="<<*iter<<endl;
    *_out = *iter;
    ++_out;
  }
  EndTotalTime();
  return _out;
}


template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
STNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  VID _v, int k, OutputIterator _out) {

  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = pMap->GetData(_v); 
  return KClosest(_rmp, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
STNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  CFG _cfg, int k, OutputIterator _out) {
  
  StartTotalTime();
  //StartConstructionTime();
  if(spillTreeNotCreated){
    m_cur_roadmap_version = _rmp->m_pRoadmap->roadmapVCS.get_version_number();
    RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
    vector<VID> vec_vids;
    pMap->GetVerticesVID(vec_vids);
    list<VID> verticies;
    for(std::vector<VID>::iterator iter = vec_vids.begin(); iter != vec_vids.end(); ++iter) {
      VID vid=*iter;
      verticies.push_back(vid);
    }
    StartConstructionTime();
    spillTreePtr = new spillTree<CFG, WEIGHT>(50,overlapDistance,.7,false,false,false);
    spillTreePtr->setDistanceMetric(dmm);
    spillTreePtr->setRoadmap(_rmp);
    spillTreePtr->create(verticies);
    //_________test code begin
    //for(std::list<VID>::iterator iter = verticies.begin(); iter != verticies.end(); ++iter) {
    //  VID vid=*iter;
    //  spillTreePtr->addVertex(vid);
    //}
    //______test code end
    EndConstructionTime();
    spillTreeNotCreated=false;
  }else{
    StartConstructionTime();
    UpdateInternalModel(_rmp);
    EndConstructionTime();
  }
  //EndConstructionTime();
  vector<VID> *closest = new vector<VID>();
  StartQueryTime();
  closest=spillTreePtr->query(_cfg, k);
  EndQueryTime();
  for(std::vector<VID>::iterator iter = closest->begin(); iter != closest->end(); ++iter) {
    *_out = *iter;
    ++_out;
  }
  EndTotalTime();
  
  return _out;

}


template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
STNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last, 
  int k, OutputIterator _out) {
 
  return _out;
}

template<typename CFG, typename WEIGHT>
void
STNF<CFG,WEIGHT>::
UpdateInternalModel( Roadmap<CFG,WEIGHT>* _rmp )
{  
  int new_version = _rmp->m_pRoadmap->roadmapVCS.get_version_number();
  if (this->m_cur_roadmap_version == new_version){
    return;
  }
  //cout << "Updating internal model from version " << this->m_cur_roadmap_version << " to " << new_version << endl;
  
  typename RoadmapVCS<CFG, WEIGHT>::cce_iter start;
  if(this->m_cur_roadmap_version == -1) {
    start = _rmp->m_pRoadmap->roadmapVCS.begin();
  } else {
    start = _rmp->m_pRoadmap->roadmapVCS.iter_at(m_cur_roadmap_version);
  }
  typename RoadmapVCS<CFG, WEIGHT>::cce_iter end = _rmp->m_pRoadmap->roadmapVCS.end();
  typename RoadmapVCS<CFG, WEIGHT>::cce_iter iter;
  
  CFG temp_cfg;
  int dim = temp_cfg.DOF();
 // cout << "CGALNF::UpdateInternalModel - dim = " << dim << endl;
  //VID _v = 0;
  for (iter = start; iter != end; iter++) {
    if ((*iter).second.IsTypeAddVertex()) {
      //cout << "Add vertex event found... VID = " << (*iter).second.GetAddVertexEvent()->GetVID() << endl;
      VID vid_to_add = (*iter).second.GetAddVertexEvent()->GetVID();
      spillTreePtr->addVertex(vid_to_add);
      //CFG cfg_to_add = _rmp->m_pRoadmap->GetData(vid_to_add);
      //cout << "Adding VID = " << vid_to_add << " CFG = " << (*iter).second.GetAddVertexEvent()->GetCFG() << endl;
     // m_tree.insert(Point_d(dim, vid_to_add, cfg_to_add.GetData().begin(), cfg_to_add.GetData().end()));
      //m_tree.insert(Point_d(dim, (*iter).second.GetAddVertexEvent()->GetVID(), 
      //                                  (*iter).second.GetAddVertexEvent()->GetCFG().GetData().begin(), 
      //                             (*iter).second.GetAddVertexEvent()->GetCFG().GetData().end()));
      //cout << "CGALNF::UpdateInternalModel :: adding point: " << (*iter).second.GetAddVertexEvent()->GetVID();
      //for(vector<double>::const_iterator it = (*iter).second.GetAddVertexEvent()->GetCFG().GetData().begin();
      //  it != (*iter).second.GetAddVertexEvent()->GetCFG().GetData().end(); ++it) {
      //    cout << " " << *it;
      //  }
      //  cout << endl;
    }
  }
  
  m_cur_roadmap_version = new_version;  
 
}

#endif //end ifndef _ST_NEIGHBORHOOD_FINDER_H_
