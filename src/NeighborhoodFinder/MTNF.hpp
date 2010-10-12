
#ifndef _MT_NEIGHBORHOOD_FINDER_H_
#define _MT_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "OBPRMDef.h"
#include "DistanceMetrics.h"
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
class MTNF: public NeighborhoodFinderMethod {
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

  MTNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(ParseLabelXML(in_Node),in_Node,in_pProblem) {
    m_epsilon = in_Node.numberXMLParameter("epsilon", false, double(0.0),
            double(0.0), double(100.0),
            "Epsilon value for CGAL");

    m_cur_roadmap_version = -1;
    spillTreeNotCreated=true;
    //spillTree<CFG, WEIGHT> sTree(5000,.1,.6,true,false);
    //spillTreePtr=&sTree;
  }

  MTNF(DistanceMetricMethod* _dmm, std::string _strLabel) :
    NeighborhoodFinderMethod(_strLabel) {
    dmm = _dmm;
    m_epsilon = 0.0;
    m_cur_roadmap_version = -1;
    spillTreeNotCreated=true;
    //spillTree<CFG, WEIGHT> sTree(5000,.1,.6,true,false);
    //spillTreePtr=&sTree;
  }

  

  virtual const std::string GetName () const {
    return MTNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "MTNF";
  }
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << ":: epsilon = " << m_epsilon << std::endl;
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
  DistanceMetricMethod* dmm; ///\todo change to a nice typedef later!
  double m_epsilon; // appr
  //Treem_tree;
  int m_cur_roadmap_version; // used when updating internal model
  bool spillTreeNotCreated;
  spillTree<CFG, WEIGHT> *spillTreePtr;


};



template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MTNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v,
    int k, OutputIterator _out) {
  //cout<<"in spill tree k closest fun"<<endl;
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  return KClosest(_rmp, _input_first, _input_last, _v_cfg, k, _out);
}


 
template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MTNF<CFG,WEIGHT>::KClosest( Roadmap<CFG,WEIGHT>* _rmp, InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k, OutputIterator _out) {
  vector<VID> verticies;    
  InputIterator v1;
  StartTotalTime();
  for(v1 = _input_first; v1 != _input_last; ++v1) {
    VID vid=*v1;
    verticies.push_back(vid);
  }
  spillTree<CFG, WEIGHT> sTree(100,1,.6,true,false,true); 
  spillTree<CFG, WEIGHT> *sTreePtr = &sTree;
    //sTreePtr = &sTree;
  sTreePtr->setDistanceMetric(dmm);
  sTreePtr->setRoadmap(_rmp);
  sTreePtr->create(verticies);
  vector<VID> *closest = new vector<VID>();
  StartQueryTime();
  closest=sTreePtr->query(_cfg, k);
  EndQueryTime();
  for(typename std::vector<VID>::iterator iter = closest->begin(); iter != closest->end(); ++iter) {
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
MTNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  VID _v, int k, OutputIterator _out) {

  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property(); 
  return KClosest(_rmp, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
MTNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  CFG _cfg, int k, OutputIterator _out) {
  
  StartTotalTime();
  
  //StartConstructionTime();
  if(spillTreeNotCreated){
    m_cur_roadmap_version = _rmp->m_pRoadmap->roadmapVCS.get_version_number();
    StartConstructionTime();
    RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
    vector<VID> vec_vids;
    pMap->GetVerticesVID(vec_vids);
    //list<VID> verticies;
    //for(std::vector<VID>::iterator iter = vec_vids.begin(); iter != vec_vids.end(); ++iter) {
    //  VID vid=*iter;
    //  verticies.push_back(vid);
    //}
    StartConstructionTime();
    spillTreePtr = new spillTree<CFG, WEIGHT>(200,1,.7,false,false, true);
    spillTreePtr->setDistanceMetric(dmm);
    spillTreePtr->setRoadmap(_rmp);
    spillTreePtr->create(vec_vids);
    EndConstructionTime();
    spillTreeNotCreated=false;
  }else{
    StartConstructionTime();
    UpdateInternalModel(_rmp);
    EndConstructionTime();
  }
  EndConstructionTime(); 

  
  vector<VID> *closest = new vector<VID>();
  StartQueryTime();
  closest=spillTreePtr->query(_cfg, k);
  EndQueryTime();
  //EndTotalTime();
  for(typename std::vector<VID>::iterator iter = closest->begin(); iter != closest->end(); ++iter) {
    *_out = *iter;
    ++_out;
  }
  EndTotalTime();
  
  return _out;

}


template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MTNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last, 
  int k, OutputIterator _out) {
 
  return _out;
}

template<typename CFG, typename WEIGHT>
void
MTNF<CFG,WEIGHT>::
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
    }
  }
  
  m_cur_roadmap_version = new_version;  
 
}

#endif //end ifndef _MT_NEIGHBORHOOD_FINDER_H_
