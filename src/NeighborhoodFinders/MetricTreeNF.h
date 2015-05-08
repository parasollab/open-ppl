
#ifndef METRICTREENF_H_
#define METRICTREENF_H_

#include "NeighborhoodFinderMethod.h"
#include "MPProblem.h"

#include <SpillTree.h>
#include <math.h>

#include <vector>
#include <functional>
using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFG, typename WEIGHT>
class MetricTreeNF: public NeighborhoodFinderMethod {
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

  MetricTreeNF(XMLNode& _node, MPProblem* _problem) :
    NeighborhoodFinderMethod(_node,_problem) {
    m_epsilon = _node.Read("epsilon", false, 0.0, 0.0, 100.0, "Epsilon value for CGAL");
    m_cur_roadmap_version = -1;
    spillTreeNotCreated=true;
  }

  MetricTreeNF(shared_ptr<DistanceMetricMethod> _dmm, std::string _label) :
    NeighborhoodFinderMethod(_dmm, _label) {
    m_epsilon = 0.0;
    m_cur_roadmap_version = -1;
    spillTreeNotCreated=true;
  }

  virtual ~MetricTreeNF() {}

  virtual const std::string GetName () const {
    return MetricTreeNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "MetricTreeNF";
  }
  virtual void Print(std::ostream& out_os) const {
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
  double m_epsilon; // appr
  //Treem_tree;
  int m_cur_roadmap_version; // used when updating internal model
  bool spillTreeNotCreated;
  spillTree<CFG, WEIGHT> *spillTreePtr;


};



template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MetricTreeNF<CFG,WEIGHT>::
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
MetricTreeNF<CFG,WEIGHT>::KClosest( Roadmap<CFG,WEIGHT>* _rmp, InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k, OutputIterator _out) {
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
MetricTreeNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp,
  VID _v, int k, OutputIterator _out) {

  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  return KClosest(_rmp, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
MetricTreeNF<CFG,WEIGHT>::
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
MetricTreeNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last,
  InputIterator _in2_first, InputIterator _in2_last,
  int k, OutputIterator _out) {

  return _out;
}

template<typename CFG, typename WEIGHT>
void
MetricTreeNF<CFG,WEIGHT>::
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
  //int dim = temp_cfg.DOF();
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
