#ifndef _CGAL_NEIGHBORHOOD_FINDER_H_
#define _CGAL_NEIGHBORHOOD_FINDER_H_


#include <CGAL/Cartesian_d.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include "NeighborhoodFinderMethod.hpp"
#include "util.h"
#include "MPProblem.h"

#include <vector>
#include <functional>
using namespace std;


typedef CGAL::Cartesian_d<double> K;


class PMPL_Point_d : public K::Point_d {
 public:
  
  template <class InputIterator>
    PMPL_Point_d (int d, InputIterator first, InputIterator last)
    : K::Point_d (d, first, last) {vid=-1;}
  template <class InputIterator>
    PMPL_Point_d (int d, int _vid, InputIterator first, InputIterator last)
    : K::Point_d (d, first, last) {vid=_vid;}

    int vid;
}; //class PMPL_Point_d


typedef CGAL::Search_traits<K::FT, PMPL_Point_d, K::Cartesian_const_iterator_d, K::Construct_cartesian_const_iterator_d> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;
typedef PMPL_Point_d Point_d;


template<typename CFG, typename WEIGHT>
class CGALNF: public NeighborhoodFinderMethod {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  CGALNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(ParseLabelXML(in_Node),in_Node,in_pProblem) {

    
    m_epsilon = in_Node.numberXMLParameter("epsilon", false, double(0.0),
                                                  double(0.0), double(100.0),
                                                  "Epsilon value for CGAL");
    m_use_scaling = in_Node.numberXMLParameter("use_scaling",false,int(0),
                                                  int(0),int(1),
                                                  "Bounding-box scaling used on pos DOFs");
    
    m_cur_roadmap_version = -1;
    CFG temp;
    m_max_bbox_range = double(0.0);
    for(int i=0; i< temp.posDOF(); ++i) {
      std::pair<double,double> range = in_pProblem->GetEnvironment()->GetBoundingBox()->GetRange(i);
      double tmp_range = range.second-range.first;
      if(tmp_range > m_max_bbox_range) m_max_bbox_range = tmp_range;
    }
  }

  CGALNF(DistanceMetricMethod* _dmm, std::string _strLabel) :
    NeighborhoodFinderMethod(_strLabel) {
    dmm = _dmm;
    m_epsilon = 0.0;
    m_use_scaling = 0;
    m_cur_roadmap_version = -1;
  }

  virtual ~CGALNF() {}

  virtual const std::string GetName () const {
    return CGALNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "CGALNF";
  }
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << ":: epsilon = " << m_epsilon << ", use_scaling = " << m_use_scaling << std::endl;
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
  double m_epsilon; // appr
  int m_use_scaling;
  Tree m_tree;
  int m_cur_roadmap_version; // used when updating internal model
  double m_max_bbox_range;

};

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
CGALNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v,
    int k, OutputIterator _out) {
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  return KClosest(_rmp, _input_first, _input_last, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
CGALNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  InputIterator _input_first, InputIterator _input_last, CFG _cfg, 
  int k, OutputIterator _out) {
  StartTotalTime();
  IncrementNumQueries();
  Tree temptree;
	InputIterator V1;
	VID _v = 0;
  int dim = _cfg.DOF();
	for(V1 = _input_first; V1 != _input_last; ++V1){
    CFG _v_cfg = (*(_rmp->m_pRoadmap->find_vertex(*V1))).property();
    temptree.insert(Point_d(dim, _v, _v_cfg.GetData().begin(),_v_cfg.GetData().end()));
    ++_v;
	}
	Point_d query(Point_d(dim, _cfg.GetData().begin(),_cfg.GetData().end()));

  StartQueryTime();
	Neighbor_search search(temptree, query, k+1, m_epsilon);
  EndQueryTime();

	for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it){
    if((*(_rmp->m_pRoadmap->find_vertex(it->first.vid))).property() == _cfg) continue;
//    cout << std::sqrt(it->second) << " - VID = " << it->first.vid << endl;
	  *_out++ = it->first.vid;
	}
  
  EndTotalTime();
	return _out;

}


template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
CGALNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  VID _v, int k, OutputIterator _out) {

  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property(); 
  return KClosest(_rmp, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
CGALNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  CFG _cfg, int k, OutputIterator _out) {
  //cout << "CGALNF::KClosest() - For entire roadmap" << endl;
  StartTotalTime();

  StartConstructionTime();
  UpdateInternalModel(_rmp);
  EndConstructionTime();

  IncrementNumQueries();
  
  int dim = _cfg.DOF();
  
  // insert scaled query (copy of original CFG)
  vector<double> query_cfg(dim);
  copy(_cfg.GetData().begin(), _cfg.GetData().end(), query_cfg.begin());
  if (m_use_scaling) {
    query_cfg[0] /= m_max_bbox_range;
    query_cfg[1] /= m_max_bbox_range;
    query_cfg[2] /= m_max_bbox_range;
  }
	Point_d query(Point_d(dim, query_cfg.begin(), query_cfg.end()));

  StartQueryTime();
	Neighbor_search search(m_tree, query, k+1, m_epsilon);
  EndQueryTime();

  for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it){
    if((*(_rmp->m_pRoadmap->find_vertex(it->first.vid))).property() == _cfg) continue;
    //cout << std::sqrt(it->second) << " - VID = " << it->first.vid << endl;
	  *_out++ = it->first.vid;
	}
	
	EndTotalTime();
	return _out;

}


template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
CGALNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last, 
  int k, OutputIterator _out) {
 
  return _out;
}

template<typename CFG, typename WEIGHT>
void
CGALNF<CFG,WEIGHT>::
UpdateInternalModel( Roadmap<CFG,WEIGHT>* _rmp )
{  
  int new_version = _rmp->m_pRoadmap->roadmapVCS.get_version_number();
  if (this->m_cur_roadmap_version == new_version)
    return;
  
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
      CFG cfg_to_add;
      
      // scale roadmap CFGs
      cfg_to_add = (*(_rmp->m_pRoadmap->find_vertex(vid_to_add))).property();
      
      //cout << "Adding VID = " << vid_to_add << " CFG = " << (*iter).second.GetAddVertexEvent()->GetCFG() << endl;
      //cout << "CFG: 0 " << cfg_to_add.GetSingleParam(0) << " -> " << cfg_data[0] << " ?=? " << cfg_to_add.GetSingleParam(0)/m_max_bbox_range << endl;
      //cout << "CFG: 1 " << cfg_to_add.GetSingleParam(1) << " -> " << cfg_data[1] << " ?=? " << cfg_to_add.GetSingleParam(1)/m_max_bbox_range << endl;
      //cout << "CFG: 2 " << cfg_to_add.GetSingleParam(2) << " -> " << cfg_data[2] << " ?=? " << cfg_to_add.GetSingleParam(2)/m_max_bbox_range << endl;
      if (m_use_scaling) {
        cfg_to_add.SetSingleParam(0, cfg_to_add.GetSingleParam(0)/m_max_bbox_range);
        cfg_to_add.SetSingleParam(1, cfg_to_add.GetSingleParam(1)/m_max_bbox_range);
        cfg_to_add.SetSingleParam(2, cfg_to_add.GetSingleParam(2)/m_max_bbox_range);
      }
      m_tree.insert(Point_d(dim, vid_to_add, cfg_to_add.GetData().begin(), cfg_to_add.GetData().end()));
      
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

#endif //end ifndef _CGAL_NEIGHBORHOOD_FINDER_H_
