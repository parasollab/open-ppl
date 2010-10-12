#ifndef _MPNN_NEIGHBORHOOD_FINDER_H_
#define _MPNN_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "OBPRMDef.h"
#include "DistanceMetrics.h"
#include "util.h"
#include "MPProblem.h"
#include "MPNNWrapper.h"

#include "Clock_Class.h"
#include <vector>
#include <functional>

class Cfg;
class MultiBody;
class Input;
class Environment;
class n_str_param;
class MPProblem;
template <class CFG, class WEIGHT> class Roadmap;

using namespace std;
/**Compare two distances in DIST_TYPE instances.
 *return (_cc1.second < _cc2.second)
 */
template <class T>
class T_PAIR_DIST_Compare : public binary_function<const pair< pair<T,T>,double >,
              const pair< pair<T,T>,double >,
              bool> {
 public:
  bool operator()(const pair< pair<T,T>,double > _cc1,
      const pair< pair<T,T>,double > _cc2) {
    return (_cc1.second < _cc2.second);
  }
};

template<typename CFG, typename WEIGHT>
class MPNNNF: public NeighborhoodFinderMethod {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  MPNNNF() { 
    
    m_epsilon = 0.0;
    m_use_scaling = 0;
    m_use_rotational = 0;
    m_max_points = 50000;
    m_max_neighbors = 1000;
    
    CFG temp;
    int dim = temp.DOF();
    
    // NOTE: everything after the 3rd DOF is rotational.
    vector<int> topology(dim);
    for (int i = 0; i < dim; i++) {
      if (i < temp.posDOF())
        topology[i] = 1;
      else
        if (m_use_rotational)
          topology[i] = 2;
        else
          topology[i] = 1;
    }
    
    kdtree = new MPNNWrapper(topology, m_max_points, m_max_neighbors, m_epsilon);
    
    m_cur_roadmap_version = -1;
  }

  MPNNNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(ParseLabelXML(in_Node), in_Node, in_pProblem) {
  
    
    m_epsilon = in_Node.numberXMLParameter("epsilon", false, double(0.0),
                                                  double(0.0), double(100.0),
                                                  "Epsilon value for MPNN");
    m_use_scaling = in_Node.numberXMLParameter("use_scaling",true,int(0),
                                                  int(0),int(1),
                                                  "Bounding-box scaling used on pos DOFs");
    m_use_rotational = in_Node.numberXMLParameter("use_rotational",true,int(0),
                                                  int(0),int(1),
                                                  "Use rotational-coordinate topology");                                                  
    m_max_points = in_Node.numberXMLParameter("max_points", false, int(50000),
                                                  int(0), int(1000000),
                                                  "Max points for MPNN");
    m_max_neighbors = in_Node.numberXMLParameter("max_k", false, int(1000),
                                                  int(0), int(10000),
                                                  "Max neighbors for MPNN");
    
    CFG temp;
    int dim = temp.DOF();
    // NOTE: everything after the 3rd DOF is rotational.
    vector<int> topology(dim);
    for (int i = 0; i < topology.size(); i++) {
      if (i < temp.posDOF())
        topology[i] = 1;
      else
        if (m_use_rotational)
          topology[i] = 2;
        else
          topology[i] = 1;
    }
    
    kdtree = new MPNNWrapper(topology, m_max_points, m_max_neighbors, m_epsilon);
    
    m_cur_roadmap_version = -1;

    m_max_bbox_range = double(0.0);
    for(int i=0; i< temp.posDOF(); ++i) {
      std::pair<double,double> range = in_pProblem->GetEnvironment()->GetBoundingBox()->GetRange(i);
      double tmp_range = range.second-range.first;
      if(tmp_range > m_max_bbox_range) m_max_bbox_range = tmp_range;
    }
    
  }

  MPNNNF(DistanceMetricMethod* _dmm, std::string _strLabel) :
    NeighborhoodFinderMethod(_strLabel) {
    dmm = _dmm;
    
    m_epsilon = 0.0;
    m_use_scaling = 0;
    m_use_rotational = 0;
    m_max_points = 50000;
    m_max_neighbors = 1000;
    
    CFG temp;
    int dim = temp.DOF();
    // NOTE: everything after the 3rd DOF is rotational.
    vector<int> topology(dim);
    for (int i = 0; i < topology.size(); i++) {
      if (i < temp.posDOF())
        topology[i] = 1;
      else
        if (m_use_rotational)
          topology[i] = 2;
        else
          topology[i] = 1;
    }
   
    kdtree = new MPNNWrapper(topology, m_max_points, m_max_neighbors, m_epsilon);

    m_cur_roadmap_version = -1;
  }

  virtual const std::string GetName () const {
    return MPNNNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "MPNNNF";
  }
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << ":: epsilon = " << m_epsilon << ", use_scaling = " << m_use_scaling << std::endl;
  }

  // this may end up being a private function used to create an internal
  // CGAL kdtree that will be populated by a roadmap in the future
  int
  AddPoint(CFG _cfg, VID _v);

  
  // Find the k closest to _v that is in the set represented by the iterators
  // You should create a temporary tree based on the iterators to search
  // not optimal, but should still do it...
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int k,
    OutputIterator _out);
  

  // Find the k closest to _v that is in the set represented by the iterators
  // You should create a temporary tree based on the iterators to search
  // not optimal, but should still do it...
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k, 
    OutputIterator _out);

  
  // KClosest that operate over the internal kd-tree structure
  //   to find the kclosest to a VID or CFG
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

  
  void
  UpdateInternalModel( Roadmap<CFG,WEIGHT>* _rmp );

private:
  int m_cur_roadmap_version; // used when updating internal model
  double m_epsilon; // measure of approximateness used by internal kd-tree
  int m_use_scaling;
  int m_use_rotational;
  double m_max_bbox_range;
  int m_max_points; // maximum number of points the internal kd-tree can store
  int m_max_neighbors; // maximum number of neighbors the internal kd-tree can find
  
  DistanceMetricMethod* dmm; ///\todo change to a nice typedef later!
  MPNNWrapper *kdtree; // set up by the AddPoint function
};


template <typename CFG, typename WEIGHT>
int
MPNNNF<CFG, WEIGHT>::AddPoint(CFG _cfg, VID _v)
{
  kdtree->add_node(_cfg.GetData(), _v);
}

    
// Find the k closest to _v that is in the set represented by the iterators
// You should create a temporary tree based on the iterators to search
// not optimal, but should still do it...
template <typename CFG, typename WEIGHT>
template <typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int k,
    OutputIterator _out) {
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  
  this->KClosest(_rmp, _input_first, _input_last, _v_cfg, k, _out);
  
  return _out;
}

// Find the k closest to _v that is in the set represented by the iterators
// You should create a temporary tree based on the iterators to search
// not optimal, but should still do it...
template <typename CFG, typename WEIGHT>
template <typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k,
  OutputIterator _out) {
  
  IncrementNumQueries();
  StartTotalTime();

//cout << "Running KCloseset???" << endl << flush;
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  int dim = _cfg.DOF();
  
  // NOTE: everything after the 3rd DOF is rotational.
  vector<int> topology(dim);
  for (int i = 0; i < topology.size(); i++) {
    if (i < 3)
      topology[i] = 1;
    else
      if (m_use_rotational)
        topology[i] = 2;
      else
        topology[i] = 1;
  }
  
  double m_epsilon = 0.0;
  MPNNWrapper *localKdtree = new MPNNWrapper(topology, 20000, 1000, m_epsilon);
  
  // add configurations from iterator into local kdtree
  InputIterator V1; 
  for (V1 = _input_first; V1 != _input_last; ++V1) {
    CFG v1 = (*(pMap->find_vertex(*V1))).property();
    if (m_use_scaling) {
      v1.SetSingleParam(0, v1.GetSingleParam(0)/m_max_bbox_range * 2 * PI);
      v1.SetSingleParam(1, v1.GetSingleParam(1)/m_max_bbox_range * 2 * PI);
      v1.SetSingleParam(2, v1.GetSingleParam(2)/m_max_bbox_range * 2 * PI);
    }
    localKdtree->add_node(v1.GetData(), *V1);
  }

  vector< pair<VID, double> > results(k, pair<VID, double>(-999, -999.9));
  
  if (m_use_scaling) {
    _cfg.SetSingleParam(0, _cfg.GetSingleParam(0)/m_max_bbox_range * 2 * PI);
    _cfg.SetSingleParam(1, _cfg.GetSingleParam(1)/m_max_bbox_range * 2 * PI);
    _cfg.SetSingleParam(2, _cfg.GetSingleParam(2)/m_max_bbox_range * 2 * PI);
  }

  StartQueryTime();
  localKdtree->KClosest(_cfg.GetData(), k, results.begin());
  EndQueryTime();

  for (int i = 0; i < results.size(); i++) {
    *_out = results[i].first;
    //cout << results[i].second << " - VID = " << results[i].first << endl;
    ++_out;
  }
  
  localKdtree->MPNNWrapper::~MPNNWrapper();
  
  EndTotalTime();
  return _out;
}


// Find the kclosest to _v that is in the internal CGal kdtree
// created by  the "AddPoint" function
template <typename CFG, typename WEIGHT>
template <typename OutputIterator>
OutputIterator
MPNNNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  VID _v, int k, OutputIterator _out) {

  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _cfg = (*(pMap->find_vertex(_v))).property();
  
  this->KClosest(_rmp, _cfg, k, _out);
}


// Find the kclosest to _cfg that is in the internal CGal kdtree
// created by  the "AddPoint" function
template <typename CFG, typename WEIGHT>
template <typename OutputIterator>
OutputIterator
MPNNNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  CFG _cfg, int k, OutputIterator _out) {
  StartTotalTime();
  
  StartConstructionTime();
  this->UpdateInternalModel(_rmp);
  EndConstructionTime();

  IncrementNumQueries();
  vector< pair<VID, double> > results(k, pair<VID, double>(-999, -999.9));

  if (m_use_scaling) {
    _cfg.SetSingleParam(0, _cfg.GetSingleParam(0)/m_max_bbox_range * 2 * PI);
    _cfg.SetSingleParam(1, _cfg.GetSingleParam(1)/m_max_bbox_range * 2 * PI);
    _cfg.SetSingleParam(2, _cfg.GetSingleParam(2)/m_max_bbox_range * 2 * PI);
  }
  
  StartQueryTime();
//  cout << "Finding " << k << "-closest to point: " << _cfg << endl;
  kdtree->KClosest(_cfg.GetData(), k, results.begin());
  EndQueryTime();


  for (int i = 0; i < results.size(); i++) {
    *_out = results[i].first;
    //cout << results[i].second << " - VID = " << results[i].first
    //     << "\t\t" << _cfg << " to " << _rmp->m_pRoadmap->find_vertex(results[i].first).property() << endl;
    ++_out;
  }
    EndTotalTime();

	return _out;
}

// Find the k pairs that have the closest distance, where one VID must
//   come from each set of input iterators.  The output should be as
//   a pair<VID, VID>.
template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last, 
  int k, OutputIterator _out) {

  // we will take an incremental approach here... 
  
  // STEP 1:
  // create local roadmap to hold [_in2_first ... _in2_last]
  //
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  int dim = (*(pMap->find_vertex(*_in1_first))).property().DOF(); 
  
  // NOTE: everything after the 3rd DOF is rotational.
  vector<int> topology(dim);
  for (int i = 0; i < topology.size(); i++) {
    if (i < 3)
      topology[i] = 1;
    else
      if (m_use_rotational)
        topology[i] = 2;
      else
        topology[i] = 1;
  }
  
  MPNNWrapper *localKdtree = new MPNNWrapper(topology, 50000, 1000, m_epsilon);
  
  InputIterator v_iter;
  for (v_iter = _in2_first; v_iter != _in2_last; ++v_iter) {
    CFG v_iter_cfg = (*(pMap->find_vertex(*v_iter))).property();
    localKdtree->add_node(v_iter_cfg.GetData(), *v_iter);
  }
  //
  
  // STEP 2:
  // iterate through vids in _in1_first, calculating KClosest with respect
  //   to the local kd tree... compile master list of results
  // 
  vector< pair< pair<VID, VID>, double> > query_results;
  //OutputIterator query_results;
  
  for (v_iter = _in1_first; v_iter != _in1_last; ++v_iter) {
    vector< pair<VID, double> > iter_results(k, pair<VID, double>(-999, -999.9));
    
    CFG query_pt = (*(pMap->find_vertex(*v_iter))).property();
    localKdtree->KClosest(query_pt.GetData(), k, iter_results.begin());
    
    // push local results back to master list
    typename vector< pair<VID, double> >::iterator iter;
    for (iter = iter_results.begin(); iter != iter_results.end(); ++iter) {
      pair< pair<VID, VID>, double> temp;
      temp.first.first = *v_iter;
      temp.first.second = (*iter).first;
      temp.second = (*iter).second;
      if (temp.second != -999.9) {
        query_results.push_back(temp);
      }
    }
  }
  
  sort(query_results.begin(), query_results.end(), T_PAIR_DIST_Compare<VID>());
  
  typename vector< pair< pair<VID, VID>, double> >::iterator q_iter;
  int count = 0;
  for (q_iter = query_results.begin(); q_iter != query_results.end(); ++q_iter) {
    if (count == k)
      break;
    (*_out) = (*q_iter).first;
    ++_out;
    count++;
  }
  
  return _out;
}

template<typename CFG, typename WEIGHT>
void
MPNNNF<CFG, WEIGHT>::
UpdateInternalModel( Roadmap<CFG,WEIGHT>* _rmp )
{
  int new_version = _rmp->m_pRoadmap->roadmapVCS.get_version_number();
  if (this->m_cur_roadmap_version == new_version)
    return;
  
  //cout << "Updating internal model from version " << this->m_cur_roadmap_version << " to " << new_version << endl;
  
  typename RoadmapVCS<CFG, WEIGHT>::cce_iter start;
  if (this->m_cur_roadmap_version == -1)
    start = _rmp->m_pRoadmap->roadmapVCS.begin();
  else
    start = _rmp->m_pRoadmap->roadmapVCS.iter_at(m_cur_roadmap_version);
  
  typename RoadmapVCS<CFG, WEIGHT>::cce_iter end = _rmp->m_pRoadmap->roadmapVCS.end();
  typename RoadmapVCS<CFG, WEIGHT>::cce_iter iter;
  
  for (iter = start; iter != end; iter++) {
    if ((*iter).second.IsTypeAddVertex()) {
      //cout << "Add vertex event found... VID = " << (*iter).second.GetAddVertexEvent()->GetVID() << endl;
      CFG tmp = (*iter).second.GetAddVertexEvent()->GetCFG();
      if (m_use_scaling) {
        tmp.SetSingleParam(0, tmp.GetSingleParam(0)/m_max_bbox_range * 2 * PI);
        tmp.SetSingleParam(1, tmp.GetSingleParam(1)/m_max_bbox_range * 2 * PI);
        tmp.SetSingleParam(2, tmp.GetSingleParam(2)/m_max_bbox_range * 2 * PI);
      }
      this->AddPoint(tmp, (*iter).second.GetAddVertexEvent()->GetVID());
    }
  }
  
  m_cur_roadmap_version = new_version;
}

#endif //end #ifndef _MPNN_NEIGHBORHOOD_FINDER_H_
