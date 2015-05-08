#ifndef CGALNF_H_
#define CGALNF_H_

#include <CGAL/Cartesian_d.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include "NeighborhoodFinderMethod.h"
#include "MPProblem.h"

#include <vector>
#include <algorithm>

using namespace std;


typedef CGAL::Cartesian_d<double> K;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PMPLPointD : public K::Point_d {
  public:
    template<typename InputIterator>
      PMPLPointD (size_t d, InputIterator first, InputIterator last)
      : K::Point_d (d, first, last), m_it(-1) {
      }

    template<typename InputIterator>
      PMPLPointD (unsigned long long _it, size_t d, InputIterator first, InputIterator last)
      : K::Point_d (d, first, last), m_it(_it) {
      }

    unsigned long long m_it;
}; //class PMPLPointD

typedef CGAL::Search_traits<K::FT, PMPLPointD, K::Cartesian_const_iterator_d, K::Construct_cartesian_const_iterator_d> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> NeighborSearch;
typedef NeighborSearch::Tree Tree;
typedef PMPLPointD PointD;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class CGALNF: public NeighborhoodFinderMethod {
  public:
    CGALNF(string _dmm = "", double _epsilon = 0.0, bool _useScaling = false, string _label = "", MPProblem* _mp = NULL) :
      NeighborhoodFinderMethod(_dmm, _label, _mp), m_epsilon(_epsilon), m_useScaling(_useScaling), m_tmpTree(NULL), m_curRoadmapVersion(-1) {
        SetName("CGALNF");
      }

    CGALNF(XMLNode& _node, MPProblem* _problem) :
      NeighborhoodFinderMethod(_node, _problem), m_tmpTree(NULL) {
        SetName("CGALNF");
        m_epsilon = _node.Read("epsilon", false, 0.0, 0.0, 100.0, "Epsilon value for CGAL");
        m_useScaling = _node.Read("useScaling", false, false, "Bounding-box scaling used on pos DOFs");
        m_curRoadmapVersion = -1;

        shared_ptr<Boundary> b = _problem->GetEnvironment()->GetBoundary();
        m_maxBBXRange = 0.0;
        for(size_t i = 0; i < (size_t)b->GetPosDOFs(); ++i) {
          std::pair<double,double> range = b->GetRange(i);
          double tmpRange = range.second-range.first;
          if(tmpRange > m_maxBBXRange) m_maxBBXRange = tmpRange;
        }

        if(this->m_debug)
          Print(cout);
      }

    virtual ~CGALNF() {
      delete m_tmpTree;
    }

    virtual void Print(std::ostream& _os) const {
      NeighborhoodFinderMethod::Print(_os);
      _os << "epsilon: " << m_epsilon << " "
        << "use_scaling: " << m_useScaling << " ";
    }

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RDMP* _rmp,
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out);

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RDMP* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          size_t _k, OutputIterator _out);

    template<typename RDMP, typename InputIterator>
      void UpdateInternalModel(RDMP* _rmp, InputIterator _first, InputIterator _last);

  private:
    double m_epsilon; // approximation
    int m_useScaling;
    Tree m_tree;
    Tree* m_queryTree;
    Tree* m_tmpTree;
    int m_curRoadmapVersion; // used when updating internal model
    double m_maxBBXRange;

};

// Returns all nodes within radius from _cfg
template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator
CGALNF::KClosest(RDMP* _roadmap, InputIterator _first, InputIterator _last,
    typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out) {

  typedef typename RDMP::VID VID;
  typedef typename RDMP::CfgType CFG;
  typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
  typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  RoadmapGraphType* map = _roadmap->m_pRoadmap;

  StartTotalTime();

  StartConstructionTime();
  UpdateInternalModel(_roadmap, _first, _last);
  EndConstructionTime();

  IncrementNumQueries();

  size_t dim = _cfg.DOF();

  // insert scaled query (copy of original CFG)
  vector<double> queryCfg(dim);
  copy(_cfg.GetData().begin(), _cfg.GetData().end(), queryCfg.begin());
  if (m_useScaling) {
    //normalize x,y,z components to [0,1]
    queryCfg[0] /= m_maxBBXRange;
    queryCfg[1] /= m_maxBBXRange;
    queryCfg[2] /= m_maxBBXRange;
  }
  PointD query(PointD(dim, queryCfg.begin(), queryCfg.end()));

  StartQueryTime();
  NeighborSearch search(*m_queryTree, query, _k+1, m_epsilon);
  EndQueryTime();

  for(NeighborSearch::iterator it = search.begin(); it != search.end(); ++it){
    VID vid = it->first.m_it;
    CFG node = GetCfg()(map, vid);
    if(node == _cfg) continue;
    *_out++ = vid;
  }

  EndTotalTime();
  return _out;
}

// Returns all pairs within radius
template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator
CGALNF::KClosestPairs(RDMP* _roadmap,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    size_t _k, OutputIterator _out) {
  cerr << "ERROR:: CGALNF::KClosestPairs is not yet implemented. Exiting" << endl;
  exit(1);
}

template<typename RDMP, typename InputIterator>
void
CGALNF::UpdateInternalModel(RDMP* _rmp, InputIterator _first, InputIterator _last){
  typedef typename RDMP::VID VID;
  typedef typename RDMP::CfgType CFG;
  typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
  typedef typename RoadmapGraphType::RoadmapVCSType RoadmapVCSType;
  typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  RoadmapGraphType* map = _rmp->m_pRoadmap;

  int newVersion = map->roadmapVCS.get_version_number();
  if (this->m_curRoadmapVersion == newVersion)
    return;

  typename RoadmapVCSType::cce_iter start;
  if(this->m_curRoadmapVersion == -1) {
    start = map->roadmapVCS.begin();
  } else {
    start = map->roadmapVCS.iter_at(m_curRoadmapVersion);
  }
  typename RoadmapVCSType::cce_iter end = map->roadmapVCS.end();
  typename RoadmapVCSType::cce_iter iter;

  size_t dim = CFG().DOF();

  for (iter = start; iter != end; iter++) {
    if ((*iter).second.IsTypeAddVertex()) {

      VID vidToAdd = (*iter).second.GetAddVertexEvent()->GetVID();

      // scale roadmap CFGs
      CFG cfgToAdd = GetCfg()(map, vidToAdd);

      if (m_useScaling) {
        cfgToAdd.SetSingleParam(0, cfgToAdd.GetSingleParam(0)/m_maxBBXRange);
        cfgToAdd.SetSingleParam(1, cfgToAdd.GetSingleParam(1)/m_maxBBXRange);
        cfgToAdd.SetSingleParam(2, cfgToAdd.GetSingleParam(2)/m_maxBBXRange);
      }

      m_tree.insert(PointD(vidToAdd, dim, cfgToAdd.GetData().begin(), cfgToAdd.GetData().end()));
    }
  }

  //should not be included if using the roadmap version
  if(!m_fromRDMPVersion){
    delete m_tmpTree;
    m_tmpTree = new Tree();
    InputIterator V1;
    for(V1 = _first; V1 != _last; ++V1){
      CFG node = GetCfg()(map, V1);
      m_tmpTree->insert(PointD(*V1, dim, node.GetData().begin(), node.GetData().end()));
    }
    m_queryTree = m_tmpTree;
  }
  else{
    m_fromRDMPVersion = false;
    //if(m_queryTree != NULL)
    m_queryTree = &m_tree;
  }

  m_curRoadmapVersion = newVersion;
}

#endif //end ifndef _CGAL_NEIGHBORHOOD_FINDER_H_
