#ifndef _MPNN_NEIGHBORHOOD_FINDER_H_
#define _MPNN_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.h"
#include "MPNNWrapper.h"

#include <vector>
#include <functional>
using namespace std;


////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPNNNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    MPNNNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5) :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
        this->SetName("MPNNNF");
        this->m_nfType = K;
        this->m_k = _k;
	/////////////////////////////////////
	//from orig
	m_epsilon = 0.0;
	m_use_scaling = 0;
	m_use_rotational = 0;
	m_max_points = 50000;
	m_max_neighbors = 1000;
	m_lastRdmpSize = -1;

	/////////////////////////////////////
      }

    MPNNNF(XMLNode& _node) :
      NeighborhoodFinderMethod<MPTraits>(_node) {
        this->SetName("MPNNNF");
        this->m_nfType = K;
        this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");
	m_lastRdmpSize = -1;
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os << "\tk: " << this->m_k << endl;
    }

    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, const CfgType& _cfg, OutputIterator _out);

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighborPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          OutputIterator _out);

    void AddPoint(CfgType& _cfg, VID _v);
    void UpdateInternalModel();


  int m_cur_roadmap_version; // used when updating internal model
  double m_epsilon; // measure of approximateness used by internal kd-tree
  int m_use_scaling;
  int m_use_rotational;
  int m_max_points; // maximum number of points the internal kd-tree can store
  int m_max_neighbors; // maximum number of neighbors the internal kd-tree can find
  int m_lastRdmpSize;

  MPNNWrapper *kdtree; // set up by the AddPoint function
};

template <class MPTraits>
void
MPNNNF<MPTraits>::AddPoint(CfgType& _cfg, VID _v) {
  kdtree->add_node(_cfg.GetData(), _v);
}

template <typename MPTraits>
void
MPNNNF<MPTraits>::
UpdateInternalModel()
{
  MethodTimer mt(this->GetStatClass(), "MPNNNF::UpdateInternalModel");

  RoadmapType* _rmp = this->GetRoadmap();
  GraphType* g = _rmp->GetGraph();
  int curRdmpSize = 0;
  for(auto v=g->begin(); v!=g->end(); v++) curRdmpSize++;
  if (curRdmpSize==m_lastRdmpSize)
    return;
  else m_lastRdmpSize = curRdmpSize; //continue to update model

  //create kdtree structure
  CfgType temp(this->GetTask()->GetRobot());
  int dim = temp.DOF();

  // NOTE: everything after the 3rd DOF is rotational.
  vector<int> topology(dim);
  for (int i = 0; i < dim; i++) {
     if (i < temp.PosDOF())
	topology[i] = 1;
     else
	if (m_use_rotational)
	   topology[i] = 2;
	else
	   topology[i] = 1;
  }

  if(m_max_points < curRdmpSize)
    m_max_points = curRdmpSize;

  kdtree = new MPNNWrapper(topology, m_max_points, m_max_neighbors, m_epsilon);

  m_cur_roadmap_version = -1;

  //set bound
  if(this->m_debug)
    cout << "Updating internal model rdmp size " << curRdmpSize << endl;

  int vertexIndex=0;
  for(auto v=g->begin(); v!=g->end(); v++,vertexIndex++) {
     CfgType cfg = v->property();
     this->AddPoint(cfg, v->descriptor());
  }

  if(this->m_debug)
    cout << "-done! Updating internal model rdmp size " << curRdmpSize << endl;
}

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp, InputIterator _first, InputIterator _last,
    const CfgType& _cfg, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(), "MPNNNF::FindNeighbors");
  this->IncrementNumQueries();

  GraphType* map = _rmp->GetGraph();
  auto dmm = this->GetDistanceMetric(this->m_dmLabel);

  if(!this->m_k) {
    for(InputIterator it = _first; it != _last; ++it)
      if(map->GetVertex(it) != _cfg)
        *_out++ = make_pair(_rmp->GetGraph()->GetVID(it),
            dmm->Distance(map->GetVertex(it), _cfg));
    return _out;
  }

  this->UpdateInternalModel();
  vector< pair<VID, double> > closest;
  kdtree->KClosest(_cfg.GetData(), this->m_k, back_inserter(closest));

  // Reverse order
  return copy(closest.begin(), closest.end(), _out);
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "Not implemented.");
  MethodTimer mt(this->GetStatClass(), "MPNNNF::FindNeighborPairs");
  this->IncrementNumQueries();

  GraphType* map = _rmp->GetGraph();
  auto dmm = this->GetDistanceMetric(this->m_dmLabel);

  if(!this->m_k){
    for(InputIterator i1 = _first1; i1!=_last1; ++i1)
      for(InputIterator i2 = _first2; i2!=_last2; ++i2)
        if(i1 != i2)
          *_out++ = make_pair(
              make_pair(map->GetVID(i1), map->GetVID(i2)),
              dmm->Distance(map->GetVertex(i1), map->GetVertex(i2)));
    return _out;
  }
  vector<pair<pair<VID, VID>, double> > closest;
  return copy(closest.rbegin(), closest.rend(), _out);
}

#endif
