#ifndef RANDOMNF_H_
#define RANDOMNF_H_

#include "NeighborhoodFinderMethod.h"
    
template<class MPTraits>
class RandomNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
      
    RandomNF(MPProblemType* _problem = NULL, string _dmLabel = "", string _label = ""):
      NeighborhoodFinderMethod<MPTraits>(_problem, _dmLabel, _label) {
	this->SetName("RandomNF");
    }
    
    RandomNF(MPProblemType* _problem, XMLNodeReader& _node):
      NeighborhoodFinderMethod<MPTraits>(_problem,_node) {
	this->SetName("RandomNF");
    }
    virtual ~RandomNF() {}

    //Doesn't actually compute k-closest, but k-random neighbors. 
    template<typename InputIterator, typename OutputIterator>
    OutputIterator KClosest(RoadmapType* _rmp,
	InputIterator _first, InputIterator _last,CfgType _cfg, size_t _k, OutputIterator _out);
    template <typename InputIterator, typename OutputIterator>
    OutputIterator KClosestPairs(RoadmapType* _rmp,
	InputIterator _first1, InputIterator _last1,
	InputIterator _first2, InputIterator _last2,
	size_t _k, OutputIterator _out);

    //TODO: PrintOptions; will be added by Jory later.
};

template <class MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
RandomNF<MPTraits>::KClosest(RoadmapType* _rmp, InputIterator _first, InputIterator _last,
    CfgType _cfg, size_t _k, OutputIterator _out) {

  this->IncrementNumQueries();
  #ifndef _PARALLEL
  this->StartTotalTime();
  this->StartQueryTime();
  #endif

  GraphType* map = _rmp->GetGraph();
  
  set<int> ids;

  for (size_t i = 0; i < _k && i<(_last-_first); ++i) {
    int id = 0;
    do {
      id = (int)(LRand()%(_last-_first));
    }
    while(ids.find(id) != ids.end() &&
	map->GetVID(_cfg) == map->GetVID(_first+id));
    ids.insert(id);
    *_out = map->GetVID(_first+id);
    ++_out;
  }

  #ifndef _PARALLEL
  this->EndQueryTime();
  this->EndTotalTime();
  #endif
  
  return _out; 
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
RandomNF<MPTraits>::KClosestPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    size_t _k, OutputIterator _out) {
  GraphType* map = _rmp->GetGraph();
  set<pair<int,int> > ids;
  
  for(size_t i=0,j=0; i < _k && i<(_last1-_first1) && j<(_last2-_first2); i++, j++) {
    int id1=0,id2=0;
    pair<int,int> pairId = make_pair(0,0);
    do {
      id1 = (int)(LRand()%(_last1-_first1));
      id2 = (int)(LRand()%(_last2-_first2));
      pairId = make_pair(id1,id2);
    }
    while(ids.find(pairId) != ids.end() &&
	map->GetVID(_first1+id1) == map->GetVID(_first2+id2));
    ids.insert(pairId);
    *_out = make_pair(map->GetVID(_first1+id1),map->GetVID(_first2+id2));
    ++_out;
  }
  return _out;
}

#endif
