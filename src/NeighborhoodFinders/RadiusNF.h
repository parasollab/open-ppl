// Finds all neighbors within a certain radius

#ifndef RADIUSNF_H_
#define RADIUSNF_H_

#include "NeighborhoodFinderMethod.h"
#include <vector>

using namespace std;

class Environment;

template<class MPTraits>
class RadiusNF: public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    RadiusNF(MPProblemType* _problem = NULL, string _dmLabel = "", double _r = 1.0, string _label = "") :
      NeighborhoodFinderMethod<MPTraits>(_problem, _dmLabel, _label), m_radius(_r) {
        this->SetName("RadiusNF");
      }
    
    RadiusNF(MPProblemType* _problem, XMLNodeReader& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("RadiusNF");
        m_radius = _node.numberXMLParameter("radius", true, 0.5, 0.0, MAX_DBL, "Radius");
        _node.warnUnrequestedAttributes();
      }
    
    virtual ~RadiusNF() {}
    
    virtual void PrintOptions(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::PrintOptions(_os);
      _os << "radius: " << m_radius << " ";
    }
    
    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, 
          InputIterator _first, InputIterator _last, CfgType _cfg, size_t _k, OutputIterator _out);

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1, 
          InputIterator _first2, InputIterator _last2, 
          size_t _k, OutputIterator _out);

  private:
    double m_radius;
};

// Returns all nodes within radius from _cfg
template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator 
RadiusNF<MPTraits>::KClosest(RoadmapType* _roadmap, InputIterator _first, InputIterator _last, 
    CfgType _cfg, size_t _k, OutputIterator _out) {
  
  // TO DO NOTE: A temporary fix to support parallel runtime. The problem here is that is the way
  // we pass pointer around which is a bit ugly with parallelism. In this particular case
  // the pointer to GetMPProblem became invalid because of the way RadiusNF is called from
  // Connector, thus call to timing stats below seg fault. One fix is to call RadiusNF(_node, _problem)
  // constructor and this will be done when parallel code supports all NF. What this means is 
  // that I can not just support RadiusNF by itself.
  #ifndef _PARALLEL
  this->StartTotalTime();
  this->StartQueryTime();
  #endif

  this->IncrementNumQueries();
  
  Environment* env = this->GetMPProblem()->GetEnvironment();
  GraphType* map = _roadmap->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  vector<pair<VID, double> > inRadius;

  // Find all nodes within radius
  for(InputIterator it = _first; it != _last; it++) {
    CfgType node = map->GetCfg(it);
    if(node == _cfg) // Don't connect to itself
      continue;

    // If within radius, add to list
    double dist = dmm->Distance(env, _cfg, node);
    if(dist <= m_radius)
      inRadius.push_back(make_pair(*it, dist));
  }
 
  sort(inRadius.begin(), inRadius.end(), CompareSecond<VID, double>());
  
  // Output results
  for(size_t i = 0; i < inRadius.size(); i++)
    *(_out++) = inRadius[i].first;

  #ifndef _PARALLEL
  this->EndQueryTime();
  this->EndTotalTime();
  #endif
  
  return _out;
}

// Returns all pairs within radius
template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator 
RadiusNF<MPTraits>::KClosestPairs(RoadmapType* _roadmap,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    size_t _k, OutputIterator _out) {
   
  Environment* env = this->GetMPProblem()->GetEnvironment();
  GraphType* map = _roadmap->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  vector<pair<pair<VID, VID >, double> > inRadius;
 
  // Find all pairs within radius
  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    CfgType node1 = map->GetCfg(it1);
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      if(*it1 == *it2) // Don't connect to itself
        continue;
      CfgType node2 = map->GetCfg(it2);
      
      // If within radius, add to list
      double dist = dmm->Distance(env, node1, node2);
      if(dist <= m_radius)
        inRadius.push_back(make_pair(make_pair(*it1, *it2), dist));
    }
  }
 
  // Sort pairs by increasing distance
  sort(inRadius.begin(), inRadius.end(), CompareSecond<pair<VID, VID>, double>());
  
  // Output results
  for(size_t i = 0; i < inRadius.size(); i++)
    *(_out++) = inRadius[i].first;
  return _out;
}

#endif
