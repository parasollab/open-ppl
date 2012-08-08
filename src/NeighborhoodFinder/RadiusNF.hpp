// Finds all neighbors within a certain radius

#ifndef RADIUSNF_H_
#define RADIUSNF_H_

#include "NeighborhoodFinderMethod.hpp"
#include <vector>

using namespace std;

class Environment;

class RadiusNF: public NeighborhoodFinderMethod {
  public:
    RadiusNF(string _dmm = "", double _r = 1.0, string _label = "", MPProblem* _problem = NULL) 
      : NeighborhoodFinderMethod(_dmm, _label, _problem), m_radius(_r) {
        SetName("RadiusNF");
      }
    
    RadiusNF(XMLNodeReader& _node, MPProblem* _problem) 
      : NeighborhoodFinderMethod(_node, _problem) {
        SetName("RadiusNF");
        m_radius = _node.numberXMLParameter("radius", true, 0.5, 0.0, MAX_DBL, "Radius");
        _node.warnUnrequestedAttributes();
        
        if(this->m_debug)
          PrintOptions(cout);
      }
    
    virtual ~RadiusNF() {}
    
    virtual void PrintOptions(ostream& _os) const {
      NeighborhoodFinderMethod::PrintOptions(_os);
      _os << "radius: " << m_radius << " ";
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

  private:
    double m_radius;
};

// Returns all nodes within radius from _cfg
template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator 
RadiusNF::KClosest(RDMP* _roadmap, InputIterator _first, InputIterator _last, 
    typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out) {
  
  typedef typename RDMP::VID VID;
  typedef typename RDMP::CfgType CFG;
  typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
  typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  // TO DO NOTE: A temporary fix to support parallel runtime. The problem here is that is the way
  // we pass pointer around which is a bit ugly with parallelism. In this particular case
  // the pointer to GetMPProblem became invalid because of the way RadiusNF is called from
  // Connector, thus call to timing stats below seg fault. One fix is to call RadiusNF(_node, _problem)
  // constructor and this will be done when parallel code supports all NF. What this means is 
  // that I can not just support RadiusNF by itself.
  #ifndef _PARALLEL
  StartTotalTime();
  StartQueryTime();
  #endif

  IncrementNumQueries();
  
  Environment* env = _roadmap->GetEnvironment();
  RoadmapGraphType* map = _roadmap->m_pRoadmap;

  vector<pair<VID, double> > inRadius;

  // Find all nodes within radius
  for(InputIterator it = _first; it != _last; it++) {
    CFG node = GetCfg()(map, it);
    if(node == _cfg) // Don't connect to itself
      continue;

    // If within radius, add to list
    double dist = GetDMMethod()->Distance(env, _cfg, node);
    if(dist <= m_radius)
      inRadius.push_back(make_pair(*it, dist));
  }
 
  sort(inRadius.begin(), inRadius.end(), CompareSecond<VID, double>());
  
  // Output results
  for(size_t i = 0; i < inRadius.size(); i++)
    *(_out++) = inRadius[i].first;

  #ifndef _PARALLEL
  EndQueryTime();
  EndTotalTime();
  #endif
  
  return _out;
}

// Returns all pairs within radius
template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator 
RadiusNF::KClosestPairs(RDMP* _roadmap,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    size_t _k, OutputIterator _out) {
   
  typedef typename RDMP::VID VID;
  typedef typename RDMP::CfgType CFG;
  typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
  typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  Environment* env = _roadmap->GetEnvironment();
  RoadmapGraphType* map = _roadmap->m_pRoadmap;
  vector<pair<pair<VID, VID >, double> > inRadius;
 
  // Find all pairs within radius
  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    CFG node1 = GetCfg()(map, it1);
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      if(*it1 == *it2) // Don't connect to itself
        continue;
      CFG node2 = GetCfg()(map, it2);
      
      // If within radius, add to list
      double dist = GetDMMethod()->Distance(env, node1, node2);
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
