// Finds all neighbors within a certain radius

#ifndef RADIUSNF_H_
#define RADIUSNF_H_

#include "NeighborhoodFinderMethod.hpp"
#include <vector>

using namespace std;

class Environment;

template<typename CFG, typename WEIGHT>
class RadiusNF: public NeighborhoodFinderMethod {

  protected:
  
    double m_radius;

  public:

    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
    RadiusNF(XMLNodeReader& _node, MPProblem* _problem) : NeighborhoodFinderMethod(_node, _problem) {
      ParseXML(_node);
      if(this->m_debug)
        PrintOptions(cout);
    }
    
    RadiusNF(shared_ptr<DistanceMetricMethod> _dmm, string _label = "",
        MPProblem* _problem = NULL) : NeighborhoodFinderMethod(_dmm, _label, _problem) {}
    
    void ParseXML(XMLNodeReader& _node) {
      m_radius = _node.numberXMLParameter("radius", true, 0.5, 0.0, MAX_DBL, "Radius");
      _node.warnUnrequestedAttributes();
    }
    
    virtual ~RadiusNF() {}
    
    virtual const string GetName () const {
      return RadiusNF::GetClassName();
    }
    
    static const string GetClassName() {
      return "RadiusNF";
    }
    
    virtual void PrintOptions(ostream& _os) const {
      _os << this->GetClassName() << endl;
      _os << "\tradius = " << m_radius << endl;
    }
    
    template <typename InputIterator, typename OutputIterator>
    OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _roadmap, InputIterator _inFirst,
        InputIterator _inLast, VID _v, int _k, OutputIterator _out);
    
    // The main worker, all other KClosest() methods call this one
    template <typename InputIterator, typename OutputIterator>
    OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _roadmap, InputIterator _inFirst,
        InputIterator _inLast, CFG _cfg, int _k, OutputIterator _out);
    
    // Operates over an entire roadmap to find nodes within radius to a VID or CFG
    // NOTE: These are the preferred methods for kClosest computations
    template <typename OutputIterator>
    OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _roadmap, 
        VID _v, int _k, OutputIterator _out);
    
    template <typename OutputIterator>
    OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _roadmap, 
        CFG _cfg, int _k, OutputIterator _out);
    
    // KClosest that operate over two ranges of VIDS
    template <typename InputIterator, typename OutputIterator>
    OutputIterator KClosestPairs(Roadmap<CFG, WEIGHT>* _roadmap,
        InputIterator _in1First, InputIterator _in1Last, 
        InputIterator _in2First, InputIterator _in2Last, 
        int _k, OutputIterator _out);
};

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator RadiusNF<CFG, WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _roadmap, 
    InputIterator _inFirst, InputIterator _inLast, VID _v, int _k, OutputIterator _out) {
  return KClosest(_roadmap, _inFirst, _inLast,
      (_roadmap->m_pRoadmap->find_vertex(_v))->property(), _k, _out);
}

// Returns all nodes within radius from _cfg
template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator RadiusNF<CFG, WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _roadmap, 
    InputIterator _inFirst, InputIterator _inLast, CFG _cfg, int _k, OutputIterator _out) {
  
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
  RoadmapGraph<CFG, WEIGHT>* map = _roadmap->m_pRoadmap;

  vector<pair<VID, double> > inRadius;

  // Find all nodes within radius
  for(InputIterator it = _inFirst; it != _inLast; it++) {
    CFG node = pmpl_detail::GetCfg<InputIterator>(map)(it);
    if(node == _cfg) // Don't connect to itself
      continue;

    // If within radius, add to list
    double dist = dmm->Distance(env, _cfg, node);
    if(dist <= m_radius)
      inRadius.push_back(make_pair(*it, dist));
  }
 
  sort(inRadius.begin(), inRadius.end(), compare_second<VID, double>());
  
  // Output results
  for(size_t i = 0; i < inRadius.size(); i++)
    *(_out++) = inRadius[i].first;

  #ifndef _PARALLEL
  EndQueryTime();
  EndTotalTime();
  #endif
  
  return _out;
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator RadiusNF<CFG, WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _roadmap, 
    VID _v, int _k, OutputIterator _out) { 
  return KClosest(_roadmap, 
      (_roadmap->m_pRoadmap->find_vertex(_v))->property(), _k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator RadiusNF<CFG,WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _roadmap, 
    CFG _cfg, int _k, OutputIterator _out) {
  return KClosest(_roadmap, _roadmap->m_pRoadmap->descriptor_begin(),
      _roadmap->m_pRoadmap->descriptor_end(), _cfg, _k, _out);
}

// Returns all pairs within radius
template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator RadiusNF<CFG, WEIGHT>::KClosestPairs(Roadmap<CFG, WEIGHT>* _roadmap,
    InputIterator _in1First, InputIterator _in1Last,
    InputIterator _in2First, InputIterator _in2Last,
    int _k, OutputIterator _out) {
   
  Environment* env = _roadmap->GetEnvironment();
  RoadmapGraph<CFG, WEIGHT>* map = _roadmap->m_pRoadmap;
  vector<pair<pair<VID, VID >, double> > inRadius;
 
  // Find all pairs within radius
  for(InputIterator it1 = _in1First; it1 != _in1Last; it1++) {
    CFG node1 = pmpl_detail::GetCfg<InputIterator>(map)(it1);
    for(InputIterator it2 = _in2First; it2 != _in2Last; it2++) {
      if(*it1 == *it2) // Don't connect to itself
        continue;
      CFG node2 = pmpl_detail::GetCfg<InputIterator>(map)(it2);
      
      // If within radius, add to list
      double dist = dmm->Distance(env, node1, node2);
      if(dist <= m_radius)
        inRadius.push_back(make_pair(make_pair(*it1, *it2), dist));
    }
  }
 
  // Sort pairs by increasing distance
  sort(inRadius.begin(), inRadius.end(), compare_second<pair<VID, VID>, double>());
  
  // Output results
  for(size_t i = 0; i < inRadius.size(); i++)
    *(_out++) = inRadius[i].first;
  return _out;
}

#endif
