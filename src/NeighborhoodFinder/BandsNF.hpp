#ifndef _BANDS_NEIGHBORHOOD_FINDER_H_
#define _BANDS_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "OBPRMDef.h"
#include "DistanceMetrics.h"
#include "util.h"
#include "MPProblem.h"

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



template <class T>
class Bands_DIST_Compare : public binary_function<const pair<T,double>,
              const pair<T,double>, bool> {
 public:
  bool operator() (const pair<T, double> _p1,
      const pair<T, double> _p2) {
    return (_p1.second < _p2.second);
  }
};

/////// Policy definitions
class Policy {

public:
  typedef RoadmapGraph<CfgType, WeightType>::VID VID;
  typedef vector< pair<VID, double> >::iterator VEC_ITR;
  
  Policy() { m_k = 0; }
  
  Policy(int k, bool debug) {
    m_k = k;
    m_debug = debug;
  }
  
  virtual vector< pair<VID, double> >
    SelectNeighbors(VEC_ITR _candidates_first, VEC_ITR _candidates_last) = 0;
  
  int getK() { return m_k; }
  
protected:
  int m_k;
  bool m_debug;
};

class ClosestPolicy : public Policy {

public:
  ClosestPolicy() : Policy() { }
  ClosestPolicy(int k, bool debug) : Policy(k, debug) { }
  
  vector< pair<VID, double> >
  SelectNeighbors(VEC_ITR _candidates_first, VEC_ITR _candidates_last)
  {
    if (m_debug) cout << "ClosestPolicy::SelectNeighbors()" << endl;
    
    int max_index = 0;
    double max_value = MAX_DIST;
    vector< pair<VID, double> > neighbors(getK(), make_pair(-1, max_value));
        
    for (VEC_ITR itr = _candidates_first; itr != _candidates_last; ++itr) {
      
      double dist = (*itr).second;
      
      // if this distance is less than the existing max, we'll replace it 
      if(dist < neighbors[max_index].second) { 
        neighbors[max_index] = *itr;
        max_value = dist;

        //search for new max_index (faster O(k) than sort O(k log k) )
        for (size_t p = 0; p < getK(); ++p) {
          if (max_value < neighbors[p].second) {
            max_value = neighbors[p].second;
            max_index = p;
          }
        }
      }
    }
    
    return neighbors;
  }
  
};

class RandomPolicy : public Policy {

public:
  RandomPolicy() : Policy() { }
  RandomPolicy(int k, bool debug) : Policy(k, debug) { }
  
  vector< pair<VID, double> >
  SelectNeighbors(VEC_ITR _candidates_first, VEC_ITR _candidates_last)
  {
    if (m_debug) cout << "RandomPolicy::SelectNeighbors()" << endl;
    
    vector< pair<VID, double> > neighbors;
    
    int max_iter = getK();
    if (_candidates_last - _candidates_first < max_iter)
      max_iter = _candidates_last - _candidates_first;
      
    for (int i = 0; i < max_iter; i++) {
      
      pair<VID, double> p;
      
      // select random candidate that hasn't been added yet
      bool done = false;
      while (!done) {
        
        int id = (int)(OBPRM_lrand()%(_candidates_last - _candidates_first));
        p = *(_candidates_first + id);

        VID v = p.first;        
        if (m_debug) cout << "\tchecking id = " << id << ", VID = " << v;
        
        // check to see if this has been added
        done = true;
        for (VEC_ITR n_itr = neighbors.begin(); n_itr != neighbors.end(); ++n_itr)
          if ((*n_itr).first == v) {
            if (m_debug) cout << " | already added" << endl;
            done = false;
          }
      }
      neighbors.push_back(p);
      if (m_debug) cout << " | added!" << endl;
    }
    
    return neighbors;
  }
  
};

class RankWeightedRandomPolicy : public Policy {

public:  
  RankWeightedRandomPolicy() : Policy() { }
  RankWeightedRandomPolicy(int k, bool debug) : Policy(k, debug) { }
  
  vector< pair<VID, double> >
  SelectNeighbors(VEC_ITR _candidates_first, VEC_ITR _candidates_last)
  {
    if (m_debug) cout << "RankWeightedRandomPolicy::SelectNeighbors()" << endl;
    
    vector< pair<VID, double> > neighbors;
    
    int max_iter = getK();
    if (_candidates_last - _candidates_first < max_iter)
      max_iter = _candidates_last - _candidates_first;
      
    double max_rank = (_candidates_last - _candidates_first);
    
    if (m_debug) cout << "\t\tmax_rank = " << max_rank << endl;
    
    for (int i = 0; i < max_iter; i++) {
      pair<VID, double> p;
      
      // select random candidate that hasn't been added yet
      bool done = false;
      while (!done) {
        
        int id = (int)(OBPRM_lrand()%(_candidates_last - _candidates_first));
        p = *(_candidates_first + id);
        
        // check to see if this VID has been added
        done = true;
        for (VEC_ITR n_itr = neighbors.begin(); n_itr != neighbors.end(); ++n_itr)
          if ((*n_itr).first == p.first)
            done = false;
            
        if (done == true) {
          // if it hasn't been added, add it with some probability
          double prob = pow((max_rank - id) / (max_rank), 1.5);
          double roll = OBPRM_drand();

          // if we are taking less than K (every neighbor in the candidate set), set prob to 1
          if (max_iter <= getK()) {
            prob = 1.0;
          }

          if (m_debug) cout << "\t\t\trank = " << id << ", prob = " << prob << ", roll = " << roll;
          
          if (roll < prob) {
            neighbors.push_back(p);
            if (m_debug) cout << " | added";
          }
          else
            done = false;
            
          if (m_debug) cout << endl;
        }
      }  
    }
    
    return neighbors;
  }
  
};

class DistanceWeightedRandomPolicy : public Policy {

public:
  DistanceWeightedRandomPolicy() : Policy() { }
  DistanceWeightedRandomPolicy(int k, bool debug) : Policy(k, debug) { }
  
  vector< pair<VID, double> >
  SelectNeighbors(VEC_ITR _candidates_first, VEC_ITR _candidates_last)
  {
    if (m_debug) cout << "DistanceWeightedRandomPolicy::SelectNeighbors()" << endl;
    
    vector< pair<VID, double> > neighbors;
    
    int max_iter = getK();
    if (_candidates_last - _candidates_first < max_iter)
      max_iter = _candidates_last - _candidates_first;
      
    reverse_iterator<VEC_ITR> riter = reverse_iterator<VEC_ITR>(_candidates_last);
    double max_dist = (*riter).second;
    double min_dist = (*_candidates_first).second;
    
    if (m_debug) cout << "\t\tmin_dist = " << min_dist << ", max_dist = " << max_dist << endl;
    
    for (int i = 0; i < max_iter; i++) {
      
      pair<VID, double> p;
      
      // select random candidate that hasn't been added yet
      bool done = false;
      while (!done) {
        
        int id = (int)(OBPRM_lrand()%(_candidates_last - _candidates_first));
        p = *(_candidates_first + id);
        
        // check to see if this VID has been added
        done = true;
        for (VEC_ITR n_itr = neighbors.begin(); n_itr != neighbors.end(); ++n_itr)
          if ((*n_itr).first == p.first)
            done = false;
            
        if (done == true) {
          // if it hasn't been added, add it with some probability
          double prob = pow((max_dist - p.second) / (max_dist - min_dist), 1.5);
          double roll = OBPRM_drand();
          
          // if we are taking less than K (every neighbor in the candidate set), set prob to 1
          if (max_iter <= getK()) {
            prob = 1.0;
          }
          
          if (m_debug) cout << "\t\t\tdist = " << p.second << ", prob = " << prob << ", roll = " << roll;

          if (roll < prob) {
            neighbors.push_back(p);
            if (m_debug) cout << " | added";
          }
          else
            done = false;
            
          if (m_debug) cout << endl;
        }
      }  
    }
    
    return neighbors;
  }
  
};


/////// Band definitions
template<typename CFG, typename WEIGHT>
class Band {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  Band(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool debug)
  {  
    m_debug = debug;
    // note: A temporary fix (hack) until distance metric is properly fixed. This picks the second listed
    // distance metric from the xml file.
    dmm = in_pProblem->GetDistanceMetric()->GetDefault()[1];
    m_min = in_Node.numberXMLParameter(string("min"), false, double(0.0), double(0.0), double(100000.0), "min");
    m_max = in_Node.numberXMLParameter(string("max"), false, DBL_MAX, double(0.0), DBL_MAX, "max");
    m_usePercent = in_Node.boolXMLParameter(string("usePercent"), false, false,
                          string("treat min and max as a percentage of the total number of vertices in the roadmap"));
    
    string policy = in_Node.stringXMLParameter(string("policy"), true, string("closest"), string("selection policy"));
    int k = in_Node.numberXMLParameter(string("k"), true, int(1), int(0), int(10000), "k");
    
    cout << "Band : Params found - min = " << m_min << " | max = " << m_max << " | k = " << k << " | policy = " << policy << endl;
    
    if (policy == "closest") {
      m_policy = new ClosestPolicy(k, debug);
    } else
    if (policy == "random") {
      m_policy = new RandomPolicy(k, debug);
    } else
    if (policy == "RWR") {
      m_policy = new RankWeightedRandomPolicy(k, debug);
    } else
    if (policy == "DWR") {
      m_policy = new DistanceWeightedRandomPolicy(k, debug);
    }
    
    m_type = "";
  }
  
  // given initial set V (_input_first --> _input_last), and CFG v1, return V_n.
  template <typename InputIterator>
  vector< pair<VID, double> >
  GetNeighbors(Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _input_first, InputIterator _input_last, CFG _cfg);
  // { 
  //     if (m_debug) cout << "Band::GetNeighbors()" << endl;
  //     
  //     // this will be overwritten by extending classes
  //     vector< pair<VID, double> > neighbors;
  //     return neighbors;
  //   }
  
  double getMin() { return m_min; }
  double getMax() { return m_max; }
  bool getDebug() { return m_debug; }
  bool getUsePercent() { return m_usePercent; }
  string getType() { return m_type; }
  
protected:
  
  template <typename InputIterator>
  vector< pair<VID, double> >
  GetCandidateSet(Roadmap<CFG,WEIGHT>* _rmp, 
      InputIterator _input_first, InputIterator _input_last, CFG _cfg)
  {
    if (m_debug) cout << "Band::GetCandidateSet()" << endl;
    
    // this will be overwritten by extending classes
    vector< pair<VID, double> > candidates;
    return candidates;
  }
  
  
  template <typename InputIterator>
  vector< pair<VID, double> >
  GetDistList(Roadmap<CFG,WEIGHT>* _rmp,
      InputIterator _input_first, InputIterator _input_last, CFG _cfg)
  {  
    if (m_debug) cout << "Band::GetDistList()" << endl;
    
    Environment* _env = _rmp->GetEnvironment();
    RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
        
    vector< pair<VID, double> > dist_list;
    
    InputIterator V1;
    
    // compute sorted neighbor list
    for (V1 = _input_first; V1 != _input_last; ++V1) {
      CFG v1 = (*(pMap->find_vertex(*V1))).property();

      if(v1 == _cfg)
        continue; //don't connect same

      double dist = dmm->Distance(_env, _cfg, v1);
      
      pair<VID, double> p = make_pair(*V1, dist);
      dist_list.push_back(p);
    }
    
    sort(dist_list.begin(), dist_list.end(), Bands_DIST_Compare<VID>());
    
    return dist_list;
  }
  
  bool m_debug;
  bool m_usePercent;
  double m_min;
  double m_max;
  string m_type;
  DistanceMetricMethod* dmm;
  Policy* m_policy;
};

template<typename CFG, typename WEIGHT>
class DBand : public Band<CfgType, WeightType> {
  
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  DBand(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool debug)
    : Band<CfgType, WeightType>(in_Node, in_pProblem, debug)
  { 
    if (debug) cout << "DBand::DBand()" << endl; 
    m_type = "DBand";
  }
    
  template <typename InputIterator>
  vector< pair<VID, double> >
  GetNeighbors(Roadmap<CFG,WEIGHT>* _rmp, InputIterator _input_first, InputIterator _input_last, CFG _cfg)
  {
    if (m_debug) cout << "DBand::GetNeighbors()" << endl;
    // get candidate set
    vector< pair<VID, double> > candidate_set = GetCandidateSet(_rmp, _input_first, _input_last, _cfg);
    if (m_debug) cout << "  num_candidates = " << candidate_set.size() << endl;
    
    // get neighbors from candidate set using policy
    vector< pair<VID, double> > neighbors = m_policy->SelectNeighbors(candidate_set.begin(), candidate_set.end());
    return neighbors; 
  }

private:
  
  template <typename InputIterator>
  vector< pair<VID, double> >
  GetCandidateSet(Roadmap<CFG,WEIGHT>* _rmp, 
      InputIterator _input_first, InputIterator _input_last, CFG _cfg)
  {
    if (getDebug()) cout << "DBand::GetCandidateSet()" << endl;
    
    // obtain sorted distance list
    vector< pair<VID, double> > dist_list = GetDistList(_rmp, _input_first, _input_last, _cfg);
    vector< pair<VID, double> > candidates;
    
    double min, max;
    
    min = getMin();
    max = getMax();
    
    // iterate through list, return all (VID, dist) pairs that are between min and max
    typename vector< pair<VID, double> >::iterator V1;
    if (getDebug()) cout << "\tchecking candidates (min = " << min << ", max = " << max << ") for CFG = " << _cfg << ": " << endl;
    for (V1 = dist_list.begin(); V1 != dist_list.end(); ++V1) {
      double dist = (*V1).second;
      if (getDebug()) cout << "\t\t(" << (*V1).first << ", " << (*V1).second << ") ";
      if (min <= dist && dist < max) {
        if (getDebug()) cout << "added";
        candidates.push_back(*V1);
      }
      if (getDebug()) cout << endl;
      //if (getDebug()) cout << "Candidate: VID = " << (*V1).first << " | dist = " << (*V1).second << endl;
    }
    
    return candidates;
  }
  
};

template<typename CFG, typename WEIGHT>
class RBand : public Band<CfgType, WeightType> {
  
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  RBand(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool debug)
    : Band<CfgType, WeightType>(in_Node, in_pProblem, debug)
  { 
    if (debug) cout << "RBand::RBand()" << endl; 
    m_type = "RBand";
  }
    
  template <typename InputIterator>
  vector< pair<VID, double> >
  GetNeighbors(Roadmap<CFG,WEIGHT>* _rmp, InputIterator _input_first, InputIterator _input_last, CFG _cfg)
  {
    if (m_debug) cout << "RBand::GetNeighbors()" << endl;
    // get candidate set
    vector< pair<VID, double> > candidate_set = GetCandidateSet(_rmp, _input_first, _input_last, _cfg);
    if (m_debug) cout << "  num_candidates = " << candidate_set.size() << endl;
    
    // get neighbors from candidate set using policy
    vector< pair<VID, double> > neighbors = m_policy->SelectNeighbors(candidate_set.begin(), candidate_set.end());
    return neighbors; 
  }

private:
  
  template <typename InputIterator>
  vector< pair<VID, double> >
  GetCandidateSet(Roadmap<CFG,WEIGHT>* _rmp, 
      InputIterator _input_first, InputIterator _input_last, CFG _cfg)
  {
    if (getDebug()) cout << "RBand::GetCandidateSet()" << endl;
    
    // obtain sorted distance list
    vector< pair<VID, double> > dist_list = GetDistList(_rmp, _input_first, _input_last, _cfg);
    vector< pair<VID, double> > candidates;

    double min, max;
    
    if (getUsePercent()) {
      min = getMin() * (_input_last - _input_first);
      max = getMax() * (_input_last - _input_first);
    } else {
      min = getMin();
      max = getMax();
    }
    
    // iterate through list, return all (VID, dist) pairs that are between min and max
    double rank = 0;
    typename vector< pair<VID, double> >::iterator V1;
    if (getDebug()) cout << "\tchecking candidates (min = " << min << ", max = " << max << ") for CFG = " << _cfg << ": " << endl;
    for (V1 = dist_list.begin(); V1 != dist_list.end(); ++V1) {
      if (getDebug()) cout << "\t\t(" << (*V1).first << ", " << (*V1).second << ") ";
      if (min <= rank && rank < max) {
        if (getDebug()) cout << "added";
        candidates.push_back(*V1);
      }
      if (getDebug()) cout << endl;
      rank++;
    }
    
    return candidates;
  }
  
};


template<typename CFG, typename WEIGHT>
class BandsNF: public NeighborhoodFinderMethod {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  BandsNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) : NeighborhoodFinderMethod(ParseLabelXML(in_Node)) {
    dmm = in_pProblem->GetDistanceMetric()->GetDefault()[0];
    m_debug = in_Node.boolXMLParameter(string("debug"), false, false, string(""));
    
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if (citr->getName() == "DBand") {
        cout << "DBand found" << endl;
        Band<CFG,WEIGHT>* dband = new DBand<CFG,WEIGHT>(*citr, in_pProblem, m_debug);
        m_bands.push_back(dband);
      } else if(citr->getName() == "RBand") {
        cout << "RBand found" << endl;
        Band<CFG,WEIGHT>* rband = new RBand<CFG,WEIGHT>(*citr, in_pProblem, m_debug);
        m_bands.push_back(rband);
      } 
    }
  }

  BandsNF(DistanceMetricMethod* _dmm, std::string _strLabel) : NeighborhoodFinderMethod(_strLabel) {
    dmm = _dmm;
    m_debug = false;
  }

  virtual const std::string GetName () const {
    return BandsNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "BandsNF";
  }
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << std::endl;
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


private:
  
  DistanceMetricMethod* dmm; ///\todo change to a nice typedef later!
  bool m_debug;
  vector<Band<CFG, WEIGHT>* > m_bands;
  
};

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BandsNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, InputIterator _input_first, InputIterator _input_last, 
  VID _v, int k, OutputIterator _out) 
{
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  return KClosest(_rmp, _input_first, _input_last, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BandsNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  InputIterator _input_first, InputIterator _input_last, CFG _cfg, 
  int k, OutputIterator _out)
{
  IncrementNumQueries();
  StartTotalTime();
  StartQueryTime();
  
  vector< pair<VID, double> > neighbors;
    
  // iterate through bands
  typename vector<Band<CFG, WEIGHT>* >::iterator band_itr;
  for (band_itr = m_bands.begin(); band_itr != m_bands.end(); ++band_itr) {
    if (m_debug) cout << "Finding Neighbors for Band" << endl;
    
    vector< pair<VID, double> > band_neighbors;
    
    if ((*band_itr)->getType() == "DBand") {
      DBand<CFG, WEIGHT>* band = (DBand<CFG, WEIGHT>*)(*band_itr);
      band_neighbors = band->GetNeighbors(_rmp, _input_first, _input_last, _cfg);
    }
    if ((*band_itr)->getType() == "RBand") {
      RBand<CFG, WEIGHT>* band = (RBand<CFG, WEIGHT>*)(*band_itr);
      band_neighbors = band->GetNeighbors(_rmp, _input_first, _input_last, _cfg);
    }
    
    typename vector< pair<VID, double> >::iterator itr;
    for (itr = band_neighbors.begin(); itr != band_neighbors.end(); ++itr) {
      if ((*itr).first != -1) {
        if (m_debug) cout << "neighbor: VID = " << (*itr).first << " | dist = " << (*itr).second << endl;
        neighbors.push_back(*itr);
      }
    }
  }
   
  sort(neighbors.begin(), neighbors.end(), T_DIST_Compare<VID>());
    
  // now add VIDs from neighbors to output
  for (size_t p = 0; p < neighbors.size(); p++) {
    if (neighbors[p].first != VID(-999)) {
      if (m_debug) cout << "\tVID = " << neighbors[p].first << " | dist = " << neighbors[p].second << endl;
      *_out = neighbors[p].first;
      ++_out;
    }
  }
  
  EndQueryTime();
  EndTotalTime();
  return _out;
}
/*
template<typename CFG, typename WEIGHT>
template<typename InputIterator>
void
BandsNF<CFG,WEIGHT>::
ClosestBand( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _input_first, InputIterator _input_last, CFG _cfg,  
  int index_first, int index_last,
  double min_dist, double max_dist,
  vector< pair< VID, double > >& closest)
{
  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;

  int max_index = index_first; 
  double max_value = MAX_DIST;
  
  for (InputIterator V1 = _input_first; V1 != _input_last; ++V1) {
    CFG v1 = pMap->find_vertex(*V1).property();

    if(v1 == _cfg)
      continue; //don't connect same

    double dist = dmm->Distance(_env, _cfg, v1);

    if(dist < closest[max_index].second && dist > min_dist && dist <= max_dist) { 
      closest[max_index] = make_pair(*V1,dist);
      max_value = dist;

      //search for new max_index (faster O(k) than sort O(k log k) )
      for (size_t p = index_first; p < index_last; ++p) {
        if (max_value < closest[p].second) {
          max_value = closest[p].second;
          max_index = p;
        }
      }
    }
  }
}

template<typename CFG, typename WEIGHT>
template<typename InputIterator>
void
BandsNF<CFG,WEIGHT>::
RandomBand( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _input_first, InputIterator _input_last, CFG _cfg,  
  int index_first, int index_last,
  double min_dist, double max_dist,
  vector< pair< VID, double > >& closest)
{
  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  
  for (size_t p = index_first; p < index_last; ++p) {
    bool found = false;
    while (!found) {
      int id = (int)(OBPRM_lrand()%(_input_last - _input_first));
      InputIterator V1 = _input_first + id;
      double dist = dmm->Distance(_env, _cfg, pMap->find_vertex(*V1).property());
      pair<VID, double> entry = make_pair(*V1, dist);

      // if this node is in a valid distance, and doesn't already exist in closest, add it
      if (dist > min_dist && dist <= max_dist && find(closest.begin(), closest.end(), entry) == closest.end()) {
        closest[p] = entry;
        found = true;
      }
    }
  }
}

// alpha = parameter to bias distance component of probability of selection
//    ... higher alpha takes longer to compute, but produces overall closer candidates
//    ... alpha = 0 gives an even probability distribution
//    ... default alpha should be 1
template<typename CFG, typename WEIGHT>
template<typename InputIterator>
void
BandsNF<CFG,WEIGHT>::
WeightedRandomBand( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _input_first, InputIterator _input_last, CFG _cfg,  
  int index_first, int index_last,
  double min_dist, double max_dist,
  vector< pair< VID, double > >& closest)
{
  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  
  // if max_dist is arbitrary, set to be the farthest node in the roadmap (O(n) operation)
  if (max_dist == MAX_DIST) {
    max_dist = 0;
    for (InputIterator V1 = _input_first; V1 != _input_last; ++V1) {
      double dist = dmm->Distance(_env, _cfg, pMap->find_vertex(*V1).property());
      if (dist > max_dist)
        max_dist = dist;
    }
    max_dist += 0.01;
  }
  
  // for each slot in closest, select with computed probability
  for (size_t p = index_first; p < index_last; ++p) {
    bool found = false;
    while (!found) {
      int id = (int)(OBPRM_lrand()%(_input_last - _input_first));
      InputIterator V1 = _input_first + id;
      double dist = dmm->Distance(_env, _cfg, pMap->find_vertex(*V1).property());
      pair<VID, double> entry = make_pair(*V1, dist);
      
      // if this node is in a valid distance, and doesn't already exist in closest, compute the probability of adding
      if (dist > min_dist && dist <= max_dist && find(closest.begin(), closest.end(), entry) == closest.end()) {
        double prob = pow((max_dist - dist) / (max_dist - min_dist), 1.0);
        double roll = OBPRM_drand();    
        if (m_debug) cout << "\t\tcandidate VID = " << *V1 << " | dist = " << dist 
                          << " | alpha = 1.0 | P = " << prob << " | roll = " << roll << endl;
    
        if (roll < prob) {
          if (m_debug) cout << "\t\t\tSELECTED" << endl;
          closest[p] = entry;
          found = true;
        }
      }
    }
  }
}
*/
template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
BandsNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, VID _v, int k, OutputIterator _out)
{
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  return KClosest(_rmp, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
BandsNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  CFG _cfg, int k, OutputIterator _out) 
{
  RoadmapGraph<CFG, WEIGHT>* pMap = _rmp->m_pRoadmap;
  vector<VID> rmp_vertices;
  pMap->GetVerticesVID(rmp_vertices);
  
  KClosest(_rmp, rmp_vertices.begin(), rmp_vertices.end(), _cfg, k, _out);
  return _out;
  /*
  StartTotalTime();
  IncrementNumQueries();
  StartConstructionTime();
  StartQueryTime();

  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;

  std::vector<VID> input;
  pMap->GetVerticesVID(input);
  
  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< VID, double > > closest(k, make_pair(-999, max_value));

  // iterate and find closest k
  typename RoadmapGraph<CFG,WEIGHT>::VI V1;
  
  int count = 0;
  for(V1 = pMap->begin(); V1 != pMap->end(); ++V1) {
    count++;
    CFG& v1 = V1.property();
    
    if(v1 == _cfg)
      continue; //don't connect same

    double dist = dmm->Distance(_env, _cfg, v1);
    
    if(dist < closest[max_index].second) { 
      closest[max_index] = make_pair(V1.descriptor(), dist);
      max_value = dist;
  
      //search for new max_index (faster O(k) than sort O(k log k) )
      for (int p = 0; p < closest.size(); ++p) {
        if (max_value < closest[p].second) {
          max_value = closest[p].second;
          max_index = p;
        }
      }

    }
  }
 
  sort(closest.begin(), closest.end(), T_DIST_Compare<VID>());
  EndQueryTime(); 
  EndConstructionTime();
  // now add VIDs from closest to
  for (int p = 0; p < closest.size(); p++) {
    if (closest[p].first != -999) {
      *_out = closest[p].first;
      ++_out;
    }
  }
  EndTotalTime();
  */
  return _out;
}


template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BandsNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp, InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last,
  int k, OutputIterator _out)
{
   
  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< pair< VID, VID >, double > > kall;
 
  // now go through all kp and find closest k                                     
  InputIterator V1, V2;
  for(V1 = _in1_first; V1 != _in1_last; ++V1) {
    // initialize w/ k elements each with huge distance...                        
    vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(-999,-999),
    max_value));
    CFG v1 = (*(pMap->find_vertex(*V1))).property();
    for(V2 = _in2_first; V2 != _in2_last; ++V2) {
      //marcom/08nov03 check if results in other functions is same                      
      if(*V1 == *V2)
        continue; //don't connect same                                                  
    
      double dist = dmm->Distance(_env, v1, (*(pMap->find_vertex(*V2))).property());
      if(dist < kp[max_index].second) {
        kp[max_index] = make_pair(make_pair(*V1,*V2),dist);
        max_value = dist;
      
        //search for new max_index (faster O(k) than sort O(k log k) )                  
        for (int p = 0; p < kp.size(); ++p) {
          if (max_value < kp[p].second) {
            max_value = kp[p].second;
            max_index = p;
          }
        }
      }
    }//endfor c2                                                                  
    kall.insert(kall.end(),kp.begin(),kp.end());
  }//endfor c1                                                                    
 
  sort(kall.begin(), kall.end(), DIST_Compare<VID>());
  
  for (int p = 0; p < k; ++p) {
    if (kall[p].first.first != -999 && kall[p].first.second != -999){
      *_out = kall[p].first;
      ++_out;
    }
  }
  return _out;
}


#endif //end #ifndef _BANDS_NEIGHBORHOOD_FINDER_H_
