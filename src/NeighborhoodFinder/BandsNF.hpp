#ifndef _BANDS_NEIGHBORHOOD_FINDER_H_
#define _BANDS_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "MPProblem.h"

#include <vector>
#include <functional>
using namespace std;


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
  
  virtual ~Policy() {}

  virtual vector< pair<VID, double> >
    SelectNeighbors(Roadmap<CfgType, WeightType>* _rmp, VEC_ITR _candidates_first, VEC_ITR _candidates_last) = 0;
  
  int getK() { return m_k; }
  
protected:
  int m_k;
  bool m_debug;
};

class ClosestPolicy : public Policy {

public:
  ClosestPolicy() : Policy() { }
  ClosestPolicy(int k, bool debug) : Policy(k, debug) { }
  virtual ~ClosestPolicy() {}

  vector< pair<VID, double> >
  SelectNeighbors(Roadmap<CfgType, WeightType>* _rmp, VEC_ITR _candidates_first, VEC_ITR _candidates_last)
  {
    if (m_debug) cout << "ClosestPolicy::SelectNeighbors()" << endl;
    
    int max_index = 0;
    double max_value = MAX_DIST;
    vector< pair<VID, double> > neighbors;
        
    for (VEC_ITR itr = _candidates_first; itr != _candidates_last; ++itr) {
      
      double dist = (*itr).second;
      if((int) neighbors.size()< getK()){
        for(vector< pair<VID, double> >::iterator iter2 = neighbors.begin(); iter2!=neighbors.end();iter2++){
	  if((*iter2).second > (*itr).second) {
	    swap(*itr,*iter2);
	  }
	}
	neighbors.push_back(*itr);
      }else{
        // if this distance is less than the existing max, we'll replace it 
        if(dist < neighbors[max_index].second) { 
          neighbors[max_index] = *itr;
          max_value = dist;

          //search for new max_index (faster O(k) than sort O(k log k) )
          for (int p = 0; p < getK(); ++p) {
            if (max_value < neighbors[p].second) {
              max_value = neighbors[p].second;
              max_index = p;
            }
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
  virtual ~RandomPolicy() {}

  vector< pair<VID, double> >
  SelectNeighbors(Roadmap<CfgType, WeightType>* _rmp, VEC_ITR _candidates_first, VEC_ITR _candidates_last)
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
        
        int id = (int)(LRand()%(_candidates_last - _candidates_first));
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

class PreferentialPolicy : public Policy {

public:
  PreferentialPolicy() : Policy() { }
  PreferentialPolicy(int k, bool debug) : Policy(k, debug) { }
  virtual ~PreferentialPolicy() {}

  vector< pair<VID, double> >
  SelectNeighbors(Roadmap<CfgType, WeightType>* _rmp, VEC_ITR _candidates_first, VEC_ITR _candidates_last)
  {
    if (m_debug) cout << "PreferentialPolicy::SelectNeighbors()" << endl;
    
    vector< pair<VID, double> > neighbors;
   
    int found = 0;
    int max_iter = getK();
    if (_candidates_last - _candidates_first < max_iter)
      max_iter = _candidates_last - _candidates_first;

    int set_degree = candidate_set_degree(_rmp, _candidates_first, _candidates_last);
      
    while (found < max_iter) {
      // iterate through candidate set, adding as neighbor with probability = pref_prob(_rmp, v, n)
      for (VEC_ITR itr = _candidates_first; itr != _candidates_last; ++itr) {
        double drand = DRand();
        pair<VID, double> p = *itr;
        VID v = p.first;
        double prob = pref_prob(_rmp, v, _candidates_last - _candidates_first, set_degree);
        if (m_debug) cout << "found = " << found << ", VID = " << v << ", drand = " << drand << ", prob = " << prob;
        if (drand < prob) {
          if (m_debug) cout << " ||| ";

          bool exists = false;
          for (VEC_ITR n_itr = neighbors.begin(); n_itr != neighbors.end(); ++n_itr) {
            if ((*n_itr).first == v) {
              if (m_debug) cout << " already in set...";
              exists = true;
            }
          } 

          if (!exists) {
            neighbors.push_back(p);
            if (m_debug) cout << " added!";
            found++;
          }
        }
        if (m_debug) cout << endl;
        if (found == max_iter) {
          break;
        }
      }
    }  
  
    return neighbors;
  }
  
  //////////////////////
  // Probability function
  double pref_prob(
          Roadmap<CfgType, WeightType>* _rm, VID _vid, int _n, int _tot_degree) {
    int candidate_degree = _rm->m_pRoadmap->get_degree(_vid);
    int total_degree = _tot_degree;
    if (_tot_degree == -1) total_degree = _rm->m_pRoadmap->get_num_edges(); 
    if (m_debug) cout << "pref_prob(" << _vid << ", " << _n << ") = " << 1 + candidate_degree << " / " << _n + total_degree << endl;
    return ((double)(1 + candidate_degree) / (double)(_n + total_degree));
  }

  //////////////////////
  // Get the total degree of the candidate set
  int candidate_set_degree(
          Roadmap<CfgType, WeightType>* _rm, VEC_ITR _candidates_first, VEC_ITR _candidates_last) {
    int total_degree = 0;
    for (VEC_ITR itr = _candidates_first; itr != _candidates_last; ++itr) {
      pair<VID, double> p = *itr;
      int candidate_degree = _rm->m_pRoadmap->get_degree(p.first);
      total_degree += candidate_degree;
      if (m_debug) cout << "candidate_set_degree += " << candidate_degree << endl;
    }
    return total_degree;
  }
};

class RankWeightedRandomPolicy : public Policy {

public:  
  RankWeightedRandomPolicy() : Policy() { }
  RankWeightedRandomPolicy(int k, double alpha, bool debug) : Policy(k, debug) { m_alpha = alpha; }
  virtual ~RankWeightedRandomPolicy() {}

  vector< pair<VID, double> >
  SelectNeighbors(Roadmap<CfgType, WeightType>* _rmp, VEC_ITR _candidates_first, VEC_ITR _candidates_last)
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
        
        int id = (int)(LRand()%(_candidates_last - _candidates_first));
        p = *(_candidates_first + id);
        
        // check to see if this VID has been added
        done = true;
        for (VEC_ITR n_itr = neighbors.begin(); n_itr != neighbors.end(); ++n_itr)
          if ((*n_itr).first == p.first)
            done = false;
            
        if (done == true) {
          // if it hasn't been added, add it with some probability
          double prob = pow((max_rank - id) / (max_rank), m_alpha);
          double roll = DRand();

          // if we are taking less than K (every neighbor in the candidate set), set prob to 1
          if (max_iter < getK()) {
            prob = 1.0;
          }

          if (m_debug) cout << "\t\t\trank = " << id << ", prob = " << prob << ", alpha = " << m_alpha << ", roll = " << roll;
          
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
  
  double m_alpha;

};

class DistanceWeightedRandomPolicy : public Policy {

public:
  DistanceWeightedRandomPolicy() : Policy() { }
  DistanceWeightedRandomPolicy(int k, double alpha, bool debug) : Policy(k, debug) { m_alpha = alpha; }
  virtual ~DistanceWeightedRandomPolicy() {}

  vector< pair<VID, double> >
  SelectNeighbors(Roadmap<CfgType, WeightType>* _rmp, VEC_ITR _candidates_first, VEC_ITR _candidates_last)
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
        
        int id = (int)(LRand()%(_candidates_last - _candidates_first));
        p = *(_candidates_first + id);
        
        // check to see if this VID has been added
        done = true;
        for (VEC_ITR n_itr = neighbors.begin(); n_itr != neighbors.end(); ++n_itr)
          if ((*n_itr).first == p.first)
            done = false;
            
        if (done == true) {
          // if it hasn't been added, add it with some probability
          double prob = pow((max_dist - p.second) / (max_dist - min_dist), m_alpha);
          double roll = DRand();
          
          // if we are taking less than K (every neighbor in the candidate set), set prob to 1
          if (max_iter < getK()) {
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
  
  double m_alpha;

};


/////// Band definitions
template<typename CFG, typename WEIGHT>
class Band : public NeighborhoodFinderMethod {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
 
  Band(shared_ptr<DistanceMetricMethod> _dmm, std::string _label) : NeighborhoodFinderMethod(_dmm, _label) {}
 
  Band(XMLNodeReader& _inNode, MPProblem* _inProblem, bool _debug): NeighborhoodFinderMethod(_inNode, _inProblem)
  {  
    m_debug = _debug;
    // note: A temporary fix (hack) until distance metric is properly fixed. This picks the second listed
    // distance metric from the xml file.
      
    std::string dm_label = _inNode.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
    dmm = _inProblem->GetDistanceMetric()->GetMethod(dm_label);


    m_min = _inNode.numberXMLParameter("min", false, 0.0, 0.0, 100000.0, "min");
    m_max = _inNode.numberXMLParameter("max", false, DBL_MAX, 0.0, DBL_MAX, "max");
    m_usePercent = _inNode.boolXMLParameter("usePercent", false, false,
                          "treat min and max as a percentage of the total number of vertices in the roadmap");
    
    double alpha = _inNode.numberXMLParameter("alpha", false, 1.0, 0.0, 100.0, "alpha");

    string policy = _inNode.stringXMLParameter("policy", true, "closest", "selection policy");
    int k = _inNode.numberXMLParameter("k", true, 1, 0, 10000, "k");
    
    cout << "Band : Params found - min = " << m_min << " | max = " << m_max << " | k = " << k << " | policy = " << policy << endl;
    
    if (policy == "closest") {
      m_policy = new ClosestPolicy(k, _debug);
    } else
    if (policy == "random") {
      m_policy = new RandomPolicy(k, _debug);
    } else
    if (policy == "RWR") {
      m_policy = new RankWeightedRandomPolicy(k, alpha, _debug);
    } else
    if (policy == "DWR") {
      m_policy = new DistanceWeightedRandomPolicy(k, alpha, _debug);
    } else
    if (policy == "preferential") {
      m_policy = new PreferentialPolicy(k, _debug);
    } else {
		cout << "policy \"" << policy << "\" is not a valid option.  Exiting..." << endl;
		exit(-1);
	}
    
    m_type = "";
  }

  virtual const std::string GetName() const {
    return Band::GetClassName();
  }
  
  static const std::string GetClassName() {
    return "Band";
  }

  virtual void PrintOptions(std::ostream& _os) const {
    //Andy G: I'm not sure what should be printed, so TODO will be placed here
    _os << this->GetClassName() << ":: TODO" << std::endl;
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
    
    sort(dist_list.begin(), dist_list.end(), compare_second<VID, double>());
    
    return dist_list;
  }
  
  bool m_debug;
  bool m_usePercent;
  double m_min;
  double m_max;
  string m_type;
  shared_ptr<DistanceMetricMethod> dmm;
  Policy* m_policy;
};

template<typename CFG, typename WEIGHT>
class DBand : public Band<CfgType, WeightType> {
  
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  DBand(XMLNodeReader& _node, MPProblem* _problem, bool debug)
    : Band<CfgType, WeightType>(_node, _problem, debug)
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
    vector< pair<VID, double> > neighbors = m_policy->SelectNeighbors(_rmp, candidate_set.begin(), candidate_set.end());
    return neighbors; 
  }

  virtual const std::string GetName() const {
    return DBand::GetClassName();
  }
  
  static const std::string GetClassName() {
    return "DBand";
  }

  virtual void PrintOptions(std::ostream& _os) const {
    //Andy G: I'm not sure what should be printed, so TODO will be placed here
    _os << this->GetClassName() << ":: TODO" << std::endl;
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
      if (getDebug()) cout << "Candidate: VID = " << (*V1).first << " | dist = " << (*V1).second << endl;
    }
    
    return candidates;
  }
  
};

template<typename CFG, typename WEIGHT>
class RBand : public Band<CfgType, WeightType> {
  
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  RBand(XMLNodeReader& _node, MPProblem* _problem, bool debug)
    : Band<CfgType, WeightType>(_node, _problem, debug)
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
    vector< pair<VID, double> > neighbors = m_policy->SelectNeighbors(_rmp, candidate_set.begin(), candidate_set.end());
    return neighbors; 
  }

  virtual const std::string GetName() const {
    return RBand::GetClassName();
  }
  
  static const std::string GetClassName() {
    return "RBand";
  }

  virtual void PrintOptions(std::ostream& _os) const {
    //Andy G: I'm not sure what should be printed, so TODO will be placed here
    _os << this->GetClassName() << ":: TODO" << std::endl;
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
  
  BandsNF(XMLNodeReader& _node, MPProblem* _problem) : NeighborhoodFinderMethod(_node, _problem) {

    m_debug = _node.boolXMLParameter("debug", false, false, "");
    
    XMLNodeReader::childiterator citr;
    for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
      if (citr->getName() == "DBand") {
        cout << "DBand found" << endl;
        Band<CFG,WEIGHT>* dband = new DBand<CFG,WEIGHT>(*citr, _problem, m_debug);
        m_bands.push_back(dband);
      } else if(citr->getName() == "RBand") {
        cout << "RBand found" << endl;
        Band<CFG,WEIGHT>* rband = new RBand<CFG,WEIGHT>(*citr, _problem, m_debug);
        m_bands.push_back(rband);
      } 
    }
  }

  BandsNF(shared_ptr<DistanceMetricMethod> _dmm, std::string _label) : NeighborhoodFinderMethod(_dmm,_label) {
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
      if ((*itr).first != INVALID_VID) {
        if (m_debug) cout << "neighbor: VID = " << (*itr).first << " | dist = " << (*itr).second << endl;
        neighbors.push_back(*itr);
      }
    }
  }
   
  sort(neighbors.begin(), neighbors.end(), compare_second<VID, double>());
  // now add VIDs from neighbors to output
  for (size_t p = 0; p < neighbors.size(); p++) {
    if (neighbors[p].first != INVALID_VID) {
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
  vector< pair< VID, double > > closest(k, make_pair(INVALID_VID, max_value));

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
 
  sort(closest.begin(), closest.end(), compare_second<VID, double>());
  EndQueryTime(); 
  EndConstructionTime();
  // now add VIDs from closest to
  for (int p = 0; p < closest.size(); p++) {
    if (closest[p].first != INVALID_VID) {
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
    vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(INVALID_VID,INVALID_VID),
    max_value));
   CFG v1 = pmpl_detail::GetCfg<InputIterator>(pMap)(V1);
    for(V2 = _in2_first; V2 != _in2_last; ++V2) {
      //marcom/08nov03 check if results in other functions is same                      
      if(*V1 == *V2)
        continue; //don't connect same                                                  
      CFG v2 = pmpl_detail::GetCfg<InputIterator>(pMap)(V2);
      double dist = dmm->Distance(_env, v1, v2);
      if(dist < kp[max_index].second) {
        kp[max_index] = make_pair(make_pair(*V1,*V2),dist);
        max_value = dist;
      
        //search for new max_index (faster O(k) than sort O(k log k) )                  
        for (size_t p = 0; p < kp.size(); ++p) {
          if (max_value < kp[p].second) {
            max_value = kp[p].second;
            max_index = p;
          }
        }
      }
    }//endfor c2                                                                  
    kall.insert(kall.end(),kp.begin(),kp.end());
  }//endfor c1                                                                    
 
  sort(kall.begin(), kall.end(), compare_second<pair<VID, VID>, double>());
  
  for (int p = 0; p < k; ++p) {
    if (kall[p].first.first != INVALID_VID && kall[p].first.second != INVALID_VID){
      *_out = kall[p].first;
      ++_out;
    }
  }
  return _out;
}


#endif //end #ifndef _BANDS_NEIGHBORHOOD_FINDER_H_
