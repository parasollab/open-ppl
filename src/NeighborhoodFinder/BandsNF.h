#ifndef BANDSNF_H_
#define BANDSNF_H_

#include "NeighborhoodFinderMethod.h"
#include "MPProblem.h"

#include <vector>
using namespace std;

class Policy;
class ClosestPolicy;
class RandomPolicy;
class PreferentialPolicy;
class RankWeightedRandomPolicy;
class DistanceWeightedRandomPolicy;

namespace pmpl_detail { //hide NeighborhoodFinderMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
    ClosestPolicy,
    RandomPolicy,
    PreferentialPolicy,
    RankWeightedRandomPolicy,
    DistanceWeightedRandomPolicy
    > PolicyList;
  
  template<typename P, typename RDMP, typename I, typename O>
      struct VirtualSelectNeighbors{
        public:
          VirtualSelectNeighbors(P* _v, RDMP* _r, I _f, I _l, O _o) : 
            m_memory(_v), m_rdmp(_r), m_first(_f), m_last(_l), m_output(_o){}

          template<typename T>
            void operator()(T& _t) {
              T* tptr = dynamic_cast<T*>(m_memory);
              if(tptr != NULL){
                tptr->SelectNeighbors(m_rdmp, m_first, m_last, m_output);
              }
            }
        private:
          P* m_memory;
          RDMP* m_rdmp;
          I m_first, m_last;
          O m_output;
      };
}

/////// Policy definitions
class Policy {
  public:
    Policy(size_t _k = 0, bool _debug = false) : m_k(_k), m_debug(_debug) {}
    virtual ~Policy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){
        typedef pmpl_detail::PolicyList PolicyList;
        boost::mpl::for_each<PolicyList>(pmpl_detail::VirtualSelectNeighbors<
            Policy, RDMP, InputIterator, OutputIterator>(this, _rmp, _first, _last, _out));
      }

  protected:
    size_t m_k;
    bool m_debug;
};

class ClosestPolicy : public Policy {
  public:
    ClosestPolicy(size_t _k = 0, bool _debug = false) : Policy(_k, _debug) { }
    virtual ~ClosestPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){
        
        if(m_debug) cout << "ClosestPolicy::SelectNeighbors()" << endl;

        typedef typename RDMP::VID VID;
        priority_queue<pair<VID, double>, vector<pair<VID, double> >, CompareSecond<VID, double> > pq;
        for(InputIterator it = _first; it != _last; it++) {
          if(pq.size() < m_k)
            pq.push(*it);
          // If better than the worst so far, replace worst so far
          else if(it->second < pq.top().second) {
            pq.pop();
            pq.push(*it);
          }
        }
        
        // Transfer k closest to vector, sorted greatest to least dist
        vector<pair<VID, double> > closest;
        while(!pq.empty()) {
          closest.push_back(pq.top());
          pq.pop();
        }
        // Reverse order
        for(typename vector<pair<VID, double> >::reverse_iterator it = closest.rbegin(); it < closest.rend(); it++) {
          *_out++ = *it;
        }
      }

};

class RandomPolicy : public Policy {
  public:
    RandomPolicy(size_t _k = 0, bool _debug = false) : Policy(_k, _debug) { }
    virtual ~RandomPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){
        
        if(m_debug) cout << "RandomPolicy::SelectNeighbors()" << endl;

        vector<typename RDMP::VID> neighbors;

        size_t maxIter = m_k;
        if(_last - _first < (int)maxIter)
          maxIter = _last - _first;

        for(size_t i = 0; i < maxIter; i++) {
          InputIterator p;

          // select random candidate that hasn't been added yet
          bool done = false;
          while(!done) {
            
            size_t id = (size_t)(LRand()%(_last - _first));
            p = _first + id;

            typename RDMP::VID v = p->first;        
            if(m_debug) cout << "\tchecking id = " << id << ", VID = " << v;

            // check to see if this has been added
            if(find(neighbors.begin(), neighbors.end(), v) == neighbors.end()){
              if(m_debug) cout << " | added!" << endl;
              neighbors.push_back(v);
              *_out++ = *p;
              break;
            }
          }
        }
      }

};

class PreferentialPolicy : public Policy {
  public:
    PreferentialPolicy(size_t _k = 0, bool _debug = false) : Policy(_k, _debug) {}
    virtual ~PreferentialPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if(m_debug) cout << "PreferentialPolicy::SelectNeighbors()" << endl;

        vector<typename RDMP::VID> neighbors;

        size_t found = 0;
        size_t maxIter = m_k;
        if(_last - _first < (int)maxIter)
          maxIter = _last - _first;

        size_t setDegree = CandidateSetDegree(_rmp, _first, _last);

        while(found < maxIter) {
          // iterate through candidate set, adding as neighbor with probability = PrefProb(_rmp, v, n)
          for(InputIterator itr = _first; itr != _last; ++itr) {
            double drand = DRand();
            pair<typename RDMP::VID, double> p = *itr;
            typename RDMP::VID v = p.first;
            double prob = PrefProb(_rmp, v, _last - _first, setDegree);
            if(m_debug) cout << "found = " << found << ", VID = " << v << ", drand = " << drand << ", prob = " << prob;
            if(drand < prob) {
              if(m_debug) cout << " ||| ";

              if(find(neighbors.begin(), neighbors.end(), v) == neighbors.end()){
                neighbors.push_back(v);
                *_out++ = p;
                if (m_debug) cout << " added!";
                found++;
              }
            }
            if(m_debug) cout << endl;
            if(found == maxIter)
              break;
          }
        }  
      }

    //////////////////////
    // Probability function
    template<typename RDMP>
      double PrefProb(RDMP* _rm, typename RDMP::VID _vid, size_t _n, size_t _totDegree) {
        size_t candidateDegree = _rm->m_pRoadmap->get_degree(_vid);
        size_t totalDegree = _totDegree;
        if (_totDegree == (size_t)-1) totalDegree = _rm->m_pRoadmap->get_num_edges(); 
        if (m_debug) cout << "PrefProb(" << _vid << ", " << _n << ") = " << 1 + candidateDegree << " / " << _n + totalDegree << endl;
        return ((double)(1 + candidateDegree) / (double)(_n + totalDegree));
      }

    //////////////////////
    // Get the total degree of the candidate set
    template<typename RDMP, typename InputIterator>
      size_t CandidateSetDegree(RDMP* _rm, InputIterator _first, InputIterator _last) {
        size_t totalDegree = 0;
        for (InputIterator itr = _first; itr != _last; ++itr) {
          size_t candidateDegree = _rm->m_pRoadmap->get_degree(itr->first);
          totalDegree += candidateDegree;
          if (m_debug) cout << "CandidateSetDegree += " << candidateDegree << endl;
        }
        return totalDegree;
      }
};

class RankWeightedRandomPolicy : public Policy {
  public:  
    RankWeightedRandomPolicy(size_t _k = 0, double _alpha = 0.0, bool _debug = false) : Policy(_k, _debug) { m_alpha = _alpha; }
    virtual ~RankWeightedRandomPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if (m_debug) cout << "RankWeightedRandomPolicy::SelectNeighbors()" << endl;

        vector<typename RDMP::VID> neighbors;

        size_t maxIter = m_k;
        if (_last - _first < (int)maxIter)
          maxIter = _last - _first;

        double maxRank = (_last - _first);

        if (m_debug) cout << "\t\tmaxRank = " << maxRank << endl;

        for (size_t i = 0; i < maxIter; i++) {
          pair<typename RDMP::VID, double> p;

          // select random candidate that hasn't been added yet
          bool done = false;
          while (!done) {

            size_t id = (size_t)(LRand()%(_last - _first));
            p = *(_first + id);

            // check to see if this VID has been added
            if(find(neighbors.begin(), neighbors.end(), p.first) == neighbors.end()){
              // if it hasn't been added, add it with some probability
              double prob = pow((maxRank - id) / (maxRank), m_alpha);
              double roll = DRand();

              // if we are taking less than K (every neighbor in the candidate set), set prob to 1
              if (maxIter < m_k) {
                prob = 1.0;
              }

              if (m_debug) cout << "\t\t\trank = " << id << ", prob = " << prob << ", alpha = " << m_alpha << ", roll = " << roll;

              if (roll < prob) {
                neighbors.push_back(p.first);
                *_out++ = p;
                if (m_debug) cout << " | added";
              }
              else
                done = false;

              if (m_debug) cout << endl;
            }
          }  
        }
      }

  private:
    double m_alpha;
};

class DistanceWeightedRandomPolicy : public Policy {
  public:
    DistanceWeightedRandomPolicy(size_t _k = 0, double _alpha = 0, bool _debug = false) : Policy(_k, _debug) { m_alpha = _alpha; }
    virtual ~DistanceWeightedRandomPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if (m_debug) cout << "DistanceWeightedRandomPolicy::SelectNeighbors()" << endl;

        vector<typename RDMP::VID> neighbors;

        size_t maxIter = m_k;
        if (_last - _first < (int)maxIter)
          maxIter = _last - _first;

        reverse_iterator<InputIterator> riter = reverse_iterator<InputIterator>(_last);
        double maxDist = (*riter).second;
        double minDist = (*_first).second;

        if (m_debug) cout << "\t\tminDist = " << minDist << ", maxDist = " << maxDist << endl;

        for (size_t i = 0; i < maxIter; i++) {

          pair<typename RDMP::VID, double> p;

          // select random candidate that hasn't been added yet
          bool done = false;
          while (!done) {

            size_t id = (size_t)(LRand()%(_last - _first));
            p = *(_first + id);

            // check to see if this VID has been added
            if(find(neighbors.begin(), neighbors.end(), p.first) == neighbors.end()){
              // if it hasn't been added, add it with some probability
              double prob = pow((maxDist - p.second) / (maxDist - minDist), m_alpha);
              double roll = DRand();

              // if we are taking less than K (every neighbor in the candidate set), set prob to 1
              if (maxIter < m_k) {
                prob = 1.0;
              }

              if (m_debug) cout << "\t\t\tdist = " << p.second << ", prob = " << prob << ", roll = " << roll;

              if (roll < prob) {
                neighbors.push_back(p.first);
                *_out++ = p;
                if (m_debug) cout << " | added";
              }
              else
                done = false;

              if (m_debug) cout << endl;
            }
          }  
        }
      }

  private:
    double m_alpha;
};


/////// Band definitions
class Band : public MPBaseObject {
  public:
    Band(string _dmm = "", string _label = "", MPProblem* _mp = NULL) : MPBaseObject(_mp, _label), m_dmLabel(_dmm) {
      SetName("Band");
    }

    Band(XMLNodeReader& _node, MPProblem* _problem): MPBaseObject(_node, _problem) {  
      SetName("Band");
      m_dmLabel = _node.stringXMLParameter("dmMethod", true, "default", "Distance Metric Method");

      m_min = _node.numberXMLParameter("min", false, 0.0, 0.0, 100000.0, "min");
      m_max = _node.numberXMLParameter("max", false, DBL_MAX, 0.0, DBL_MAX, "max");
      m_usePercent = _node.boolXMLParameter("usePercent", false, false,
          "treat min and max as a percentage of the total number of vertices in the roadmap");

      double alpha = _node.numberXMLParameter("alpha", false, 1.0, 0.0, 100.0, "alpha");

      string policy = _node.stringXMLParameter("policy", true, "closest", "selection policy");
      size_t k = _node.numberXMLParameter("k", true, 1, 0, 10000, "k");

      if (policy == "closest") {
        m_policy = new ClosestPolicy(k, m_debug);
      } 
      else if (policy == "random") {
        m_policy = new RandomPolicy(k, m_debug);
      } 
      else if (policy == "RWR") {
        m_policy = new RankWeightedRandomPolicy(k, alpha, m_debug);
      } 
      else if (policy == "DWR") {
        m_policy = new DistanceWeightedRandomPolicy(k, alpha, m_debug);
      } 
      else if (policy == "preferential") {
        m_policy = new PreferentialPolicy(k, m_debug);
      } 
      else {
        cout << "policy \"" << policy << "\" is not a valid option.  Exiting..." << endl;
        exit(-1);
      }
    }

    virtual void PrintOptions(ostream& _os) const {
      _os << this->GetName() << ":: TODO" << std::endl;
    }

    // given initial set V (_first --> _last), and CFG v1, return V_n.
    template<typename RDMP, typename InputIterator>
      vector< pair<typename RDMP::VID, double> >
      GetNeighbors(RDMP* _rmp,
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg);

  protected:
    template<typename RDMP, typename InputIterator>
      vector< pair<typename RDMP::VID, double> >
      GetCandidateSet(RDMP* _rmp, 
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg){
        if (m_debug) cout << "Band::GetCandidateSet()" << endl;

        // this will be overwritten by extending classes
        vector< pair<typename RDMP::VID, double> > candidates;
        return candidates;
      }


    template<typename RDMP, typename InputIterator>
      vector< pair<typename RDMP::VID, double> >
      GetDistList(RDMP* _rmp,
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg) {  
        if (m_debug) cout << "Band::GetDistList()" << endl;

        typedef typename RDMP::VID VID;
        typedef typename RDMP::CfgType CFG;
        typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
        typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

        Environment* env = _rmp->GetEnvironment();
        DistanceMetric::DistanceMetricPointer dmm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel);
        RoadmapGraphType* map = _rmp->m_pRoadmap;

        vector< pair<VID, double> > distList;

        InputIterator V1;

        // compute sorted neighbor list
        for (V1 = _first; V1 != _last; ++V1) {
          CFG v1 = GetCfg()(map, V1);

          if(v1 == _cfg)
            continue; //don't connect same

          double dist = dmm->Distance(env, _cfg, v1);
          pair<VID, double> p = make_pair(*V1, dist);
          distList.push_back(p);
        }

        sort(distList.begin(), distList.end(), CompareSecond<VID, double>());

        return distList;
      }

    bool m_debug;
    bool m_usePercent;
    double m_min;
    double m_max;
    string m_type;
    string m_dmLabel;
    Policy* m_policy;
};

class DBand : public Band {
  public:
    DBand(XMLNodeReader& _node, MPProblem* _problem) : Band(_node, _problem){ 
      SetName("DBand");
    }

    template<typename RDMP, typename InputIterator>
      vector<pair<typename RDMP::VID, double> >
      GetNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg){
        if (m_debug) cout << "DBand::GetNeighbors()" << endl;
        // get candidate set
        vector< pair<typename RDMP::VID, double> > candidateSet = GetCandidateSet(_rmp, _first, _last, _cfg);
        if (m_debug) cout << "  num_candidates = " << candidateSet.size() << endl;

        // get neighbors from candidate set using policy
        vector<pair<typename RDMP::VID, double> > neighborSet;
        m_policy->SelectNeighbors(_rmp, candidateSet.begin(), candidateSet.end(), back_inserter(neighborSet));
        return neighborSet;
      }

  private:
    template <typename RDMP, typename InputIterator>
      vector< pair<typename RDMP::VID, double> >
      GetCandidateSet(RDMP* _rmp, 
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg) {
        if (m_debug) cout << "DBand::GetCandidateSet()" << endl;

        // obtain sorted distance list
        vector< pair<typename RDMP::VID, double> > distList = GetDistList(_rmp, _first, _last, _cfg);
        vector< pair<typename RDMP::VID, double> > candidates;

        double min, max;

        min = m_min;
        max = m_max;

        // iterate through list, return all (VID, dist) pairs that are between min and max
        typename vector< pair<typename RDMP::VID, double> >::iterator V1;
        if (m_debug) cout << "\tchecking candidates (min = " << min << ", max = " << max << ") for CFG = " << _cfg << ": " << endl;
        for (V1 = distList.begin(); V1 != distList.end(); ++V1) {
          double dist = (*V1).second;
          if (m_debug) cout << "\t\t(" << (*V1).first << ", " << (*V1).second << ") ";
          if (min <= dist && dist < max) {
            if (m_debug) cout << "added";
            candidates.push_back(*V1);
          }
          if (m_debug) cout << endl;
          if (m_debug) cout << "Candidate: VID = " << (*V1).first << " | dist = " << (*V1).second << endl;
        }

        return candidates;
      }
};

class RBand : public Band {
  public:
    RBand(XMLNodeReader& _node, MPProblem* _problem) : Band(_node, _problem) { 
      SetName("RBand");
    }

    template<typename RDMP, typename InputIterator>
      vector< pair<typename RDMP::VID, double> >
      GetNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg) {
        if (m_debug) cout << "RBand::GetNeighbors()" << endl;
        // get candidate set
        vector<pair<typename RDMP::VID, double> > candidateSet = GetCandidateSet(_rmp, _first, _last, _cfg);
        if (m_debug) cout << "  num_candidates = " << candidateSet.size() << endl;

        // get neighbors from candidate set using policy
        vector<pair<typename RDMP::VID, double> > neighborSet;
        m_policy->SelectNeighbors(_rmp, candidateSet.begin(), candidateSet.end(), back_inserter(neighborSet));
        return neighborSet;
      }

  private:
    template<typename RDMP, typename InputIterator>
      vector< pair<typename RDMP::VID, double> >
      GetCandidateSet(RDMP* _rmp, 
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg) {
        if (m_debug) cout << "RBand::GetCandidateSet()" << endl;

        // obtain sorted distance list
        vector< pair<typename RDMP::VID, double> > distList = GetDistList(_rmp, _first, _last, _cfg);
        vector< pair<typename RDMP::VID, double> > candidates;

        double min, max;

        if (m_usePercent) {
          min = m_min * (_last - _first);
          max = m_max * (_last - _first);
        } else {
          min = m_min;
          max = m_max;
        }

        // iterate through list, return all (VID, dist) pairs that are between min and max
        double rank = 0;
        typename vector< pair<typename RDMP::VID, double> >::iterator V1;
        if (m_debug) cout << "\tchecking candidates (min = " << min << ", max = " << max << ") for CFG = " << _cfg << ": " << endl;
        for (V1 = distList.begin(); V1 != distList.end(); ++V1) {
          if (m_debug) cout << "\t\t(" << (*V1).first << ", " << (*V1).second << ") ";
          if (min <= rank && rank < max) {
            if (m_debug) cout << "added";
            candidates.push_back(*V1);
          }
          if (m_debug) cout << endl;
          rank++;
        }

        return candidates;
      }
};


class BandsNF: public NeighborhoodFinderMethod {
  public:
    BandsNF(string _dmm = "", string _label = "", MPProblem* _mp = NULL) : NeighborhoodFinderMethod(_dmm, _label, _mp) {
      SetName("BandsNF");
    }

    BandsNF(XMLNodeReader& _node, MPProblem* _problem) : NeighborhoodFinderMethod(_node, _problem) {
      SetName("BandsNF");
      XMLNodeReader::childiterator citr;
      for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
        if (citr->getName() == "DBand") {
          Band* dband = new DBand(*citr, _problem);
          m_bands.push_back(dband);
        } 
        else if(citr->getName() == "RBand") {
          Band* rband = new RBand(*citr, _problem);
          m_bands.push_back(rband);
        } 
      }
    }

    virtual void PrintOptions(std::ostream& _os) const {
      NeighborhoodFinderMethod::PrintOptions(_os);
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
    vector<Band*> m_bands;
};

// Returns all nodes within radius from _cfg
template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator 
BandsNF::KClosest(RDMP* _roadmap, InputIterator _first, InputIterator _last, 
    typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out) {
  
  typedef typename RDMP::VID VID;
  typedef typename RDMP::CfgType CFG;
  typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
  typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  IncrementNumQueries();
  StartTotalTime();
  StartQueryTime();

  vector< pair<VID, double> > neighbors;

  // iterate through bands
  typename vector<Band*>::iterator bandIT;
  for (bandIT = m_bands.begin(); bandIT != m_bands.end(); ++bandIT) {
    if (m_debug) cout << "Finding Neighbors for Band" << endl;

    vector< pair<VID, double> > bandNeighbors;

    if((*bandIT)->GetName() == "DBand"){
      bandNeighbors = ((DBand*)*bandIT)->GetNeighbors(_roadmap, _first, _last, _cfg);
    }
    else if((*bandIT)->GetName() == "RBand"){
      bandNeighbors = ((RBand*)*bandIT)->GetNeighbors(_roadmap, _first, _last, _cfg);
    }
    
    typename vector< pair<VID, double> >::iterator itr;
    for (itr = bandNeighbors.begin(); itr != bandNeighbors.end(); ++itr) {
      if ((*itr).first != INVALID_VID) {
        if (m_debug) cout << "neighbor: VID = " << (*itr).first << " | dist = " << (*itr).second << endl;
        neighbors.push_back(*itr);
      }
    }
  }

  sort(neighbors.begin(), neighbors.end(), CompareSecond<VID, double>());
  
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

// Returns all pairs within radius
template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator 
BandsNF::KClosestPairs(RDMP* _roadmap,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    size_t _k, OutputIterator _out) {
  cerr << "ERROR:: BandsNF::KClosestPairs is not yet implemented. Exiting" << endl;
  exit(1);
}

#endif //end #ifndef
