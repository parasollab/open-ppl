#ifndef PMPL_BANDS_NF_H_
#define PMPL_BANDS_NF_H_

#include "NeighborhoodFinderMethod.h"
#include "MPProblem/MPProblem.h"

#include <vector>

class Policy;
class ClosestPolicy;
class RandomPolicy;
class PreferentialPolicy;
class RankWeightedRandomPolicy;
class DistanceWeightedRandomPolicy;

namespace pmpl_detail {

typedef boost::mpl::list<
  ClosestPolicy,
  RandomPolicy,
  PreferentialPolicy,
  RankWeightedRandomPolicy,
  DistanceWeightedRandomPolicy
    > PolicyList;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
/// @tparam P Policy type
/// @tparam RDMP Roadamp type
/// @tparam I Input iterator type
/// @tparam O Output iterator type
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class ClosestPolicy : public Policy {
  public:
    ClosestPolicy(size_t _k = 0, bool _debug = false) : Policy(_k, _debug) { }
    virtual ~ClosestPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if(m_debug) std::cout << "ClosestPolicy::SelectNeighbors()" << std::endl;

        std::priority_queue<Neighbor> pq;

        for(InputIterator it = _first; it != _last; it++) {
          if(pq.size() < m_k)
            pq.push(*it);
          // If better than the worst so far, replace worst so far
          else if((it->distance) < pq.top().distance) {
            pq.pop();
            pq.push(*it);
          }
        }

        // Transfer k closest to vector, sorted greatest to least dist
        std::vector<Neighbor> closest;
        while(!pq.empty()) {
          closest.push_back(pq.top());
          pq.pop();
        }
        // Reverse order
        for(auto it = closest.rbegin(); it < closest.rend(); it++) {
          *_out++ = *it;
        }
      }

};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class RandomPolicy : public Policy {
  public:
    RandomPolicy(size_t _k = 0, bool _debug = false) : Policy(_k, _debug) { }
    virtual ~RandomPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if(m_debug) std::cout << "RandomPolicy::SelectNeighbors()" << std::endl;

        std::vector<typename RDMP::VID> neighbors;

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

            typename RDMP::VID v = p->target;
            if(m_debug) std::cout << "\tchecking id = " << id << ", VID = " << v;

            // check to see if this has been added
            if(find(neighbors.begin(), neighbors.end(), v) == neighbors.end()){
              if(m_debug) std::cout << " | added!" << std::endl;
              neighbors.push_back(v);
              *_out++ = *p;
              break;
            }
          }
        }
      }

};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class PreferentialPolicy : public Policy {
  public:
    PreferentialPolicy(size_t _k = 0, bool _debug = false) : Policy(_k, _debug) {}
    virtual ~PreferentialPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if(m_debug) std::cout << "PreferentialPolicy::SelectNeighbors()" << std::endl;

        std::vector<typename RDMP::VID> neighbors;

        size_t found = 0;
        size_t maxIter = m_k;
        if(_last - _first < (int)maxIter)
          maxIter = _last - _first;

        size_t setDegree = CandidateSetDegree(_rmp, _first, _last);

        while(found < maxIter) {
          // iterate through candidate set, adding as neighbor with probability = PrefProb(_rmp, v, n)
          for(InputIterator itr = _first; itr != _last; ++itr) {
            double drand = DRand();
            const Neighbor& p = *itr;
            typename RDMP::VID v = p.target;
            double prob = PrefProb(_rmp, v, _last - _first, setDegree);
            if(m_debug) std::cout << "found = " << found << ", VID = " << v << ", drand = " << drand << ", prob = " << prob;
            if(drand < prob) {
              if(m_debug) std::cout << " ||| ";

              if(find(neighbors.begin(), neighbors.end(), v) == neighbors.end()){
                neighbors.push_back(v);
                *_out++ = p;
                if (m_debug) std::cout << " added!";
                found++;
              }
            }
            if(m_debug) std::cout << std::endl;
            if(found == maxIter)
              break;
          }
        }
      }

    //////////////////////
    // Probability function
    template<typename RDMP>
      double PrefProb(RDMP* _rm, typename RDMP::VID _vid, size_t _n, size_t _totDegree) {
        size_t candidateDegree = _rm->GetGraph()->get_degree(_vid);
        size_t totalDegree = _totDegree;
        if (_totDegree == (size_t)-1) totalDegree = _rm->GetGraph()->get_num_edges();
        if (m_debug) std::cout << "PrefProb(" << _vid << ", " << _n << ") = " << 1 + candidateDegree << " / " << _n + totalDegree << std::endl;
        return ((double)(1 + candidateDegree) / (double)(_n + totalDegree));
      }

    //////////////////////
    // Get the total degree of the candidate set
    template<typename RDMP, typename InputIterator>
      size_t CandidateSetDegree(RDMP* _rm, InputIterator _first, InputIterator _last) {
        size_t totalDegree = 0;
        for (InputIterator itr = _first; itr != _last; ++itr) {
          size_t candidateDegree = _rm->GetGraph()->get_degree(itr->target);
          totalDegree += candidateDegree;
          if (m_debug) std::cout << "CandidateSetDegree += " << candidateDegree << std::endl;
        }
        return totalDegree;
      }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class RankWeightedRandomPolicy : public Policy {
  public:
    RankWeightedRandomPolicy(size_t _k = 0, double _alpha = 0.0, bool _debug = false) : Policy(_k, _debug) { m_alpha = _alpha; }
    virtual ~RankWeightedRandomPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if (m_debug) std::cout << "RankWeightedRandomPolicy::SelectNeighbors()" << std::endl;

        std::vector<typename RDMP::VID> neighbors;

        size_t maxIter = m_k;
        if (_last - _first < (int)maxIter)
          maxIter = _last - _first;

        double maxRank = (_last - _first);

        if (m_debug) std::cout << "\t\tmaxRank = " << maxRank << std::endl;

        for (size_t i = 0; i < maxIter; i++) {

          // select random candidate that hasn't been added yet
          bool done = false;
          while (!done) {

            size_t id = (size_t)(LRand()%(_last - _first));
            const auto& p = *(_first + id);

            // check to see if this VID has been added
            if(find(neighbors.begin(), neighbors.end(), p.target) == neighbors.end()){
              // if it hasn't been added, add it with some probability
              double prob = pow((maxRank - id) / (maxRank), m_alpha);
              double roll = DRand();

              // if we are taking less than K (every neighbor in the candidate set), set prob to 1
              if (maxIter < m_k) {
                prob = 1.0;
              }

              if (m_debug) std::cout << "\t\t\trank = " << id << ", prob = " << prob << ", alpha = " << m_alpha << ", roll = " << roll;

              if (roll < prob) {
                neighbors.push_back(p.target);
                *_out++ = p;
                if (m_debug) std::cout << " | added";
              }
              else
                done = false;

              if (m_debug) std::cout << std::endl;
            }
          }
        }
      }

  private:
    double m_alpha;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class DistanceWeightedRandomPolicy : public Policy {
  public:
    DistanceWeightedRandomPolicy(size_t _k = 0, double _alpha = 0, bool _debug = false) : Policy(_k, _debug) { m_alpha = _alpha; }
    virtual ~DistanceWeightedRandomPolicy() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      void SelectNeighbors(RDMP* _rmp, InputIterator _first, InputIterator _last, OutputIterator _out){

        if (m_debug) std::cout << "DistanceWeightedRandomPolicy::SelectNeighbors()" << std::endl;

        std::vector<typename RDMP::VID> neighbors;

        size_t maxIter = m_k;
        if (_last - _first < (int)maxIter)
          maxIter = _last - _first;

        std::reverse_iterator<InputIterator> riter(_last);
        double maxDist = (*riter).distance;
        double minDist = (*_first).distance;

        if (m_debug) std::cout << "\t\tminDist = " << minDist << ", maxDist = " << maxDist << std::endl;

        for (size_t i = 0; i < maxIter; i++) {

          // select random candidate that hasn't been added yet
          bool done = false;
          while (!done) {

            size_t id = (size_t)(LRand()%(_last - _first));
            const auto& p = *(_first + id);

            // check to see if this VID has been added
            if(find(neighbors.begin(), neighbors.end(), p.target) == neighbors.end()){
              // if it hasn't been added, add it with some probability
              double prob = pow((maxDist - p.distance) / (maxDist - minDist), m_alpha);
              double roll = DRand();

              // if we are taking less than K (every neighbor in the candidate set), set prob to 1
              if (maxIter < m_k) {
                prob = 1.0;
              }

              if (m_debug) std::cout << "\t\t\tdist = " << p.distance << ", prob = " << prob << ", roll = " << roll;

              if (roll < prob) {
                neighbors.push_back(p.target);
                *_out++ = p;
                if (m_debug) std::cout << " | added";
              }
              else
                done = false;

              if (m_debug) std::cout << std::endl;
            }
          }
        }
      }

  private:
    double m_alpha;
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class Band : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::GraphType           GraphType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    Band(std::string _dmm = "", std::string _label = "") :
        MPBaseObject<MPTraits>(_label), m_dmLabel(_dmm) {
      this->SetName("Band");
    }

    Band(XMLNode& _node): MPBaseObject<MPTraits>(_node) {
      this->SetName("Band");
      m_dmLabel = _node.Read("dmLabel", true, "default", "Distance Metric Method");

      m_min = _node.Read("min", false, 0.0, 0.0, 100000.0, "min");
      m_max = _node.Read("max", false, MAX_DBL, 0.0, MAX_DBL, "max");
      m_usePercent = _node.Read("usePercent", false, false,
          "treat min and max as a percentage of the total number of vertices in the roadmap");

      double alpha = _node.Read("alpha", false, 1.0, 0.0, 100.0, "alpha");

      std::string policy = _node.Read("policy", true, "closest", "selection policy");
      size_t k = _node.Read("k", true, 1, 0, 10000, "k");

      if (policy == "closest") {
        m_policy = std::unique_ptr<Policy>(new ClosestPolicy(k, m_debug));
      }
      else if (policy == "random") {
        m_policy = std::unique_ptr<Policy>(new RandomPolicy(k, m_debug));
      }
      else if (policy == "RWR") {
        m_policy = std::unique_ptr<Policy>(new RankWeightedRandomPolicy(k, alpha, m_debug));
      }
      else if (policy == "DWR") {
        m_policy = std::unique_ptr<Policy>(new DistanceWeightedRandomPolicy(k, alpha, m_debug));
      }
      else if (policy == "preferential") {
        m_policy = std::unique_ptr<Policy>(new PreferentialPolicy(k, m_debug));
      }
      else {
        std::cout << "policy \"" << policy << "\" is not a valid option.  Exiting..." << std::endl;
        exit(-1);
      }
    }

    virtual void Print(ostream& _os) const {
      _os << this->GetNameAndLabel() << ":: TODO" << std::endl;
    }

    // given initial set V (_first --> _last), and CFG v1, return V_n.
    template<typename InputIterator, typename RoadmapType, typename CfgType>
      std::vector<Neighbor>
      GetNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, const CfgType& _cfg);

  protected:
    template <typename InputIterator, typename RoadmapType, typename CfgType>
    std::vector<Neighbor>
    GetCandidateSet(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, const CfgType& _cfg){
      if (m_debug) std::cout << "Band<MPTraits>::GetCandidateSet()" << std::endl;

      // this will be overwritten by extending classes
      std::vector<Neighbor> candidates;
      return candidates;
    }

    template<typename InputIterator, typename RoadmapType, typename CfgType>
    std::vector<Neighbor>
    GetDistList(RoadmapType* _rmp, InputIterator _first,
                InputIterator _last, const CfgType& _cfg) {
      typedef typename RoadmapType::VID VID;

      if (m_debug)
        std::cout << "Band<MPTraits>:::GetDistList()" << std::endl;

      typename RoadmapType::GraphType* map = _rmp->GetGraph();
      auto dmm = this->GetDistanceMetric(this->m_dmLabel);

      std::vector<Neighbor> distList;

      // compute sorted neighbor list
      for (InputIterator V1 = _first; V1 != _last; ++V1) {
        CfgType v1 = map->GetVertex(V1); //same as ??? map->GetCfg(V1);

        if(v1 == _cfg)
          continue; //don't connect same

        double dist = dmm->Distance(_cfg, v1);
        distList.emplace_back(map->GetVID(V1), dist);
      }

      std::sort(distList.begin(), distList.end());

      return distList;
    }

  protected:

    bool m_debug;
    bool m_usePercent;
    double m_min;
    double m_max;
    std::string m_type;
    std::string m_dmLabel;
    std::unique_ptr<Policy> m_policy;

};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DBand : public Band<MPTraits> {

  public:

    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::GraphType           GraphType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    DBand(XMLNode& _node) : Band<MPTraits>(_node){
      this->SetName("DBand");
    }

    template<typename InputIterator, typename RoadmapType, typename CfgType>
    std::vector<Neighbor>
    GetNeighbors(RoadmapType* _rmp, InputIterator _first, InputIterator _last,
                 const CfgType& _cfg) {
      typedef typename RoadmapType::VID VID;

      if (this->m_debug)
        std::cout << "DBand<MPTraits>::GetNeighbors()" << std::endl;

      // get candidate set
      std::vector<Neighbor> candidateSet = GetCandidateSet(_rmp, _first, _last, _cfg);
      if (this->m_debug) std::cout << "  num_candidates = " << candidateSet.size() << std::endl;

      // get neighbors from candidate set using policy
      std::vector<Neighbor> neighborSet;
      this->m_policy->SelectNeighbors(_rmp, candidateSet.begin(), candidateSet.end(), back_inserter(neighborSet));
      return neighborSet;
    }

  private:
    template <typename InputIterator, typename RoadmapType, typename CfgType>
    std::vector<Neighbor>
    GetCandidateSet(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, const CfgType& _cfg) {
      typedef typename RoadmapType::VID VID;

      if (this->m_debug)
        std::cout << "DBand<MPTraits>::GetCandidateSet()" << std::endl;

      // obtain sorted distance list
      std::vector<Neighbor> distList = this->GetDistList(_rmp, _first, _last, _cfg);
      std::vector<Neighbor> candidates;

      double min, max;

      min = this->m_min;
      max = this->m_max;

      // iterate through list, return all (VID, dist) pairs that are between min and max
      typename std::vector<Neighbor>::iterator V1;
      if (this->m_debug)
        std::cout << "\tchecking candidates (min = " << min << ", max = " << max
             << ") for CFG = " << _cfg << ": " << std::endl;
      for (V1 = distList.begin(); V1 != distList.end(); ++V1) {
        double dist = (*V1).distance;
        if (this->m_debug)
          std::cout << "\t\t(" << (*V1).target << ", " << (*V1).distance << ") ";
        if (min <= dist && dist < max) {
          if (this->m_debug) std::cout << "added";
          candidates.push_back(*V1);
        }
        if (this->m_debug) {
          std::cout << std::endl;
          std::cout << "Candidate: VID = " << (*V1).target << " | dist = "
               << (*V1).distance << std::endl;
        }
      }

      return candidates;
    }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RBand : public Band<MPTraits> {

  public:

    typedef typename MPTraits::CfgType     CfgType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    RBand(XMLNode& _node) : Band<MPTraits> (_node) {
      this->SetName("RBand");
    }

    template<typename InputIterator, typename RoadmapType, typename CfgType>
    std::vector<Neighbor>
    GetNeighbors(RoadmapType* _rmp, InputIterator _first, InputIterator _last, const CfgType& _cfg) {
      typedef typename RoadmapType::VID VID;

      if (this->m_debug)
        std::cout << "RBand<MPTraits>::GetNeighbors()" << std::endl;

      // get candidate set
      std::vector<Neighbor> candidateSet = GetCandidateSet(_rmp, _first, _last, _cfg);
      if (this->m_debug) std::cout << "  num_candidates = " << candidateSet.size() << std::endl;

      // get neighbors from candidate set using policy
      std::vector<Neighbor> neighborSet;
      this->m_policy->SelectNeighbors(_rmp, candidateSet.begin(), candidateSet.end(), back_inserter(neighborSet));
      return neighborSet;
    }

  private:
    template<typename InputIterator, typename RoadmapType, typename CfgType>
    std::vector<Neighbor>
    GetCandidateSet(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, const CfgType& _cfg) {
      typedef typename RoadmapType::VID VID;

      if (this->m_debug)
        std::cout << "RBand<MPTraits>::GetCandidateSet()" << std::endl;

      // obtain sorted distance list
      std::vector<Neighbor> distList = this->GetDistList(_rmp, _first, _last, _cfg);
      std::vector<Neighbor> candidates;

      double min, max;

      if (this->m_usePercent) {
        min = this->m_min * distance(_first, _last);
        max = this->m_max * distance(_first, _last);
      } else {
        min = this->m_min;
        max = this->m_max;
      }

      // iterate through list, return all (VID, dist) pairs that are between min and max
      double rank = 0;
      typename std::vector<Neighbor>::iterator V1;
      if (this->m_debug)
        std::cout << "\tchecking candidates (min = " << min << ", max = " << max
             << ") for CFG = " << _cfg << ": " << std::endl;
      for (V1 = distList.begin(); V1 != distList.end(); ++V1) {
        if (this->m_debug)
          std::cout << "\t\t(" << (*V1).target << ", " << (*V1).distance << ") ";
        if (min <= rank && rank < max) {
          if (this->m_debug)
            std::cout << "added";
          candidates.push_back(*V1);
        }
        if (this->m_debug)
          std::cout << std::endl;
        rank++;
      }

      return candidates;
    }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BandsNF: public NeighborhoodFinderMethod<MPTraits> {

  public:

    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::GraphType           GraphType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    BandsNF() : NeighborhoodFinderMethod<MPTraits>() {
      this->SetName("BandsNF");
    }

    BandsNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
      this->SetName("BandsNF");
      for(auto& child : _node) {
        if(child.Name() == "DBand") {
          this->m_bands.emplace_back(new DBand<MPTraits>(child));
        }
        else if(child.Name() == "RBand") {
          this->m_bands.emplace_back(new RBand<MPTraits>(child));
        }
      }
    }

    virtual void Print(std::ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
    }

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out) {
      throw NotImplementedException(WHERE);
    }

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out) {
      throw NotImplementedException(WHERE);
    }

  private:
    std::vector<std::unique_ptr<Band<MPTraits>>> m_bands;
};

// Returns all nodes within radius from _cfg
template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BandsNF<MPTraits>::
FindNeighbors(RoadmapType* _roadmap,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  std::vector<Neighbor> neighbors;

  // iterate through bands
  for(auto bandIT = m_bands.begin(); bandIT != m_bands.end(); ++bandIT) {
    if (this->m_debug) std::cout << "Finding Neighbors for Band" << std::endl;

    std::vector<Neighbor> bandNeighbors;

    if((*bandIT)->GetName() == "DBand"){
      bandNeighbors = ((DBand<MPTraits>*)(*bandIT).get())->GetNeighbors(_roadmap, _first, _last, _cfg);
    }
    else if((*bandIT)->GetName() == "RBand"){
      bandNeighbors = ((RBand<MPTraits>*)(*bandIT).get())->GetNeighbors(_roadmap, _first, _last, _cfg);
    }

    for (typename std::vector<Neighbor>::iterator itr = bandNeighbors.begin(); itr != bandNeighbors.end(); ++itr) {
      if ((*itr).target != INVALID_VID) {
        if (this->m_debug) std::cout << "neighbor: VID = " << (*itr).target << " | dist = " << (*itr).distance << std::endl;
        neighbors.push_back(*itr);
      }
    }

  }

  std::sort(neighbors.begin(), neighbors.end());

  // now add VIDs from neighbors to output
  for(typename std::vector<Neighbor>::iterator itr = neighbors.begin();
      itr != neighbors.end(); ++itr) {
    if ((*itr).target != INVALID_VID) {
      if(this->m_debug)
        std::cout << "\tVID = " << (*itr).target << " | dist = " << (*itr).distance << std::endl;

      *_out = *itr;
      ++_out;
    }
  }

  return _out;
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BandsNF<MPTraits>::FindNeighborPairs(RoadmapType* _roadmap,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

#endif
