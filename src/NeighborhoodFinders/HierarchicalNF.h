#ifndef HIERARCHICALNF_H_
#define HIERARCHICALNF_H_

#include "NeighborhoodFinderMethod.h"
#include "BruteForceNF.h"

template<class MPTraits>
class HierarchicalNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    HierarchicalNF(string _dmLabel1 = "", string _dmLabel2 = "", size_t _k1 = 10, size_t _k2 = 5, bool _unconnected = false)
      : NeighborhoodFinderMethod<MPTraits>(),
      m_nf1(_dmLabel1, _unconnected, _k1),
      m_nf2(_dmLabel2, _unconnected, _k2) {
        this->SetName("HierarchicalNF");
        this->m_nfType = K;
      }

    HierarchicalNF(MPProblemType* _problem, XMLNodeReader& _node)
      : NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("HierarchicalNF");
        this->m_nfType = K;

        size_t k = _node.numberXMLParameter("k", true, 5, 0, MAX_INT, "K1 value for HierarchicalNF");
        size_t k2 = _node.numberXMLParameter("k2", true, 5, 0, MAX_INT, "K2 value for HierarchicalNF");
        string dmLabel2 = _node.stringXMLParameter("dmLabel2",true,"","Second distance metric method");

        m_nf1 = BruteForceNF<MPTraits>(this->m_dmLabel, this->m_unconnected, k);
        m_nf2 = BruteForceNF<MPTraits>(dmLabel2, this->m_unconnected, k2);
      }

    virtual void PrintOptions(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::PrintOptions(_os);
      _os << "\tNF1: ";
      m_nf1.PrintOptions(_os);
      _os << endl;
      _os << "\tNF2: ";
      m_nf2.PrintOptions(_os);
      _os << endl;
    }

    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, CfgType _cfg, size_t _k, OutputIterator _out){

        this->IncrementNumQueries();
        this->StartTotalTime();
        this->StartQueryTime();

        vector<pair<VID, double> > closest;
        m_nf1.FindNeighbors(_rmp, _first, _last, _cfg, back_inserter(closest));
        m_nf2.FindNeighbors(_rmp, closest.begin(), closest.end(), _cfg, _out);

        this->EndQueryTime();
        this->EndTotalTime();

        return _out;
      }

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.

    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          size_t _k, OutputIterator _out){
        cerr << "ERROR:: HierarchicalNF::KClosestPairs is not yet implemented. Exiting" << endl;
        exit(1);
      }

  private:
    BruteForceNF<MPTraits> m_nf1;
    BruteForceNF<MPTraits> m_nf2;
};

#endif
