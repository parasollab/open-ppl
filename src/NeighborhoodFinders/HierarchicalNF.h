#ifndef HIERARCHICALNF_H_
#define HIERARCHICALNF_H_

#include "NeighborhoodFinderMethod.h"
#include "BruteForceNF.h"
#include <vector>

using namespace std;

template<class MPTraits>
class HierarchicalNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID; 

    HierarchicalNF(MPProblemType* _problem = NULL, string _dmm = "", string _dmm2 = "", size_t _k2 = 5, string _label = "") 
      : NeighborhoodFinderMethod<MPTraits>(_problem, _dmm, _label), m_dmLabel2(_dmm2), m_k2(_k2) {
        this->SetName("HierarchicalNF");
        m_nf1 = new BruteForceNF<MPTraits>(_problem, _dmm,"");
        m_nf2 = new BruteForceNF<MPTraits>(_problem, _dmm2,"");
      }

    HierarchicalNF(MPProblemType* _problem, XMLNodeReader& _node) 
      : NeighborhoodFinderMethod<MPTraits>(_problem,_node) {
        this->SetName("HierarchicalNF");
        m_k2 = _node.numberXMLParameter("k2", true, 5, 0, MAX_INT, "K value for HierarchicalNF");
        m_dmLabel2 = _node.stringXMLParameter("dmLabel2",true,"","Distance Metric Method the second one");

        m_nf1 = new BruteForceNF<MPTraits>(_problem, m_dmLabel,"");
        m_nf2 = new BruteForceNF<MPTraits>(_problem, m_dmLabel2,"");
      }

    virtual ~HierarchicalNF(){
      delete m_nf1;
      delete m_nf2;
    }

    virtual void PrintOptions(ostream& _os) const {
      PrintOptions(_os);
      _os << "dmMethod2: " << m_dmLabel2 << " "
        << "k2: " << m_k2 << " ";
    }

    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, 
          InputIterator _first, InputIterator _last, CfgType _cfg, size_t _k, OutputIterator _out){
        vector<VID> closest;
        m_nf1->KClosest(_rmp, _first, _last, _cfg, m_k2, back_inserter(closest));
        return m_nf2->KClosest(_rmp, closest.begin(), closest.end(), _cfg, _k, _out);
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
    string m_dmLabel; 
    string m_dmLabel2;
    BruteForceNF<MPTraits> *m_nf1;
    BruteForceNF<MPTraits> *m_nf2;
    size_t m_k2;
};

#endif //end #ifndef
