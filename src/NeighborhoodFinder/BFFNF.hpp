#ifndef BFFNF_H_
#define BFFNF_H_

#include "NeighborhoodFinderMethod.hpp"
#include "DistanceMetrics.h"
#include <vector>

class BFFNF: public NeighborhoodFinderMethod {
  public:
    BFFNF(string _dmm = "", string _dmm2 = "", size_t _k2 = 5, string _label = "", MPProblem* _problem = NULL) 
      : NeighborhoodFinderMethod(_dmm, _label, _problem), m_dmLabel2(_dmm2), m_k2(_k2) {
        SetName("BFFNF");
        m_nf1 = new BFNF(_dmm,"",_problem);
        m_nf2 = new BFNF(_dmm2,"",_problem);
      }

    BFFNF(XMLNodeReader& _node, MPProblem* _problem) 
      : NeighborhoodFinderMethod(_node,_problem) {
        SetName("BFFNF");

        m_k2 = _node.numberXMLParameter("k2", true, 5, 0, MAX_INT, "K value for BFFNF");
        m_dmLabel2 = _node.stringXMLParameter("dmMethod2",true,"","Distance Metric Method the second one");

        m_nf1 = new BFNF(m_dmLabel,"",_problem);
        m_nf2 = new BFNF(m_dmLabel2,"",_problem);

        if(this->m_debug)
          PrintOptions(cout);
      }

    virtual ~BFFNF(){
      delete m_nf1;
      delete m_nf2;
    }

    virtual void PrintOptions(ostream& _os) const {
      NeighborhoodFinderMethod::PrintOptions(_os);
      _os << "dmMethod2: " << m_dmLabel2 << " "
        << "k2: " << m_k2 << " ";
    }

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RDMP* _rmp, 
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out){
        vector<typename RDMP::VID> closest;
        m_nf1->KClosest(_rmp, _first, _last, _cfg, m_k2, back_inserter(closest));
        return m_nf2->KClosest(_rmp, closest.begin(), closest.end(), _cfg, _k, _out);
      }

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RDMP* _rmp,
          InputIterator _first1, InputIterator _last1, 
          InputIterator _first2, InputIterator _last2, 
          size_t _k, OutputIterator _out){
        cerr << "ERROR:: BFFNF::KClosestPairs is not yet implemented. Exiting" << endl;
        exit(1);
      }

  private:
    string m_dmLabel2;
    BFNF *m_nf1;
    BFNF *m_nf2;
    size_t m_k2;
};

#endif //end #ifndef
