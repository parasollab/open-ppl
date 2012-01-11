#ifndef GRIDSAMPLER_H_
#define GRIDSAMPLER_H_

#include "SamplerMethod.h"

template <typename CFG>
class GridSampler : public SamplerMethod<CFG> {
  string m_vcm;
  map<size_t, size_t> m_numPoints;
  bool m_useBBX;

  public:
  GridSampler() : m_vcm(""), m_useBBX("false") {
    this->SetName("GridSampler");
  }

  GridSampler(string _vcm, map<size_t, size_t> _numPoints, bool _useBBX) 
    : m_vcm(_vcm), m_numPoints(_numPoints), m_useBBX(_useBBX) {
    this->SetName("GridSampler");
  }

  GridSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
    this->SetName("GridSampler");
    ParseXML(_node);
  }

  ~GridSampler(){}

  void  ParseXML(XMLNodeReader& _node) {
    for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
      if(citr->getName() == "dimension") {
        size_t points = citr->numberXMLParameter("points", true, 10, 0, MAXINT, "Number of grid points excluding min & max");
        size_t index = citr->numberXMLParameter("index", true, 0, 0, MAXINT, "Index in BBX");
        m_numPoints[index] = points;
        citr->warnUnrequestedAttributes();
      }
    }
    
    m_vcm = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
    m_useBBX = _node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
    
    _node.warnUnrequestedAttributes();
  }

  virtual void PrintOptions(ostream& _os) const {
    SamplerMethod<CFG>::PrintOptions(_os);
    _os << "\tm_vcm = " << m_vcm << endl;
    _os << "\tm_useBBX = " << m_useBBX << endl;
    _os << "\tm_numPoints (index, points):";
    for(map<size_t, size_t>::const_iterator it = m_numPoints.begin() ; it != m_numPoints.end(); it++ ) 
      _os << "\n\t\t" << it->first << ", " << it->second;
    _os << endl;
  }

  virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
    // When using grid sampler, set the TestEval to a number 
    // that is a no more than 70-80% of the total number of 
    // possible grid points. Otherwise a very long time may pass
    // before enough unique points are generated
    bool generated = false;
    string callee(this->GetName());
    callee += "::sampler()";
    ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
    CDInfo cdInfo;

    int attempts = 0;
    do {
      attempts++;

      CFG tmp = _cfgIn;
      if(tmp==CFG())
        tmp.GetRandomCfg(_env,_bb);

      for (map<size_t, size_t>::iterator it=m_numPoints.begin() ; it != m_numPoints.end(); it++ ) {
        int index = it->first;
        int numPoints = it->second;

        double min = _bb->GetRange(index).first; 
        double max = _bb->GetRange(index).second;

        double givenp = tmp.GetData()[index];

        double delta = (max-min)/(numPoints+1);
        double steps = floor(((givenp-min)/delta)+0.5);
        double gridp = steps*delta + min;
        if(gridp > max){
          gridp = max;
        }
        if(gridp < min){
          gridp = min;
        }
        if(m_useBBX){
          if(gridp <= max + delta/10 && gridp >= max - delta/10){
            gridp = max - delta;    
          }
          if(gridp <= min + delta/10 && gridp >= min -delta/10){
            gridp = min + delta;
          }
        }

        tmp.SetSingleParam(index,gridp);
      }

      bool tmpFree = tmp.InBoundingBox(_env,_bb) && vc->IsValid(vc->GetVCMethod(m_vcm), tmp, _env, _stats, cdInfo, true, &callee);

      if(tmpFree){
        _cfgOut.push_back(tmp);
        generated = true;
      }

    } while (!generated && (attempts < _maxAttempts));

    return generated;
  }
};

#endif

