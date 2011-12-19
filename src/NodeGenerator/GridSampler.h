#ifndef GRIDSAMPLER_h_
#define GRIDSAMPLER_h_

#include "SamplerMethod.h"

template <typename CFG>
class GridSampler : public SamplerMethod<CFG> {

  ValidityChecker<CFG>* m_vc;
  string m_vcm;
  map<size_t, size_t> m_numPoints;
  bool m_useBBX;

  public:
  GridSampler(){
    this->SetName("GridSampler");
  }

  GridSampler(ValidityChecker<CFG>* _vc, string _vcm, map<size_t, size_t> _numPoints, bool _useBBX) 
    : m_vc(_vc), m_vcm(_vcm), m_numPoints(_numPoints), m_useBBX(_useBBX) {
      this->SetName("GridSampler");
    }

  GridSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem){
    this->SetName("GridSampler");
    ParseXML(_node);
    m_vc = _problem->GetValidityChecker();
  }

  ~GridSampler(){}

  void  ParseXML(XMLNodeReader& _node){
    XMLNodeReader::childiterator citr;
    for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
      if(citr->getName() == "dimension"){
        size_t points = citr->numberXMLParameter("points", true, 10, 0, MAXINT, "Number of grid points excluding min & max");
        size_t index = citr->numberXMLParameter("index", true, 0, 0, MAXINT, "Index in BBX");
        m_numPoints[index] = points;
        citr->warnUnrequestedAttributes();
      }
    }
    
    m_vcm = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
    m_useBBX = _node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
    
    _node.warnUnrequestedAttributes();
    
    Print(cout);
  }

  virtual void Print(ostream& _os) const {
    _os << this->GetName()
      << "\n  m_vcm = " << m_vcm
      << "\n  m_useBBX = " << m_useBBX
      << "\n  m_numPoints (index, points):";  

    for (map<size_t, size_t>::const_iterator it = m_numPoints.begin() ; it != m_numPoints.end(); it++ ) {
      _os << "\n    " << it->first << ", " << it->second;
    }
  }

  virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
    // When using grid sampler, set the TestEval to a number 
    // that is a no more than 70-80% of the total number of 
    // possible grid points. Otherwise a very long time may pass
    // before enough unique points are generated
    bool generated = false;
    string callee(this->GetName());
    callee += "::sampler()";
    CDInfo cdInfo;

    int attempts = 0;
    do {
      attempts++;

      CFG tmp = _cfgIn;
      if(tmp==CFG())
        tmp.GetRandomCfg(_env);

      for (map<size_t, size_t>::iterator it=m_numPoints.begin() ; it != m_numPoints.end(); it++ ) {
        int index = it->first;
        int numPoints = it->second;

        double min = _env->GetBoundingBox()->GetRange(index).first; 
        double max = _env->GetBoundingBox()->GetRange(index).second;

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

      bool tmpFree = tmp.InBoundingBox(_env) && m_vc->IsValid(m_vc->GetVCMethod(m_vcm), tmp, _env, _stats, cdInfo, true, &callee);

      if(tmpFree){
        _cfgOut.push_back(tmp);
        generated = true;
      }

    } while (!generated && (attempts < _maxAttempts));

    return generated;
  }
};

#endif

