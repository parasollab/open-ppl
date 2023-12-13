#include "GaussianSampler.h"

#include "MPLibrary/MPLibrary.h"


GaussianSampler::
GaussianSampler(string _vcLabel, string _dmLabel,
    double _d, bool _useBoundary) :
  m_d(_d), m_useBoundary(_useBoundary),
  m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
    this->SetName("GaussianSampler");
}


GaussianSampler::
GaussianSampler(XMLNode& _node)
  : SamplerMethod(_node) {
    this->SetName("GaussianSampler");
    ParseXML(_node);
}


void
GaussianSampler::
ParseXML(XMLNode& _node) {
  m_d = _node.Read("d", true, 0.0, 0.0, MAX_DBL, "Gaussian D value");
  m_useBoundary = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance Metric Method");

  // Set mean and standard deviation separately
  mean = m_d;
  std = m_d;
}


//Display values of class variables at calling time
void
GaussianSampler::
Print(ostream& _os) const {
  SamplerMethod::Print(_os);
  // _os << "\td = " << m_d << endl;
  _os << "\tmean = " << mean << endl;
  _os << "\tstandard dev = " << std << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
}


bool
GaussianSampler::
Sampler(Cfg& _cfg, const Boundary* const _boundary,
    vector<Cfg>& _result, vector<Cfg>& _collision) {
  const std::string callee = this->GetNameAndLabel() + "::SampleImpl()";
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);

  if(this->m_debug)
    VDClearAll();

  //Start configuration, which was passed in.
  Cfg& cfg1 = _cfg;

  //First define validity of first CFG. If useBBX, require it to be in BBX
  //and only check validity with obstacles. Otherwise have both conditions.
  bool cfg1Free;
  if(!m_useBoundary) {
    if(!cfg1.InBounds(_boundary)) {
      if(this->m_debug){
        VDAddTempCfg(cfg1, false);
        VDComment("GaussianSampler::Attempt out of bounds.");
      }
      return false;
    }
    //We are in the box
    cfg1Free = vc->IsValid(cfg1, callee);
  }
  else
    cfg1Free = cfg1.InBounds(_boundary) && vc->IsValid(cfg1, callee);

  if(this->m_debug) {
    cout << "cfg1::" << cfg1 << endl;
    cout << "cfg1Free::" << cfg1Free << endl;
    VDAddTempCfg(cfg1, cfg1Free);
    if(cfg1Free)
      VDComment("GaussianSampler::Cfg1 is valid.");
    else
      VDComment("GaussianSamper::Cfg1 is invalid.");
  }

  //Determine cfg2 attributes with similar consdirations/method to cfg1
  auto robot = this->GetTask()->GetRobot();
  Cfg cfg2(robot), incr(robot);
  bool cfg2Free;
  if(!m_useBoundary) {
    do {
      incr.GetRandomRay(fabs(GaussianDistribution(fabs(mean), fabs(std))), dm);
      cfg2 = cfg1 + incr;
    } while(!cfg2.InBounds(_boundary));

    cfg2Free = vc->IsValid(cfg2, callee);
  }
  else {
    incr.GetRandomRay(fabs(GaussianDistribution(fabs(mean), fabs(std))), dm);
    cfg2 = cfg1 + incr;

    cfg2Free = cfg2.InBounds(_boundary) &&
      vc->IsValid(cfg2, callee);
  }

  if(this->m_debug) {
    cout << "incr::" << incr << endl;
    cout << "cfg2::" << cfg2 << endl;
    VDAddTempRay(incr);
    cout << "cfg2Free::" << cfg2Free << endl;
    VDAddTempCfg(cfg2, cfg2Free);
    if(cfg2Free)
      VDComment("GaussianSampler::Cfg2 is valid.");
    else
      VDComment("GaussianSampler::Cfg2 is invalid.");
  }

  // Check what sort of samples we want. 
  if (newSamplesOnly && cfg2Free) {
	  _result.push_back(cfg2);
	
    if(this->m_debug) {
      cout << "Generated::" << cfg2 << endl; //May be repetitive..
      VDComment("GaussianSampling::Successful");
    }
    return true;
  } else if (newSamplesOnly) {return false;}

  //If validities are different, retain the free node
  if(cfg1Free != cfg2Free) {
    Cfg gen = cfg1Free ? cfg1 : cfg2;
    Cfg col = cfg1Free ? cfg2 : cfg1;
    _result.push_back(gen);
    _collision.push_back(col);

    if(this->m_debug) {
      cout << "Generated::" << gen << endl; //May be repetitive..
      VDComment("GaussianSampling::Successful");
    }

    return true;
  } 
  return false;
}


void
GaussianSampler::
ChangeGaussianParams(double newMean, double newStd) {
  this->mean = newMean;
  this->std = newStd;

  if (this->m_debug) {
    cout << "Changing Gaussian Sampler's mean to " << this->mean
        << "and std to " << this->std << endl;
  }
}


void 
GaussianSampler::
ReturnNewSamplesOnly(bool setting) {
  newSamplesOnly = setting;

  if (this->m_debug) {
    cout << "Changing Gaussian Sampler to only return samples " << 
            "influenced by Gaussian (no means)." << endl;
  }
}
