#ifndef MIX_EXTENDER_H_
#define MIX_EXTENDER_H_

#include "ExtenderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Randomly choose an extender from a set of extenders.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MixExtender : public ExtenderMethod<MPTraits> {
  public:
    // Extender label, probability, normalize probability
    typedef vector<pair<string, pair<double, double> > > ExpanderSet;

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MixExtender();
    MixExtender(MPProblemType* _problem, XMLNode& _node);

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    double GetDelta() const;

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);

  private:
    ExpanderSet m_growSet;

    mutable bool m_deltaComputed = false;
    mutable double m_delta = numeric_limits<double>::min();
};

template<class MPTraits>
MixExtender<MPTraits>::
MixExtender() :
  ExtenderMethod<MPTraits>() {
    this->SetName("MixExtender");
  }

template<class MPTraits>
MixExtender<MPTraits>::
MixExtender(MPProblemType* _problem, XMLNode& _node) :
  ExtenderMethod<MPTraits>(_problem, _node) {
    this->SetName("MixExtender");
    ParseXML(_node);
  }

template<class MPTraits>
void
MixExtender<MPTraits>::
ParseXML(XMLNode& _node) {
  // Get RRT Extender label and probability
  for(auto& child : _node) {
    if(child.Name() == "Extender"){
      string label = child.Read("label", true, "",
          "Extender label");
      double probability = child.Read("probability", true, 0.0,
          0.0, 1.0, "Extender probability");
      m_growSet.push_back(make_pair(label, make_pair(probability, 0)));
    }
  }

  // Normalize probabilities
  double total = 0;
  for(auto& exp : m_growSet)
    total += exp.second.first;

  double r = 0;
  for(auto& exp : m_growSet) {
    exp.second.first = exp.second.first/total;
    exp.second.second = r + exp.second.first;
    r = exp.second.second;
  }

  if(total == 0)
    throw ParseException(_node.Where(), "Total probability of growth methods"
        " is 0.");

  // Print
  if(this->m_debug) {
    cout << "Growth\nmehod\tprob\t\tprob norms" << endl;
    for(auto& exp : m_growSet)
      cout << exp.first << "\t" << exp.second.first << "\t"
           << exp.second.second << endl;
  }
}

template<class MPTraits>
void
MixExtender<MPTraits>::Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\textender label : " << endl;
  for(ExpanderSet::const_iterator it = m_growSet.begin();
      it != m_growSet.end(); it++)
    _os << "\t\t" << it->first << endl;
 }

template<class MPTraits>
double
MixExtender<MPTraits>::
GetDelta() const {
  if(!m_deltaComputed) {
    for(auto& e : m_growSet)
      m_delta = max(m_delta, this->GetExtender(e.first)->GetDelta());
    m_deltaComputed = true;
  }

  return m_delta;
}

template<class MPTraits>
bool
MixExtender<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  if(m_growSet.size() > 0) {
    double growthProb = DRand();
    for(ExpanderSet::iterator it = m_growSet.begin(); it != m_growSet.end();
        it++) {
      if(growthProb < it->second.second) {
        if(this->m_debug)
          cout << " calling : " << it->first << endl;
        return this->GetExtender(it->first)
          ->Extend(_near, _dir, _new, _lpOutput);
      }
    }
  }
  else{
    cerr << "Error::ExpandTree : No extender method" << endl;
    exit(1);
  }

  return false;
}

#endif
