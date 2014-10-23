#ifndef MIXEXTENDER_H_
#define MIXEXTENDER_H_

#include "ExtenderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Randomly choose an extender from a set of extenders.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MixExtender : public ExtenderMethod<MPTraits> {
  public:
    // Extender label, probability, normalize probability
    typedef vector<pair<string, pair<double, double> > > ExpanderSet;

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MixExtender();
    MixExtender(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);
    virtual void Print(ostream& _os) const;

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, vector<CfgType>& _innerNodes);

  private:
    ExpanderSet m_growSet;
};

template<class MPTraits>
MixExtender<MPTraits>::MixExtender() :
  ExtenderMethod<MPTraits>() {
    this->SetName("MixExtender");
  }

template<class MPTraits>
MixExtender<MPTraits>::MixExtender(MPProblemType* _problem, XMLNodeReader& _node) :
  ExtenderMethod<MPTraits>(_problem, _node) {
    this->SetName("MixExtender");
    ParseXML(_node);
  }

template<class MPTraits>
void
MixExtender<MPTraits>::ParseXML(XMLNodeReader& _node) {
  // Get RRT Extender label and probability
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "Extender"){
      string label = citr->stringXMLParameter("label", true, "", "Extender label");
      double probability = citr->numberXMLParameter("probability", true, 0.0, 0.0, 1.0, "Extender probability");
      m_growSet.push_back(make_pair(label, make_pair(probability, 0)));
      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }

  // Normalize probabilities
  double total = 0;
  for(ExpanderSet::iterator it = m_growSet.begin(); it != m_growSet.end(); it++)
    total += it->second.first;

  double r = 0;
  for(ExpanderSet::iterator it = m_growSet.begin(); it != m_growSet.end(); it++) {
    it->second.first = it->second.first/total;
    it->second.second = r + it->second.first;
    r = it->second.second;
  }

  if(total == 0) {
    cerr << "Error::ParseXML : total probability of growth method is null" << endl;
    exit(1);
  }

  // Print
  if(this->m_debug) {
    cout << "Growth\nmehod\tprob\t\tprob norms" << endl;
    for(ExpanderSet::iterator it = m_growSet.begin(); it != m_growSet.end(); it++)
      cout << it->first << "\t" << it->second.first << "\t" << it->second.second << endl;
  }
}

template<class MPTraits>
void
MixExtender<MPTraits>::Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\textender label : " << endl;
  for(ExpanderSet::const_iterator it = m_growSet.begin(); it != m_growSet.end(); it++)
    _os << "\t\t" << it->first << endl;
 }

template<class MPTraits>
bool
MixExtender<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, vector<CfgType>& _innerNodes) {
  if(m_growSet.size() > 0) {
    double growthProb = DRand();
    for(ExpanderSet::iterator it = m_growSet.begin(); it != m_growSet.end(); it++) {
      if(growthProb < it->second.second) {
        if(this->m_debug)
          cout << " calling : " << it->first << endl;
        return this->GetMPProblem()->GetExtender(it->first)
          ->Extend(_near, _dir, _new, _innerNodes);
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
