#ifndef MIX_EXTENDER_H_
#define MIX_EXTENDER_H_

#include "ExtenderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Randomly choose an extender from a set of extenders.
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MixExtender : public ExtenderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    /// Extender label, probability, normalize probability
    typedef vector<pair<string, pair<double, double>>> ExpanderSet;

    ///@}
    ///@name Construction
    ///@{

    MixExtender();

    MixExtender(XMLNode& _node);

    virtual ~MixExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@{
    ///@name ExtenderMethod Overrides
    ///@{

    virtual double GetMinDistance() const override;
    virtual double GetMaxDistance() const override;

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Compute the minimum and maximum distances over the extender set.
    void ComputeLimits() const;

    ///@}
    ///@name MixExtender State
    ///@{

    ExpanderSet m_growSet;
    mutable bool m_limitsCached{false};

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MixExtender<MPTraits>::
MixExtender() : ExtenderMethod<MPTraits>() {
  this->SetName("MixExtender");
}


template <typename MPTraits>
MixExtender<MPTraits>::
MixExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node) {
  this->SetName("MixExtender");

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

/*---------------------------- MPBaseObject Overrides ------------------------*/

template <typename MPTraits>
void
MixExtender<MPTraits>::
Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\textender label : " << endl;
  for(auto ex : m_growSet)
    _os << "\t\t" << ex.first << endl;
 }

/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
double
MixExtender<MPTraits>::
GetMinDistance() const {
  if(!m_limitsCached)
    ComputeLimits();
  return this->m_minDist;
}


template <typename MPTraits>
double
MixExtender<MPTraits>::
GetMaxDistance() const {
  if(!m_limitsCached)
    ComputeLimits();
  return this->m_maxDist;
}


template <typename MPTraits>
bool
MixExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  if(m_growSet.empty())
    throw RunTimeException(WHERE, "No extender methods present in MixExtender");

  double growthProb = DRand();
  for(ExpanderSet::iterator it = m_growSet.begin(); it != m_growSet.end();
      it++) {
    if(growthProb < it->second.second) {
      if(this->m_debug)
        cout << " calling : " << it->first << endl;
      return this->GetExtender(it->first)->Extend(_start, _end, _new, _lp);
    }
  }

  return false;
}

/*----------------------------- Helpers --------------------------------------*/

template <typename MPTraits>
void
MixExtender<MPTraits>::
ComputeLimits() const {
  double& minDist = const_cast<double&>(this->m_minDist);
  double& maxDist = const_cast<double&>(this->m_maxDist);
  maxDist = numeric_limits<double>::min();
  minDist = numeric_limits<double>::max();
  for(auto& ex : m_growSet) {
    auto e = this->GetExtender(ex.first);
    maxDist = max(maxDist, e->GetMaxDistance());
    minDist = min(minDist, e->GetMinDistance());
  }
  m_limitsCached = true;
}

/*----------------------------------------------------------------------------*/

#endif
