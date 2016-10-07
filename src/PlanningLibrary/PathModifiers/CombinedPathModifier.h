#ifndef COMBINEDPATHMODIFIER_H_
#define COMBINEDPATHMODIFIER_H_

#include "PathModifierMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CombinedPathModifier : public PathModifierMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::PathModifierPointer PathModifierPointer;

    CombinedPathModifier(const vector<string>& _pathModifierLabels = vector<string>());
    CombinedPathModifier(MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    bool ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath);

  private:
    vector<string> m_pathModifierLabels;
};

template<class MPTraits>
CombinedPathModifier<MPTraits>::
CombinedPathModifier(const vector<string>& _pathModifierLabels) :
  PathModifierMethod<MPTraits>(), m_pathModifierLabels(_pathModifierLabels) {
    this->SetName("CombinedPathModifier");
  }

template<class MPTraits>
CombinedPathModifier<MPTraits>::
CombinedPathModifier(MPProblemType* _problem, XMLNode& _node) :
  PathModifierMethod<MPTraits>(_problem, _node) {
    this->SetName("CombinedPathModifier");
    ParseXML(_node);
  }

template<class MPTraits>
void
CombinedPathModifier<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    if(child.Name() == "Modifier")
      m_pathModifierLabels.push_back(child.Read("label", true, "", "Path modifier label"));
}

template<class MPTraits>
void
CombinedPathModifier<MPTraits>::Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tuse method(s) :" << endl;
  for (size_t i=0; i < m_pathModifierLabels.size() ; i++)
    _os << "\t" << m_pathModifierLabels[i] << endl;
}

template<class MPTraits>
bool
CombinedPathModifier<MPTraits>::ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath) {
  if(this->m_debug)
    cout << "\n*C* Executing CombinedPathModifier::Modifier" << endl;

  vector<CfgType> tmpPath;
  _newPath = _originalPath;

  for(vector<string>::iterator it = m_pathModifierLabels.begin(); it != m_pathModifierLabels.end(); it++) {
    if(this->m_debug)
      cout << "*C* Executing path modifier : " << *it << endl;
    tmpPath = _newPath;
    this->GetMPProblem()->GetPathModifier(*it)->Modify(tmpPath, _newPath);
  }

  return false;
}

#endif
