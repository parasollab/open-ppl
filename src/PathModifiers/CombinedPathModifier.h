#ifndef COMBINEDPATHMODIFIER_H_
#define COMBINEDPATHMODIFIER_H_

#include "PathModifierMethod.h"

template<class MPTraits>
class CombinedPathModifier : public PathModifierMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::PathModifierPointer PathModifierPointer;

    CombinedPathModifier(const vector<string>& _pathModifierLabels = vector<string>());
    CombinedPathModifier(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;

    bool ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath);

  private:
    vector<string> m_pathModifierLabels;
};

// Non-XML Constructor
template<class MPTraits>
CombinedPathModifier<MPTraits>::CombinedPathModifier(const vector<string>& _pathModifierLabels) :
  PathModifierMethod<MPTraits>(), m_pathModifierLabels(_pathModifierLabels) {
    this->SetName("CombinedPathModifier");
  }

// XML Constructor
template<class MPTraits>
CombinedPathModifier<MPTraits>::CombinedPathModifier(MPProblemType* _problem, XMLNodeReader& _node) :
  PathModifierMethod<MPTraits>(_problem, _node) {
    this->SetName("CombinedPathModifier");
    ParseXML(_node);
  }

template<class MPTraits>
void
CombinedPathModifier<MPTraits>::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if(citr->getName() == "Modifier") {
      m_pathModifierLabels.push_back(citr->stringXMLParameter("label", true, "", "Path modifier label"));
      citr->warnUnrequestedAttributes();
    }
  }
}

template<class MPTraits>
void
CombinedPathModifier<MPTraits>::PrintOptions(ostream& _os) const {
  PathModifierMethod<MPTraits>::PrintOptions(_os);
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
