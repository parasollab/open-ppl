#ifndef EXTENDERMETHOD_H_
#define EXTENDERMETHOD_H_

#include <string>
#include <iostream>

template<class MPTraits>
class ExtenderMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ExtenderMethod();
    ExtenderMethod(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void PrintOptions(ostream& _os) const;

    virtual bool Extend(const CfgType& _nearest, const CfgType& _dir,
        CfgType& _new, vector<CfgType>& _innerNodes) =0;
};

template <class MPTraits>
ExtenderMethod<MPTraits>::ExtenderMethod() :
  MPBaseObject<MPTraits>() {
    this->SetName("ExtenderMethod");
  }

template <class MPTraits>
ExtenderMethod<MPTraits>::ExtenderMethod(MPProblemType* _problem, XMLNodeReader& _node) :
  MPBaseObject<MPTraits>(_problem, _node) {
    this->SetName("ExtenderMethod");
  }

template <class MPTraits>
void
ExtenderMethod<MPTraits>::PrintOptions(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

#endif
