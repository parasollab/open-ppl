#ifndef EXTENDERMETHOD_H_
#define EXTENDERMETHOD_H_

#include <string>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Base algorithm abstraction for \ref Extenders.
///
/// ExtenderMethod has one main method, @c Extend, to grow a simple path from a
/// starting node in some input direction.
/// @usage
/// @code
/// ExtenderPointer e = this->GetMPProblem()->GetExtender(m_eLabel);
/// CfgType c, cDir, cNew;
/// vector<CfgType> intermediates;
/// e->Extend(c, cDir, cNew, intermediates);
/// @endcode
/// @c Extend returns a boolean of success/fail, fills @c cNew with the extended
/// configuration, and fills @c intermediates with any intermediate
/// configurations along the polygon chain of the expansion --- not all
/// expansion methods go in straight lines through @cspace.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ExtenderMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ExtenderMethod();
    ExtenderMethod(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void Print(ostream& _os) const;

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
ExtenderMethod<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

#endif
