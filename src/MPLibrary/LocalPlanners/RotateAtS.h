#ifndef ROTATE_AT_S_H_
#define ROTATE_AT_S_H_

#include "TransformAtS.h"


////////////////////////////////////////////////////////////////////////////////
/// Generates a local plan by translating the start configuration to the location
/// "s" percent along the straight line path, then rotating to match the goal
/// orientation, and finally translating to the goal.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RotateAtS : public TransformAtS<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    RotateAtS(double _s = 0.5, const string& _vcLabel = "",
        bool _evalation = false, bool _saveIntermediates = false);

    RotateAtS(XMLNode& _node);

    virtual ~RotateAtS() = default;

    ///@}

  protected:

    ///@name TransformAtS Overrides
    ///@{

    virtual std::vector<CfgType> GetSequenceNodes(const CfgType& _c1,
        const CfgType& _c2, const bool _reverse = false) override;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
RotateAtS<MPTraits>::
RotateAtS(double _s, const string& _vcLabel, bool _evalation,
    bool _saveIntermediates) :
    TransformAtS<MPTraits>(_s, _vcLabel, _evalation, _saveIntermediates) {
  this->SetName("RotateAtS");
}


template <typename MPTraits>
RotateAtS<MPTraits>::
RotateAtS(XMLNode& _node): TransformAtS<MPTraits>(_node) {
  this->SetName("RotateAtS");
}

/*-------------------------- TransformAtS Overrides --------------------------*/

template <typename MPTraits>
std::vector<typename MPTraits::CfgType>
RotateAtS<MPTraits>::
GetSequenceNodes(const CfgType& _c1, const CfgType& _c2, const bool _reverse) {
  // Push the start node into the sequence.
  std::vector<CfgType> sequence;
  sequence.reserve(4);
  sequence.push_back(_c1);

  // Make the first transform point (up to the rotation).
  CfgType upToRotation = _c1;
  upToRotation.SetLinearPosition(_c2.GetLinearPosition());
  upToRotation.WeightedSum(_c1, upToRotation, this->m_s);
  sequence.push_back(upToRotation);

  // Make the second transform point (after the rotation).
  CfgType afterRotation = _c2;
  afterRotation.SetLinearPosition(upToRotation.GetLinearPosition());
  sequence.push_back(afterRotation);

  // Push the end node into the sequence.
  sequence.push_back(_c2);

  return sequence;
}

/*----------------------------------------------------------------------------*/

#endif
