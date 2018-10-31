#ifndef PMPL_ROTATION_THEN_TRANSLATION_H_
#define PMPL_ROTATION_THEN_TRANSLATION_H_

#include "BasicExtender.h"


////////////////////////////////////////////////////////////////////////////////
/// Extend all rotational DOF before extending all translational DOF.
///
/// Rotation followed by Extension. In this way of extending the source
/// configuration is first rotated to align with the target configuration until
/// it is aligned or there is collision. It is then extended toward the target
/// configuration until collision or the target configuration is reached. This
/// can be seen as growing with a modified rotate-at-s local planner where
/// \f$s = 0\f$. Growing toward \f$q_{dir}\f$ can be seen as extending from
/// \f$q_{near}\f$ to \f$q_{rand1}\f$ which is only a change in orientation
/// followed by a translation from \f$q_{rand1}\f$ to \f$q_{rand}\f$.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RotationThenTranslation : public BasicExtender<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    RotationThenTranslation();

    RotationThenTranslation(XMLNode& _node);

    virtual ~RotationThenTranslation() = default;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
RotationThenTranslation<MPTraits>::
RotationThenTranslation() {
  this->SetName("RotationThenTranslation");
}


template <typename MPTraits>
RotationThenTranslation<MPTraits>::
RotationThenTranslation(XMLNode& _node) : BasicExtender<MPTraits>(_node) {
  this->SetName("RotationThenTranslation");
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
bool
RotationThenTranslation<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  CfgType innerCfg(robot), newDir(robot), newPos(robot);

  // Rotate component
  if(this->m_debug)
    cout << "rotate component" << endl;
  newDir = _end;
  for(size_t i = 0; i < _end.PosDOF(); i++)
    newDir[i] = _start[i];

  if(this->Expand(_start, newDir, innerCfg, this->m_maxDist, _lp,
      env->GetPositionRes(), env->GetOrientationRes())) {
    _lp.m_intermediates.push_back(innerCfg);

    // Translate component
    if(this->m_debug)
      cout << "translate component" << endl;
    newPos = innerCfg;
    for(size_t i = 0; i < newPos.PosDOF(); i++)
      newPos[i] = _end[i];

    LPOutput<MPTraits> newLPOutput;
    bool result = this->Expand(innerCfg, newPos, _new, this->m_maxDist,
        newLPOutput, env->GetPositionRes(), env->GetOrientationRes());
    _lp.m_edge.first.SetWeight(_lp.m_edge.first.GetWeight() +
        newLPOutput.m_edge.first.GetWeight());
    _lp.m_edge.second.SetWeight(_lp.m_edge.second.GetWeight() +
        newLPOutput.m_edge.second.GetWeight());
    return result;
  }

  return false;
}

/*----------------------------------------------------------------------------*/

#endif
