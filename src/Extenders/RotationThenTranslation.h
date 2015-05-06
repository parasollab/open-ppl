#ifndef ROTATIONTHENTRANSLATION_H_
#define ROTATIONTHENTRANSLATION_H_

#include "BasicExtender.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend all rotational DOF before extending all translational DOF.
///
/// Rotation followed by Extension. In this way of extending the source
/// configuration is first rotated to align with the target configuration until
/// it is aligned or there is collision. It is then extended toward the target
/// configuration until collision or the target configuration is reached. This
/// can be seen as growing with a modified rotate-at-s local planner where
/// \f$s = 0\f$. Growing toward \f$q_{dir}\f$ can be seen as extending from
/// \f$q_{near}\f$ to \f$q_{rand1}\f$ which is only a change in orientation
/// followed by a translation from \f$q_{rand1}\f$ to \f$q_{rand}\f$.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RotationThenTranslation : public BasicExtender<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    RotationThenTranslation(const string& _dmLabel = "", 
        const string& _vcLabel = "", double _delta = 1.0);
    RotationThenTranslation(MPProblemType* _problem, XMLNode& _node);

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);
};

template<class MPTraits>
RotationThenTranslation<MPTraits>::RotationThenTranslation(const string& _dmLabel,
    const string& _vcLabel, double _delta) :
  BasicExtender<MPTraits>(_dmLabel, _vcLabel, _delta) {
    this->SetName("RotationThenTranslation");
  }

template<class MPTraits>
RotationThenTranslation<MPTraits>::RotationThenTranslation(MPProblemType* _problem, XMLNode& _node) :
  BasicExtender<MPTraits>(_problem, _node) {
    this->SetName("RotationThenTranslation");
  }

template<class MPTraits>
bool
RotationThenTranslation<MPTraits>::Extend(const CfgType& _near, 
    const CfgType& _dir, CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  CfgType innerCfg, newDir, newPos;

  // Rotate component
  if(this->m_debug)
    cout << "rotate component" << endl;
  newDir = _dir;
  for(size_t i = 0; i < _dir.PosDOF(); i++)
    newDir[i] = _near[i];

  if(this->Expand(_near, newDir, innerCfg, this->m_delta, _lpOutput,
      env->GetPositionRes(), env->GetOrientationRes())) {
    _lpOutput.m_intermediates.push_back(innerCfg);

    // Translate component
    if(this->m_debug)
      cout << "translate component" << endl;
    newPos = innerCfg;
    for(size_t i = 0; i < newPos.PosDOF(); i++)
      newPos[i] = _dir[i];

    LPOutput<MPTraits> newLPOutput;
    bool result = this->Expand(innerCfg, newPos, _new, this->m_delta, 
        newLPOutput, env->GetPositionRes(), env->GetOrientationRes());
    _lpOutput.m_edge.first.SetWeight(_lpOutput.m_edge.first.GetWeight() + 
        newLPOutput.m_edge.first.GetWeight());
    _lpOutput.m_edge.second.SetWeight(_lpOutput.m_edge.second.GetWeight() + 
        newLPOutput.m_edge.second.GetWeight());
    return result;
  }

  return false;
}

#endif
