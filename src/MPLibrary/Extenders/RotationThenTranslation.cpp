#include "RotationThenTranslation.h"

/*------------------------------ Construction --------------------------------*/

RotationThenTranslation::RotationThenTranslation() {
  this->SetName("RotationThenTranslation");
}

RotationThenTranslation::RotationThenTranslation(XMLNode& _node)
    : ExtenderMethod(_node), BasicExtender(_node) {
  this->SetName("RotationThenTranslation");
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

bool RotationThenTranslation::Extend(const Cfg& _start,
                                     const Cfg& _end,
                                     Cfg& _new,
                                     LPOutput& _lp) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  Cfg innerCfg(robot), newDir(robot), newPos(robot);

  // Rotate component
  if (this->m_debug)
    cout << "rotate component" << endl;
  newDir = _end;
  for (size_t i = 0; i < _end.PosDOF(); i++)
    newDir[i] = _start[i];

  if (this->Expand(_start, newDir, innerCfg, this->m_maxDist, _lp,
                   env->GetPositionRes(), env->GetOrientationRes())) {
    _lp.m_intermediates.push_back(innerCfg);

    // Translate component
    if (this->m_debug)
      cout << "translate component" << endl;
    newPos = innerCfg;
    for (size_t i = 0; i < newPos.PosDOF(); i++)
      newPos[i] = _end[i];

    LPOutput newLPOutput;
    bool result =
        this->Expand(innerCfg, newPos, _new, this->m_maxDist, newLPOutput,
                     env->GetPositionRes(), env->GetOrientationRes());
    _lp.m_edge.first.SetWeight(_lp.m_edge.first.GetWeight() +
                               newLPOutput.m_edge.first.GetWeight());
    _lp.m_edge.second.SetWeight(_lp.m_edge.second.GetWeight() +
                                newLPOutput.m_edge.second.GetWeight());
    return result;
  }

  return false;
}

/*----------------------------------------------------------------------------*/