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
class RotationThenTranslation : public BasicExtender {
 public:
  ///@name Motion Planning Types
  ///@{

  ///@}
  ///@name Construction
  ///@{

  RotationThenTranslation();

  RotationThenTranslation(XMLNode& _node);

  virtual ~RotationThenTranslation() = default;

  ///@}
  ///@name ExtenderMethod Overrides
  ///@{

  virtual bool Extend(const Cfg& _start,
                      const Cfg& _end,
                      Cfg& _new,
                      LPOutput& _lp) override;

  ///@}
};

#endif
