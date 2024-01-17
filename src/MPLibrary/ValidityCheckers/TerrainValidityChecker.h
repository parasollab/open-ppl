#ifndef TERRAIN_VALIDITY_CHECKER_H
#define TERRAIN_VALIDITY_CHECKER_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Check that a robot is within terrains for which it has the required
/// capability. A configuration is considered valid if it is within the
/// required terrain and outside all other terrains.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class TerrainValidityChecker : virtual public ValidityCheckerMethod {

  public:

    ///@name Construction
    ///@{

    TerrainValidityChecker();
    TerrainValidityChecker(XMLNode& _node);
    virtual ~TerrainValidityChecker() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    virtual bool IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo,
        const string& _callName) override;

    ///@}

};

#endif
