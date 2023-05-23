#ifndef TERRAIN_VALIDITY_CHECKER_H
#define TERRAIN_VALIDITY_CHECKER_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Check that a robot is within terrains for which it has the required
/// capability. A configuration is considered valid if it is within the
/// required terrain and outside all other terrains.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TerrainValidityChecker : virtual public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    TerrainValidityChecker();
    TerrainValidityChecker(XMLNode& _node);
    virtual ~TerrainValidityChecker() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName) override;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
TerrainValidityChecker<MPTraits>::
TerrainValidityChecker() {
  this->SetName("TerrainValidity");
}


template <typename MPTraits>
TerrainValidityChecker<MPTraits>::
TerrainValidityChecker(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("TerrainValidity");
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
TerrainValidityChecker<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo&, const string&) {
  if(_cfg.GetRobot()->GetLabel() == "coordinator")
    return true;

  auto env = this->GetEnvironment();

  // Check whether the cfg is within a terrain boundary. If so,
  // Check that the capability of its robot matches the capability of the terrain.
  for(auto& elem : env->GetTerrains()) {
    auto terrainCapability = elem.first;

    if(this->m_debug)
      std::cout << "Terrain Capability: " << terrainCapability
                << " Robot Capability:  " << _cfg.GetRobot()->GetCapability()
                << std::endl;

    // Check if robot has same capability as terrain
    const bool hasCapability = terrainCapability == _cfg.GetRobot()->GetCapability();
    bool inBoundary = false;

    for(auto& terrain : elem.second) {
			if(terrain.IsVirtual() and this->m_debug){
				std::cout << "virtual boundary" << std::endl; //remove this line
			}

      // Check if cfg is in the terrain boundary
      inBoundary |= terrain.InTerrain(_cfg.GetPoint()) ||
                    terrain.InTerrain(_cfg);

      // If the cfg is within the terrain boundary and has the same capability
      // then move on to check the other terrains.
      if(hasCapability and inBoundary)
        break;
      // Else if we do not have the capability, we cannot be inside this
      // boundary.
      else if(!hasCapability and inBoundary and !terrain.IsVirtual())
        return false;
    }

    if(hasCapability and !inBoundary)
      return false;
  }

  // If the cfg is not in any invalid terrains, return true.
  return true;
}

/*----------------------------------------------------------------------------*/

#endif
