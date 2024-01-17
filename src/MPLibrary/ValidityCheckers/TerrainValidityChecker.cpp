#include "TerrainValidityChecker.h"

/*------------------------------ Construction --------------------------------*/

TerrainValidityChecker::
TerrainValidityChecker() {
  this->SetName("TerrainValidity");
}


TerrainValidityChecker::
TerrainValidityChecker(XMLNode& _node) : ValidityCheckerMethod(_node) {
  this->SetName("TerrainValidity");
}

/*------------------------- ValidityChecker Interface ------------------------*/

bool
TerrainValidityChecker::
IsValidImpl(Cfg& _cfg, CDInfo&, const string&) {
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
