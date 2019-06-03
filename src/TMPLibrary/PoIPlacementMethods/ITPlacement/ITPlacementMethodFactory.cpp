#include "BallFilling.h"
#include "FixedBase.h"
#include "GridFilling.h"
#include "DisjointWorkspaces.h"
#include "OverlappingWorkspacesDensity.h"
#include "ITPlacementMethod.h"
#include "WorkspaceGuidance.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <string>


std::unique_ptr<PlacementMethod>
ITPlacementMethod::
Factory(XMLNode& _node) {
  // Read the node and mark it as visited.
  std::string type = _node.Read("type", true, "", "The IT Placement Method name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<ITPlacementMethod> output;

  if(type == "fixedbase")
    output = std::unique_ptr<FixedBase>(
        new FixedBase(_node)
    );
  else if(type == "disjointworkspaces")
    output = std::unique_ptr<DisjointWorkspaces>(
        new DisjointWorkspaces(_node)
    );
  else if(type == "owdensity")
    output = std::unique_ptr<OverlappingWorkspacesDensity>(
        new OverlappingWorkspacesDensity(_node)
    );
  else if(type == "workspaceguidance")
    output = std::unique_ptr<WorkspaceGuidance>(
        new WorkspaceGuidance(_node)
    );
  else if(type == "ballfilling")
    output = std::unique_ptr<BallFilling>(
        new BallFilling(_node)
    );
  else if(type == "gridfilling")
    output = std::unique_ptr<GridFilling>(
        new GridFilling(_node)
    );
  else
    throw ParseException(_node.Where(), "IT Placement Method type '" + type + "'.");

  // Read the debug flag.
  output->m_debug = _node.Read("debug", false, false, "Show debug messages.");

  return output;
}
