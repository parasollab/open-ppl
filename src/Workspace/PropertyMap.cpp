#include "PropertyMap.h"

#include "MPLibrary/MPLibrary.h"

/*------------------ Clearance annotated skeleton ------------------------*/

/// Function to generate the annotated clearance skeleton
PropertyMap<vector<double>,double>*
ClearanceAnnotatedSkeleton(MPBaseObject* _mp, WorkspaceSkeleton* _ws,
       bool _boundary) {
  auto clearanceMap = new PropertyMap<vector<double>,double>(_ws);

  auto g = _ws;
  auto boundary = _mp->GetEnvironment()->GetBoundary();
  auto vc = _mp->GetMPLibrary()->GetValidityChecker("pqp_solid");

  auto pointRobot = _mp->GetMPProblem()->GetRobot("point");

  // Function to compute clearance for input point _p.
  auto getClearance = [&](const Point3d& _p) -> double {
    // Check against obstacles using a point robot.
    Cfg cfg(_p, pointRobot);
    CDInfo cdInfo(true);
    vc->IsValid(cfg, cdInfo, "Skeleton Clearance");

    // Check against boundary.
    if(_boundary)  {
      const double boundaryClearance = boundary->GetClearance(_p);
      if(boundaryClearance < cdInfo.m_minDist) {
        cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
        cdInfo.m_minDist = boundaryClearance;
      }
    }
    // Return the minimum clearance.
    return cdInfo.m_minDist;
  };

  // Graph vertices clearance.
  for(auto vit = g->begin(); vit != g->end(); ++vit)
    clearanceMap->SetVertexProperty(vit->descriptor(),
        getClearance(vit->property()));

  // Graph edges clearance.
  for(auto eit = g->edges_begin(); eit != g->edges_end(); ++eit)	{
    vector<double> clearances;
    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
      clearances.push_back(getClearance(*pit));
    auto ed = WorkspaceSkeleton::ED(eit->source(), eit->target(),
        eit->descriptor().id());
    clearanceMap->SetEdgeProperty(ed,clearances);
  }
  return clearanceMap;
}

/// Function to generate the skeleton with edges filtered based on a
//tolerance value
// void
// ClearanceFilteredSkeleton(double _t, MPBaseObject* _mp,
//                           WorkspaceSkeleton* _ws, bool _boundary) {

//   // Function for filtering edges based on minimum clearance
//   struct ClearanceFiltration {
//     double m_min;   ///< Minimum clearance
//     ClearanceFiltration(double _a)  { m_min = _a; }
//     bool operator()(vector<double>& _i)  {
//       for(auto i: _i)
//         if(i < m_min)
// 	  return true;
//       return false;
//     }
//   };

//   auto clearanceMap = ClearanceAnnotatedSkeleton(_mp, _ws, _boundary);
//   _ws = clearanceMap->GetEdgeFilteredSkeleton(ClearanceFiltration(_t));
// }

/*------------------------------------------------------------------------*/
