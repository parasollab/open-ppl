#include "CollisionEvaluator.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/EdgeValidityCheckers/EdgeValidityCheckerMethod.h"


CollisionEvaluator::
CollisionEvaluator() : MapEvaluatorMethod() {
  this->SetName("CollisionEvaluator");
}


CollisionEvaluator::
CollisionEvaluator(XMLNode& _node) : MapEvaluatorMethod(_node) {
  this->SetName("CollisionEvaluator");

  m_evcLabel = _node.Read("evcLabel", true, "", " the edge validity checker label.");
}


bool
CollisionEvaluator::
operator()() {

  EdgeValidityCheckerMethod* evc = this->GetMPLibrary()->GetEdgeValidityChecker(m_evcLabel);

  RoadmapType* r = this->GetRoadmap();

  auto env = this->GetEnvironment();

  size_t numObst = env->NumObstacles();

  // Flags for obstacles in collision
  std::vector<bool> collisionIndicators = std::vector<bool>(numObst, false);

  std::vector<size_t> collisions;

  Path* path= this->GetPath();

  bool pathExists = (path and path->Size());

  if(pathExists){
    const std::vector<VID>& pathVIDs = path->VIDs();

    for (size_t i = 0; i < pathVIDs.size() - 1; i++) {

      evc->ValidateEdge(pathVIDs[i], pathVIDs[i+1], collisions);
      
      Cfg c1 = r->GetVertex(pathVIDs[i]);
      Cfg c2 = r->GetVertex(pathVIDs[i+1]);

      for (auto &col: collisions) {
        collisionIndicators[col] = true;
      }
    }
  }

  for (size_t i = 0; i < numObst; i++){
    if (collisionIndicators[i])
      m_collisions.push_back(i);
  }

  const std::string base = this->GetBaseFilename();

  Write(base + ".pc" );


  return pathExists;
}


void
CollisionEvaluator::
Write(const std::string& _filename) const {
  std::ofstream ofs(_filename);
  
  for (const size_t& obstIdx: m_collisions)
    ofs << obstIdx << " ";

  ofs.close();
}
