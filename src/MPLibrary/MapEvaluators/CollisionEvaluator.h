#ifndef COLLISION_EVALUATION_H
#define COLLISION_EVALUATION_H

#include "MapEvaluatorMethod.h"
#include "MPLibrary/EdgeValidityCheckers/EdgeValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates whether a given metric meets a specific numeric condition.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CollisionEvaluator : public MapEvaluatorMethod<MPTraits> {
  public:

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID           VID;
    typedef typename MPTraits::Path             Path;


    CollisionEvaluator();
    CollisionEvaluator(XMLNode& _node);
    virtual ~CollisionEvaluator() = default;

    virtual bool operator()();

  protected:

  private:

    /// Write the coliisions into a path collision (.pc) file.
    /// @param _filename The name of the map file to write to.
    void Write(const std::string& _filename) const;

    string m_evcLabel;               ///< the edge validity checker label

    vector<size_t> m_collisions;     ///< Indices of obstacles found in collision

};

template <typename MPTraits>
CollisionEvaluator<MPTraits>::
CollisionEvaluator() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("CollisionEvaluator");
}

template <typename MPTraits>
CollisionEvaluator<MPTraits>::
CollisionEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("CollisionEvaluator");

  m_evcLabel = _node.Read("evcLabel", true, "", " the edge validity checker label.");

}



template <typename MPTraits>
bool
CollisionEvaluator<MPTraits>::
operator()() {

  EdgeValidityCheckerMethod<MPTraits>* evc = this->GetMPLibrary()->GetEdgeValidityChecker(m_evcLabel);

  RoadmapType* r = this->GetRoadmap();

  auto env = this->GetEnvironment();

  size_t numObst = env->NumObstacles();

  // Flags for obstacles in collision
  vector<bool> collisionIndicators = vector<bool>(numObst, false);

  vector<size_t> collisions;

  Path* path= this->GetPath();

  bool pathExists = (path and path->Size());

  if(pathExists){
    const vector<VID>& pathVIDs = path->VIDs();

    for (size_t i = 0; i < pathVIDs.size() - 1; i++) {

      evc->ValidateEdge(pathVIDs[i], pathVIDs[i+1], collisions);
      
      CfgType c1 = r->GetVertex(pathVIDs[i]);
      CfgType c2 = r->GetVertex(pathVIDs[i+1]);


      for (auto &col: collisions) {
        collisionIndicators[col] = true;
      }

      // std::cout << std::setprecision(4);

      // cout << "the edge (" << c1[0] << ", " << c1[1] <<", " << c1[2] <<") --> (" << c2[0] << ", " << c2[1] <<", " << c2[2] <<")" << endl;

      // cout << "collides with obsts: ";
      // for (auto &col: collisions) {
      //   cout << col << " ";
      // }
      // cout << endl;

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


template <typename MPTraits>
void
CollisionEvaluator<MPTraits>::
Write(const std::string& _filename) const {
  std::ofstream ofs(_filename);
  
  for (const size_t& obstIdx: m_collisions)
    ofs << obstIdx << " ";

  ofs.close();
}


#endif
