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

    string m_evcLabel;               ///< the edge validity checker label

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

  vector<size_t> collisions;

  // for (auto eit = r->edges_begin(); eit != r->edges_end(); ++eit ){
  //   evc->ValidateEdge(eit->source(), eit->target(), collisions);

  //   CfgType c1 = r->GetVertex(eit->source());
  //   CfgType c2 = r->GetVertex(eit->target());

  //   std::cout << std::setprecision(4);

  //   cout << "the edge (" << c1[0] << ", " << c1[1] <<", " << c1[2] <<") --> (" << c2[0] << ", " << c2[1] <<", " << c2[2] <<")" << endl;

  //   cout << "collides with obsts: ";
  //   for (auto &col: collisions) {
  //     cout << col << " ";
  //   }
  //   cout << endl;
  // }

  Path* path= this->GetPath();

  if(path and path->Size()){
    const vector<VID>& pathVIDs = path->VIDs();

    for (size_t i = 0; i < pathVIDs.size() - 1; i++) {

      evc->ValidateEdge(pathVIDs[i], pathVIDs[i+1], collisions);
      
      CfgType c1 = r->GetVertex(pathVIDs[i]);
      CfgType c2 = r->GetVertex(pathVIDs[i+1]);

      std::cout << std::setprecision(4);

      cout << "the edge (" << c1[0] << ", " << c1[1] <<", " << c1[2] <<") --> (" << c2[0] << ", " << c2[1] <<", " << c2[2] <<")" << endl;

      cout << "collides with obsts: ";
      for (auto &col: collisions) {
        cout << col << " ";
      }
      cout << endl;

    }
  }

   return true;
}

#endif
