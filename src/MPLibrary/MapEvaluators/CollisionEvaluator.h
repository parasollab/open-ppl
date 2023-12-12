#ifndef COLLISION_EVALUATION_H
#define COLLISION_EVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates whether a given metric meets a specific numeric condition.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class CollisionEvaluator : public MapEvaluatorMethod {
  public:

    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID           VID;


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

#endif
