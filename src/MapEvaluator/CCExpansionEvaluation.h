#ifndef _CC_EXPANSION_EVALUATION_H
#define _CC_EXPANSION_EVALUATION_H

#include "CCDistanceEvaluation.h"
#include "CCDiameterEvaluation.h"

template <class CFG, class WEIGHT>
class CCExpansionEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  CCExpansionEvaluation(float dia, float dist, DistanceMetric* dm) : 
    cc_diameter(dia, dm),
    cc_distance(dist, dm)
  {}
  
  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    bool diameter_stable = cc_diameter.evaluate(rmap);
    bool distance_stable = cc_distance.evaluate(rmap);
    return (diameter_stable && distance_stable);
  }

  CCDiameterEvaluation<CFG,WEIGHT> cc_diameter;
  CCDistanceEvaluation<CFG,WEIGHT> cc_distance;
};

#endif
