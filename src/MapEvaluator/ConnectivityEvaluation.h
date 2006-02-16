#ifndef _CONNECTIVITY_EVALUATION_H
#define _CONNECTIVITY_EVALUATION_H

#include "CoverageEvaluation.h"

template <class CFG, class WEIGHT>
class ConnectivityEvaluation : public CoverageEvaluation<CFG,WEIGHT> {
 public:
  ConnectivityEvaluation(double t, ConnectMap<CFG,WEIGHT>* CM,
                         LocalPlanners<CFG,WEIGHT>* LP,
                         CollisionDetection* CD,
                         DistanceMetric* DM,
                         vector<CFG>& S) {
    CoverageEvaluation<CFG,WEIGHT>::m_threshold = t; 
    CoverageEvaluation<CFG,WEIGHT>::cm = CM;
    CoverageEvaluation<CFG,WEIGHT>::lp = LP; 
    CoverageEvaluation<CFG,WEIGHT>::cd = CD; 
    CoverageEvaluation<CFG,WEIGHT>::dm = DM; 
    CoverageEvaluation<CFG,WEIGHT>::samples = S;
    CoverageEvaluation<CFG,WEIGHT>::all_data = true;
  }

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    CoverageEvaluation<CFG,WEIGHT>::evaluate(rmap);

    int num_queries = 0;
    for(int i=0; i<CoverageEvaluation<CFG,WEIGHT>::connections.size(); ++i)
      sort(CoverageEvaluation<CFG,WEIGHT>::connections[i].begin(), 
           CoverageEvaluation<CFG,WEIGHT>::connections[i].end());
    for(int i=0; i<CoverageEvaluation<CFG,WEIGHT>::connections.size()-1; ++i)
      for(int j=i+1; j<CoverageEvaluation<CFG,WEIGHT>::connections.size(); ++j) {
        vector<VID> intersection;
        set_intersection(CoverageEvaluation<CFG,WEIGHT>::connections[i].begin(), 
                         CoverageEvaluation<CFG,WEIGHT>::connections[i].end(),
                         CoverageEvaluation<CFG,WEIGHT>::connections[j].begin(), 
                         CoverageEvaluation<CFG,WEIGHT>::connections[j].end(),
                         back_insert_iterator<vector<VID> >(intersection));
        if(!(intersection.empty()))
          num_queries ++;
      }
    double p_queries = ((double)num_queries) / 
                       ((double)(CoverageEvaluation<CFG,WEIGHT>::connections.size()*(CoverageEvaluation<CFG,WEIGHT>::connections.size()-1)/2));
    cout << "% Queries Solved: " << p_queries << endl;
    return (p_queries >= CoverageEvaluation<CFG,WEIGHT>::m_threshold);
  }
};

#endif
