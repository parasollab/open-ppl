#ifndef _COVERAGE_EVALUATION_H
#define _COVERAGE_EVALUATION_H

/////////////////////////
// evaluate the coverage of a given roadmap
#include "NodeGeneratorMethod.h"

template <class CFG, class WEIGHT> 
class CoverageEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  double m_threshold;
  vector<CFG> samples;
  vector<vector<VID> > connections;
  ConnectMap<CFG,WEIGHT>* cm;
  LocalPlanners<CFG,WEIGHT>* lp;
  CollisionDetection* cd;
  DistanceMetric* dm;
  bool all_data;

  CoverageEvaluation() {}

  CoverageEvaluation(double t, ConnectMap<CFG,WEIGHT>* CM,
                     LocalPlanners<CFG,WEIGHT>* LP,
                     CollisionDetection* CD,
                     DistanceMetric* DM,
                     vector<CFG>& S) :
    m_threshold(t), cm(CM), lp(LP), cd(CD), dm(DM), samples(S) {
    all_data = false;
  }

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    RoadmapGraph<CFG,WEIGHT>* pMap = rmap->m_pRoadmap;
    VID backupVID = pMap->getVertIDs();

    connections = vector<vector<VID> >(samples.size());

    vector<pair<int,VID> > ccs;
    vector<pair<int,VID> >::iterator CC;
    GetCCStats(*pMap, ccs);

    vector<VID> samplelist, cc;
    Stat_Class Stats;
    for(int i=0; i<samples.size(); ++i) {
      VID sampleVID = pMap->AddVertex(samples[i]);
      samplelist.clear();
      samplelist.push_back(sampleVID);

      for(CC = ccs.begin(); CC != ccs.end(); ++CC) {
        cc.clear();
        GetCC(*pMap, CC->second, cc);

        int degree_before = pMap->GetVertexOutDegree(sampleVID);

        cm->ConnectNodes(rmap, Stats, cd,  dm,  lp, false, false,
                         samplelist, cc);
        
        if(pMap->GetVertexOutDegree(sampleVID) > degree_before) {
          connections[i].push_back(CC->second);
          if(!all_data) 
            break;
        }
      }
      
      pMap->DeleteVertex(sampleVID);
    }

    pMap->setVertIDs(backupVID); 

    int num_connections = 0;
    for(int i=0; i<connections.size(); ++i)
      if(!(connections[i].empty()))
        num_connections++;
    cout << "Connection %: " << ((double)num_connections)/((double)connections.size()) << endl;
    return (((double)num_connections)/((double)connections.size()) >= m_threshold);
  }
};

#endif
