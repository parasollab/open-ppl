#ifndef _CC_DIAMETER_EVALUATION_H
#define _CC_DIAMETER_EVALUATION_H

/////////////////////////
// component diameter
template <class CFG, class WEIGHT>
class CCDiameterEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  CCDiameterEvaluation (float t, 
                        DistanceMetric* dm)
    : m_dm(dm), m_threshold(t) {}
  
  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    /*
      want to track CC diameters and see if 
      1) still getting larger
      2) merging components

      could get all cc diamerters and sort
      then compare previous and current diameters
      return true if for each cc, it is w/in x% of previous value
      false ow
      probably want to filter out small ccs, say = 1 node
    */
    
    vector<double> new_diameters;
    
    RoadmapGraph<CFG,WEIGHT>* p_map = rmap->m_pRoadmap;
    Environment* p_env = rmap->GetEnvironment();
    
    //get ccs
    vector<pair<int,VID> > CCs;
    GetCCStats(*p_map, CCs);
    
    //filter out singletons
    vector<pair<int,VID> > filtered_ccs;
    vector<pair<int,VID> >::iterator CC;
    for(CC = CCs.begin(); CC != CCs.end(); ++CC)
      if(CC->first > 1)
        filtered_ccs.push_back(*CC);
    
    //compute new cc diameters
    vector<VID> cc_vids;
    vector<VID>::iterator V;
    vector<CFG> cc_data;
    for(CC = filtered_ccs.begin(); CC != filtered_ccs.end(); ++CC) {
      cc_vids.clear();
      cc_data.clear();
      GetCC(*p_map, CC->second, cc_vids);
      for(V = cc_vids.begin(); V != cc_vids.end(); ++V)
        cc_data.push_back(p_map->GetData(*V));
      new_diameters.push_back(CCdiameter(cc_data, p_env));
    }
    sort(new_diameters.begin(), new_diameters.end(), greater<double>());
    
    //debugging
    cout << "previous diameters:\n";
    copy(prev_diameters.begin(), prev_diameters.end(), 
         ostream_iterator<double>(cout, " "));
    cout << endl;
    cout << "new diameters:\n";
    copy(new_diameters.begin(), new_diameters.end(), 
         ostream_iterator<double>(cout, " "));
    cout << endl;
    
    //if # cc increased, return false
    if(new_diameters.size() > prev_diameters.size()) {
      prev_diameters = new_diameters;
      return false;
    }
    
    //if cc diameter difference greater than threshold, return false
    vector<double>::iterator P, N;
    N = new_diameters.begin();
    for(P = prev_diameters.begin();
        P != prev_diameters.end() && N != new_diameters.end();
        ++P, ++N) {
      if(abs((int)(((*N - *P)/ *P))) > m_threshold) {
        prev_diameters = new_diameters;
        return false;
      }
    }
    
    //return true
    prev_diameters = new_diameters;
    return true;
  }
  
 protected:
  double CCdiameter(vector<CFG>& cfgs,Environment* p_env) {
    //compute com
    CFG com;
    for(typename vector<CFG>::iterator i=cfgs.begin();i!=cfgs.end();i++)
      com.add(com,*i);
    com.divide(com,cfgs.size());
    
    //compute diameter (radius?)
    double max_d=-1e20;
    for(typename vector<CFG>::iterator i=cfgs.begin();i!=cfgs.end();i++){
      double d=m_dm->Distance(p_env,com,*i);
      if(d>max_d) max_d=d;
      return true;
    }
  }

 protected:
  double CCdiameter(vector<CFG>& cfgs,Environment* p_env) {
    //compute com
    CFG com;
    for(typename vector<CFG>::iterator i=cfgs.begin();i!=cfgs.end();i++)
      com.add(com,*i);
    com.divide(com,cfgs.size());
    
    //compute diameter (radius?)
    double max_d=-1e20;
    for(typename vector<CFG>::iterator i=cfgs.begin();i!=cfgs.end();i++){
      double d=m_dm->Distance(p_env,com,*i);
      if(d>max_d) max_d=d;
    }
    
    //done
    return max_d;
  }
  
 protected:
  float m_threshold; // success rate
  vector<double> prev_diameters;
  
  //planner stuff
  DistanceMetric* m_dm;
};

#endif
