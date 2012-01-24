#ifndef _CC_DISTANCE_EVALUATION_H
#define _CC_DISTANCE_EVALUATION_H

////////////////////
// evaluate component length

template <class CFG, class WEIGHT>
class CCDistanceEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  CCDistanceEvaluation(float t, 
                       DistanceMetric* dm)
    : m_dm(dm), m_threshold(t) {}

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    /*
      what we want here is to detect if the distances are changing
      so, we will look at all cc pairs (filtering out singletons)
      and record intercc dist
      we will return true if the distances stabilize
      (i.e., all pairs are within x% of last time)
     */

    vector<double> new_distances;

    RoadmapGraph<CFG,WEIGHT>* p_map = rmap->m_pRoadmap;
    Environment* p_env = rmap->GetEnvironment();

    //get ccs
    vector< pair<size_t,VID> > ccs; 
    stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
    get_cc_stats(*(rmap->m_pRoadmap), ccs);

    //filter out singletons
    vector<pair<int,VID> > filtered_ccs;
    vector<pair<int,VID> >::iterator cci, ccj;
    for(cci = ccs.begin(); cci != ccs.end(); ++cci)
      if(cci->first > 1)
        filtered_ccs.push_back(*cci);

    //compute new inter cc distances
    for(cci = filtered_ccs.begin(); cci+1 < filtered_ccs.end(); ++cci) {
      vector<VID> cci_vids;
      cmap.reset();
      get_cc(*(p_map), cmap, cci->second, cci_vids);

      for(ccj = cci+1; ccj != filtered_ccs.end(); ++ccj) {      
        vector<VID> ccj_vids;
	cmap.reset();
        get_cc(*(p_map),cmap, ccj->second, ccj_vids);
        
        vector<pair<VID,VID> > pairs = 
          m_dm->FindKClosestPairs(rmap, cci_vids, ccj_vids, 1); 
        new_distances.push_back(m_dm->Distance(p_env,
                                               (*(p_map->find_vertex(pairs[0].first))).property(), 
                                               (*(p_map->find_vertex(pairs[0].second))).property()));
      }
    }    
    sort(new_distances.begin(), new_distances.end(), greater<double>());

    //debugging
    cout << "previous distances:\n";
    copy(prev_distances.begin(), prev_distances.end(), 
         ostream_iterator<double>(cout, " "));
    cout << endl;
    cout << "new distances:\n";
    copy(new_distances.begin(), new_distances.end(), 
         ostream_iterator<double>(cout, " "));
    cout << endl;

    //if # cc pairs increased since last time, return false
    if(new_distances.size() > prev_distances.size()) {
      prev_distances = new_distances;
      return false;
    }

    //if dist difference > threshold, return false
    vector<double>::iterator P, N;
    N = new_distances.begin();
    for(P = prev_distances.begin();
        P != prev_distances.end() && N != new_distances.end();
        ++P, ++N) {
      if(abs((int)(((*N - *P)/ *P))) > m_threshold) {
        prev_distances = new_distances;
        return false;
      }
    }

    //return true
    prev_distances = new_distances;
    return true;    
  }

 protected:
  float m_threshold;
  vector<double> prev_distances;
  DistanceMetric* m_dm;
};

#endif
