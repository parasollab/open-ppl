#ifndef LPOUTPUT_H_
#define LPOUTPUT_H_

template<class MPTraits>
struct LPOutput {
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;
  typedef pair<WeightType, WeightType> LPEdge;
  typedef pair< pair<CfgType, CfgType>, LPEdge> LPSavedEdge;

  vector<CfgType> path;          // Path found by local planner.
  vector<CfgType> intermediates;
  LPEdge edge; // Contains weights of edges defined in path.
  vector<LPSavedEdge> savedEdge;  // Failed Edge: savedEdge.second -> position failed.

  void Clear(){
    path.clear();
    intermediates.clear();
    edge.first.SetWeight(0);
    edge.second.SetWeight(0);
    savedEdge.clear();
  }
  
  void SetLPLabel(string _l){
    edge.first.SetLPLabel(_l);
    edge.second.SetLPLabel(_l);
  }

  void AddIntermediatesToWeights(bool _saveIntermediates){
    if(_saveIntermediates){
      edge.first.SetIntermediates(intermediates);
      vector<CfgType> tmp = intermediates;
      reverse(tmp.begin(), tmp.end());
      edge.second.SetIntermediates(tmp);
    }
  }
};

#endif
