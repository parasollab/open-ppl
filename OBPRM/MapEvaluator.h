#include "Roadmap.h"

template <class CFG, class WEIGTH> 
class MapEvaluationMethod;

template <class CFG, class WEIGHT>
class MapEvaluator {
 public:
  vector<MapEvaluationMethod<CFG,WEIGHT>*> evaluators;

  MapEvaluator() {}
  MapEvaluator(const vector<MapEvaluationMethod<CFG,WEIGHT>*>& e) : 
    evaluators(e) {}
  ~MapEvaluator() {}

  void AddEvaluator(MapEvaluationMethod<CFG,WEIGHT>* e) {
    evaluators.push_back(e);
  }

  bool operator()(const Roadmap<CFG,WEIGHT>* rmap) {
    vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator E;
    for(E = evaluators.begin(); E != evaluators.end(); ++E)
      if(!(*E)->evaluate(rmap))
	return false;
    return true;
  }
};

template <class CFG, class WEIGHT>
class MapEvaluationMethod {
 public:
  virtual bool evaluate(const Roadmap<CFG,WEIGHT>* rmap) = 0;
};

//for testing the framework only... will be deleted later
template <class CFG, class WEIGHT>
class TestEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  int size;
  TestEvaluation(int s = 400) : size(s) {}
  ~TestEvaluation() {}

  virtual bool evaluate(const Roadmap<CFG,WEIGHT>* rmap) {
    return (rmap->m_pRoadmap->GetVertexCount() > size);
  }
};
