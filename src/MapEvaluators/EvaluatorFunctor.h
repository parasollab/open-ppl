#ifndef EVALUATORFUNCTOR_H_
#define EVALUATORFUNCTOR_H_

template<class MapEvaluatorPointer>
class EvaluatorFunctor {
  public:
    EvaluatorFunctor() {}
    ~EvaluatorFunctor() {}

    bool operator()(MapEvaluatorPointer _conditionalType) {
      return _conditionalType->operator()();
    }
};

#endif
