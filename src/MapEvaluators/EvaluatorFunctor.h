#ifndef EVALUATORFUNCTOR_H_
#define EVALUATORFUNCTOR_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Function object for MapEvaluator pointers for use in method composition.
////////////////////////////////////////////////////////////////////////////////
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
