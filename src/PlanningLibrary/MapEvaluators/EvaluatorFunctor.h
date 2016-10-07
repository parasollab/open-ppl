#ifndef EVALUATOR_FUNCTOR_H_
#define EVALUATOR_FUNCTOR_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Function object for MapEvaluator pointers for use in method
///        composition.
/// @tparam MapEvaluatorPointer MapEvaluatorMethod for functor
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
