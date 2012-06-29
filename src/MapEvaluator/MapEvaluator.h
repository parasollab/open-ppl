#ifndef MAPEVALUATOR_H
#define MAPEVALUATOR_H

#include <boost/mpl/list.hpp>
#include "MPUtils.h"
#include "TrueEvaluation.h"
#include "PrintMapEvaluation.h"
#include "QueryEvaluation.h"
#include "ComposeEval.h"
#include "NegateEvaluation.h"
#include "ConditionalEvaluation.h"

namespace pmpl_detail {
  typedef boost::mpl::list<
    TrueEvaluation,
    PrintMapEvaluation, 
    QueryEvaluation<CfgType, WeightType>,
    ComposeEvaluation<CfgType, WeightType>,
    NegateEvaluation<CfgType, WeightType>,
    ConditionalEvaluation<CfgType, WeightType>
    > MapEvaluationMethodList;
}

template <class CFG, class WEIGHT>
class MapEvaluator : private ElementSet<MapEvaluationMethod>, public MPBaseObject {
  public:
    typedef shared_ptr<MapEvaluationMethod> MapEvaluationMethodPtr;
    typedef ElementSet<MapEvaluationMethod>::MethodPointer MapEvaluationPointer;

    template<typename MethodList>
    MapEvaluator() : ElementSet<MapEvaluationMethod>(MethodList()) {}
  
    MapEvaluator() : ElementSet<MapEvaluationMethod>(pmpl_detail::MapEvaluationMethodList()) {}

    template <typename MethodList>
    MapEvaluator(XMLNodeReader& _node, MPProblem* _problem, MethodList const&)
      : ElementSet<MapEvaluationMethod>(MethodList()), MPBaseObject(_problem) {
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
        if(!ElementSet<MapEvaluationMethod>::AddElement(citr->getName(), *citr, _problem))
	  citr->warnUnknownNode();
      }
      PrintOptions(cout);
    }

    MapEvaluator(XMLNodeReader& _node, MPProblem* _problem)
      : ElementSet<MapEvaluationMethod>(pmpl_detail::MapEvaluationMethodList()), MPBaseObject(_problem) {
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) 
	if(!ElementSet<MapEvaluationMethod>::AddElement(citr->getName(), *citr, _problem)) {
	  citr->warnUnknownNode();
   	}
      PrintOptions(cout);
    }

    virtual ~MapEvaluator() {}

    MapEvaluationPointer GetMethod(string _label) {
      return ElementSet<MapEvaluationMethod>::GetElement(_label);
    }

    void AddMethod(string const& _label, MapEvaluationPointer _eval) {
      ElementSet<MapEvaluationMethod>::AddElement(_label, _eval);
    }

    virtual void SetMPProblem(MPProblem* _mp) {
      MPBaseObject::SetMPProblem(_mp);
      ElementSet<MapEvaluationMethod>::SetMPProblem(_mp);
    }

    void PrintOptions(ostream& _os) const {
      _os << "Map Evaluators methods available : " << endl;
      for(map<string, shared_ptr<MapEvaluationMethod> >::const_iterator M = ElementsBegin(); M != ElementsEnd(); ++M)
        _os << "\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;

    }
};

#endif
