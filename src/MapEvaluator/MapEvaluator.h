#ifndef _MAP_EVALUATOR_H
#define _MAP_EVALUATOR_H

#include "boost/shared_ptr.hpp"
using boost::shared_ptr;

#include "MapEvaluationMethod.h"
#include "MPRegionComparerMethod.h"
#include "NumNodesEvaluation.h"
#include "NumEdgesEvaluation.h"
#include "QueryEvaluation.h"
#include "PrintMapEvaluation.h"
#include "CoverageEvaluation.h"
#include "ConnectivityEvaluation.h"

template <class CFG, class WEIGHT>
class MapEvaluator : public MPBaseObject 
{
 public:
  typedef shared_ptr<MapEvaluationMethod> conditional_type;
  typedef typename vector<conditional_type>::iterator conditional_iterator;
  
  typedef shared_ptr<MPRegionComparerMethod<CFG, WEIGHT> > comparer_type;
  typedef typename vector<comparer_type>::iterator comparer_iterator;

  MapEvaluator() {}
  
  MapEvaluator(const vector<conditional_type>& e, vector<comparer_type> _m_comparer_evaluators) 
  : m_conditional_evaluators(e), m_comparer_evaluators(_m_comparer_evaluators)
  {}
  
  MapEvaluator(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  {
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
    {
      if (citr->getName() == "MPRegionComparers")
        ParseXMLComparers(*citr, in_pProblem);
      else if (citr->getName() == "MPRegionConditionalEvaluators") 
        ParseXMLConditionalEvaluators(*citr, in_pProblem);
      else
        citr->warnUnknownNode();
    }
  }
  
  virtual ~MapEvaluator() {}
  
  void ParseXMLComparers(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  {
    m_comparer_evaluators.clear();
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
    {
      if (citr->getName() == "ConnectableComponentComparer") 
        m_comparer_evaluators.push_back(comparer_type(new ConnectableComponentComparer<CFG, WEIGHT>(*citr, in_pProblem)));
      else if (citr->getName() == "RandomConnectComparer") 
        m_comparer_evaluators.push_back(comparer_type(new RandomConnectComparer<CFG, WEIGHT>(*citr, in_pProblem)));
      else if (citr->getName() == "RegionCoverageComparer")
        m_comparer_evaluators.push_back(comparer_type(new RegionCoverageComparer<CFG, WEIGHT>(*citr, in_pProblem)));
      else if (citr->getName() == "RegionSimilarity")
        m_comparer_evaluators.push_back(comparer_type(new RegionSimilarity<CFG, WEIGHT>(*citr, in_pProblem)));
      else
        citr->warnUnknownNode();
    }
  }
  
  virtual void ParseXMLConditionalEvaluators(XMLNodeReader& in_Node, MPProblem *in_pProblem) 
  {
    m_conditional_evaluators.clear();
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
    {
      if (citr->getName() == "NumNodesEvaluation")
        m_conditional_evaluators.push_back(conditional_type(new NumNodesEvaluation(*citr, in_pProblem)));
      else if (citr->getName() == "NumEdgesEvaluation")
        m_conditional_evaluators.push_back(conditional_type(new NumEdgesEvaluation(*citr, in_pProblem)));
      else if (citr->getName() == "QueryEvaluation")
        m_conditional_evaluators.push_back(conditional_type(new QueryEvaluation<CFG, WEIGHT>(*citr, in_pProblem))); 
      else if (citr->getName() == "PrintMapEvaluation")
        m_conditional_evaluators.push_back(conditional_type(new PrintMapEvaluation(*citr, in_pProblem))); 
      else if (citr->getName() == "TrueEvaluation")
         m_conditional_evaluators.push_back(conditional_type(new TrueEvaluation(*citr, in_pProblem)));
      else if (citr->getName() == "CoverageEvaluation")
        m_conditional_evaluators.push_back(conditional_type(new CoverageEvaluation<CFG, WEIGHT>(*citr, in_pProblem)));
      else if (citr->getName() == "ConnectivityEvaluation")
        m_conditional_evaluators.push_back(conditional_type(new ConnectivityEvaluation<CFG, WEIGHT>(*citr, in_pProblem)));
      else
        citr->warnUnknownNode();
    }
  }
  
  void AddEvaluator(conditional_type& e)
  {
    m_conditional_evaluators.push_back(e);
  }
  
  bool operator()() 
  {
    return (*this)(GetMPProblem->CreateMPRegion());
  }
  
  bool operator()(int in_RegionID)
  {
    for (conditional_iterator E = m_conditional_evaluators.begin(); E != m_conditional_evaluators.end(); ++E)
      if (!((**E)(in_RegionID)))
        return false;
    return true;
  }
  
  bool operator()(MPRegion<CFG,WEIGHT>* region_a, MPRegion<CFG,WEIGHT>* region_b) 
  {
    for (comparer_iterator E = m_comparer_evaluators.begin(); E != m_comparer_evaluators.end(); ++E)
      (*E)->compare(region_a, region_b);
    //@todo operator() should return meaningful comparison, not just a boolean
    return true;
  }

  conditional_type GetConditionalMethod(const string& in_label)
  {
    for (conditional_iterator I = m_conditional_evaluators.begin(); I != m_conditional_evaluators.end(); ++I)
      if ((*I)->GetLabel() == in_label)
        return *I;

    cerr << "MapEvaluator:: cannot find ConditionalMethod = " << in_label << endl;
    exit(-1);
  }

  comparer_type GetComparerMethod(const string& in_label) 
  {
    for (comparer_iterator I = m_comparer_evaluators.begin(); I != m_comparer_evaluators.end(); ++I) 
      if ((*I)->GetLabel() == in_label)
        return *I;
      
    cerr << "MapEvaluator:: cannot find ComparerMethod = " << in_label << endl;
    exit(-1);
  }
  
  void PrintOptions(ostream& out_os) 
  {
    out_os << "  Map Evaluation: " << m_conditional_evaluators.size() << " Conditional Methods" << endl;
    for (conditional_iterator I = m_conditional_evaluators.begin(); I != m_conditional_evaluators.end(); ++I)
    {
      out_os << "    ";
      (*I)->PrintOptions(out_os);
    }

    out_os << "  Map Evaluation: " << m_comparer_evaluators.size() << " Comparer Methods" << endl;
    for (comparer_iterator I = m_comparer_evaluators.begin(); I != m_comparer_evaluators.end(); ++I)
    {
      out_os << "    ";
      (*I)->PrintOptions(out_os);
    }
  }

  //@todo remove when Input class deleted
  void PrintDefaults(ostream& out_os) {}

  vector<conditional_type> m_conditional_evaluators;
  vector<comparer_type> m_comparer_evaluators;
};

#endif
