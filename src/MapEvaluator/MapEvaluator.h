#ifndef _MAP_EVALUATOR_H
#define _MAP_EVALUATOR_H

#include "Roadmap.h"

#include "util.h"
#include "MPRegionComparerMethod.h"

template <class CFG, class WEIGHT> 
class MapEvaluationMethod;

template <class CFG, class WEIGHT>
class ConnectableComponentComparer;

template <class CFG, class WEIGHT> 
class TestEvaluation;

template <class CFG, class WEIGHT>
class MapEvaluator : public MPBaseObject {
 public:
  MapEvaluator() {
    //add all evaluators
    TestEvaluation<CFG,WEIGHT>* test = new TestEvaluation<CFG,WEIGHT>();
    all.push_back(test);
  }
  MapEvaluator(const vector<MapEvaluationMethod<CFG,WEIGHT>*>& e) : 
    m_conditional_evaluators(e), MapEvaluator() {
  }
  MapEvaluator(TiXmlNode* in_pNode, MPProblem* in_pProblem) {
    LOG_DEBUG_MSG("MapEvaluator::MapEvaluator()");
    if (!in_pNode) {
      LOG_ERROR_MSG("MapEvaluator::MapEvaluator() error xml input");
      exit(-1);
    }
    if (string(in_pNode->Value()) != "MPEvaluator_methods") {
      LOG_ERROR_MSG("MaPEvaluator::MaPEvaluator() error xml input");
    }

    for (TiXmlNode* pChild=in_pNode->FirstChild(); pChild != NULL; pChild = pChild->NextSibling()) {
      if (string(pChild->Value()) == "MPRegionComparers") {
	ParseXMLComparers(pChild, in_pProblem);
      } else if (string(pChild->Value()) == "MPRegionConditionalEvaluators") {
	ParseXMLConditionalEvaluators(pChild, in_pProblem);
      } else {
	LOG_WARNING_MSG("MapEvaluator:: I don't know: " << endl << *pChild);
      }
    }
  }
  ~MapEvaluator() {
    typename vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator I;
    for(I=m_conditional_evaluators.begin(); I != m_conditional_evaluators.end(); ++I)
      delete *I;
    m_conditional_evaluators.clear();
    for(I=all.begin(); I != all.end(); ++I)
      delete *I;
    all.clear();
  }

  void ParseXMLComparers(TiXmlNode* in_pNode, MPProblem* in_pProblem) {
    LOG_DEBUG_MSG("MapEvaluator::ParseXMLComparers()");
    if (!in_pNode) {
      LOG_ERROR_MSG("MapEvaluator::ParseXMLComparers() error xml input");
      exit(-1);
    }
    if (string(in_pNode->Value()) != "MPRegionComparers") {
      LOG_ERROR_MSG("MapEvaluator::ParseXMLComparers() error xml input"); 
      exit(-1);
    }

    m_comparer_evaluators.clear();
    for (TiXmlNode* pChild = in_pNode->FirstChild(); pChild != NULL; pChild = pChild->NextSibling()) {
      if (string(pChild->Value()) == "ConnectableComponentComparer") {
	ConnectableComponentComparer<CFG,WEIGHT>* connectable_component_comparer = new ConnectableComponentComparer<CFG,WEIGHT>(pChild, in_pProblem);
	m_comparer_evaluators.push_back(connectable_component_comparer);
      }
      else if (string(pChild->Value()) == "RandomConnectComparer") {
	RandomConnectComparer<CFG,WEIGHT>* random_query_comparer = new RandomConnectComparer<CFG,WEIGHT>(pChild, in_pProblem);
	m_comparer_evaluators.push_back(random_query_comparer);
      } 
      else if (string(pChild->Value()) == "RegionCoverageComparer") {
	RegionCoverageComparer<CFG,WEIGHT>* coverage_comparer = 
                  new RegionCoverageComparer<CFG,WEIGHT>(pChild, in_pProblem);
	m_comparer_evaluators.push_back(coverage_comparer);
      } else if (string(pChild->Value()) == "RegionSimilarity") {
	RegionSimilarity<CFG,WEIGHT>* similar = 
                  new RegionSimilarity<CFG,WEIGHT>(pChild, in_pProblem);
	m_comparer_evaluators.push_back(similar);
      } else {
	LOG_DEBUG_MSG("I don't understand");
      }
    }
  }

  void ParseXMLConditionalEvaluators(TiXmlNode* in_pNode, MPProblem *in_pProblem) {
    LOG_DEBUG_MSG("MapEvaluator::ParseXMLConditionalEvaluators()");
    if (!in_pNode) {
      LOG_ERROR_MSG("MapEvaluator::ParseXMLConditionalEvaluators() error xml input");
      exit(-1);
    }
    if (string(in_pNode->Value()) != "MPRegionConditionalEvaluators") {
      LOG_ERROR_MSG("MapEvaluator::ParseXMLConditionalEvaluators() error xml input"); 
      exit(-1);
    }

    m_conditional_evaluators.clear();
    for (TiXmlNode* pChild = in_pNode->FirstChild(); pChild != NULL; pChild = pChild->NextSibling()) {

      if (string(pChild->Value()) == "") { // remove when methods below are implemented
/*       if (string(pChild->Value()) == "TestConditionalEvaluator") { */
/* 	<CFG,WEIGHT>*  = new <CFG,WEIGHT>(pChild, in_pProblem); */
/* 	m_conditional_evaluators.push_back(); */
/*       } else  */
/*       if (string(pChild->Value()) == "MaxFlowConditionalEvaluator") { */
/* 	<CFG,WEIGHT>*  = new <CFG,WEIGHT>(pChild, in_pProblem); */
/* 	m_conditional_evaluators.push_back(); */
/*       } else if (string(pChild->Value()) == "CoverageConditionalEvaluator") { */
/* 	<CFG,WEIGHT>*  = new <CFG,WEIGHT>(pChild, in_pProblem); */
/* 	m_conditional_evaluators.push_back(); */
/*       } else if (string(pChild->Value()) == "QueryConditionalEvaluator") { */
/* 	<CFG,WEIGHT>*  = new <CFG,WEIGHT>(pChild, in_pProblem); */
/* 	m_conditional_evaluators.push_back(); */
/*       } else if (string(pChild->Value()) == "CCDistanceConditionalEvaluator") { */
/* 	<CFG,WEIGHT>*  = new <CFG,WEIGHT>(pChild, in_pProblem); */
/* 	m_conditional_evaluators.push_back(); */
/*       } else if (string(pChild->Value()) == "CCDiameterConditionalEvaluator") { */
/* 	<CFG,WEIGHT>*  = new <CFG,WEIGHT>(pChild, in_pProblem); */
/* 	m_conditional_evaluators.push_back(); */
      } else {
	LOG_DEBUG_MSG("I don't understand");
      }
    }
  }

  int ReadCommandLine(n_str_param* MEstrings[MAX_GN], int numMEs) {
    typename vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator I;
    for(I=m_conditional_evaluators.begin(); I != m_conditional_evaluators.end(); ++I)
      delete *I;
    m_conditional_evaluators.clear();

    for(int i=0; i<numMEs; ++i) {
      std::istringstream myistream(MEstrings[i]->GetValue());
      int argc = 0;
      char* argv[50];
      char cmdFields[50][100];
      while(myistream >> cmdFields[argc]) {
        argv[argc] = (char*)(&cmdFields[argc]);
        ++argc;
      }

      bool found = FALSE;
      try {
        int cmd_begin = 0;
        int cmd_argc = 0;
	char* cmd_argv[50];
	do {
	  for(I=all.begin(); I != all.end(); ++I) {
	    if(!strcmp(argv[cmd_begin], (*I)->GetName())) {
	      cmd_argc = 0;
	      bool is_method_name = false;
	      do {
		cmd_argv[cmd_argc] = &(*(argv[cmd_begin+cmd_argc]));
		cmd_argc++;
		typename vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator N;
		is_method_name = false;
		for(N=all.begin(); N != all.end() && cmd_begin+cmd_argc < argc; ++N)
		  if(!strcmp(argv[cmd_begin+cmd_argc],(*N)->GetName())) {
		    is_method_name = true;
		    break;
		  }
	      } while (!is_method_name && cmd_begin+cmd_argc < argc);
	      (*I)->ParseCommandLine(cmd_argc, cmd_argv);
	      m_conditional_evaluators.push_back((*I)->CreateCopy());
	      (*I)->SetDefault();
	      found = TRUE;
	      break;
	    }
          }
          if(!found)
	    break;
	  cmd_begin = cmd_begin + cmd_argc;
	} while (cmd_begin < argc);
	if(!found)
	  throw BadUsage();
      } catch (BadUsage) {
	cerr << "Command line error" << endl;
	PrintUsage(cerr);
	exit(-1);
      }
    }

    return m_conditional_evaluators.size();
  }

  void PrintUsage(ostream& os) {
    typename vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator I;
    for(I=all.begin(); I != all.end(); ++I)
    (*I)->PrintUsage(os);
  }

  void PrintValues(ostream& os) {
    typename vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator I;
    for(I=m_conditional_evaluators.begin(); I != m_conditional_evaluators.end(); ++I)
    (*I)->PrintUsage(os);
  }

  void PrintDefaults(ostream& os) {
  }

  void AddEvaluator(MapEvaluationMethod<CFG,WEIGHT>* e) {
    m_conditional_evaluators.push_back(e);
  }

  bool operator()(Roadmap<CFG,WEIGHT>* rmap) {
    typename vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator E;
    for(E = m_conditional_evaluators.begin(); E != m_conditional_evaluators.end(); ++E)
      if(!(*E)->evaluate(rmap))
        return false;
    return true;
  }

  bool operator()(MPRegion<CFG,WEIGHT>* region_a, MPRegion<CFG,WEIGHT>* region_b) {
    typename vector<MPRegionComparerMethod<CFG,WEIGHT>* >::iterator E;
    for (E = m_comparer_evaluators.begin(); E!= m_comparer_evaluators.end(); ++E)
      compare(region_a, region_b);
    // @todo operator() should return meaningful comparison, not just a boolean
    return true;
  }

  MPRegionComparerMethod< CFG, WEIGHT > * GetComparerMethod(string& in_label) {
    typedef typename vector< MPRegionComparerMethod< CFG, WEIGHT > * >::iterator Itrtr;
    for (Itrtr itrtr = m_comparer_evaluators.begin(); itrtr < m_comparer_evaluators.end(); itrtr++) {
      if ((*itrtr)->GetLabel() == in_label) {
	return *itrtr;
      }
    }
    LOG_ERROR_MSG( "MapEvaluator:: cannot find ComparerMethod = " << in_label);
    exit(-1);
  }

  void PrintOptions(ostream& out_os) {
    out_os << "  Map Evaluation: " << m_comparer_evaluators.size() << " Comparer Methods" << endl;
    typedef typename vector< MPRegionComparerMethod< CFG, WEIGHT > * >::iterator Itrtr;
    for (Itrtr itrtr = m_comparer_evaluators.begin(); itrtr < m_comparer_evaluators.end(); itrtr++) { 
      (*itrtr)->PrintOptions(out_os);
    }
  }

  vector<MapEvaluationMethod<CFG,WEIGHT>*> m_conditional_evaluators;
  vector<MPRegionComparerMethod<CFG,WEIGHT>* > m_comparer_evaluators;

  vector<MapEvaluationMethod<CFG,WEIGHT>*> all;
};

template <class CFG, class WEIGHT>
class MapEvaluationMethod: public MPBaseObject {
 public:
  MapEvaluationMethod() {}
  MapEvaluationMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPBaseObject(in_pNode, in_pProblem) { }
  virtual void ParseXML(TiXmlNode* in_pNode) = 0;
  virtual void operator() ()=0;
  virtual void operator() (int in_RegionID) = 0;
  // @todo evaluate may need to be replaced by the () operator
  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) = 0;

  virtual char* GetName() const = 0;
  virtual void SetDefault() = 0;
  virtual void ParseCommandLine(int argc, char** argv) = 0;
  virtual MapEvaluationMethod<CFG,WEIGHT>* CreateCopy() const = 0;
  virtual void PrintUsage(ostream& os) const = 0;
};


////////////////////////////////////////
//TEST
//for testing the framework only... will be deleted later
template <class CFG, class WEIGHT>
class TestEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  num_param<int> size;
  TestEvaluation(int s = 400) : size("size", 400, 1, 1000000) {
    size.PutDesc("INTEGER","size, default 400");
    size.PutValue(s);
  }
  ~TestEvaluation() {}

  virtual char* GetName() const {
    return "test";
  }
  virtual void SetDefault() { 
    size.PutValue(400); 
  }
  virtual void ParseCommandLine(int argc, char** argv) {
    for(int i=1; i<argc; ++i) 
      if(!size.AckCmdLine(&i,argc,argv)) {
        cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
        for(int j=0; j<argc; ++j)
 	  cerr << argv[j] << " ";
        cerr << "\"\n\n";
        PrintUsage(cerr);
        cerr << endl;
        exit(-1);
      }
  }
  virtual MapEvaluationMethod<CFG,WEIGHT>* CreateCopy() const {
    TestEvaluation<CFG,WEIGHT>* _copy = new TestEvaluation<CFG,WEIGHT>(size.GetValue());
    return _copy;
  }
  virtual void PrintUsage(ostream&os) const {
    os.setf(ios::left,ios::adjustfield);
    os << endl << GetName() << " ";
    os << "\n\t"; size.PrintUsage(os);
    os.setf(ios::right,ios::adjustfield);
  }

  virtual void ParseXML(TiXmlNode* in_pNode) {}
  virtual void operator() () {};
  virtual void operator() (int in_RegionID) {};

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    return (rmap->m_pRoadmap->GetVertexCount() > size.GetValue());
  }
};


#include "MaxFlowEvaluation.h"
#include "CoverageEvaluation.h"
#include "ConnectivityEvaluation.h"
#include "QueryEvaluation.h"
#include "CCDistanceEvaluation.h"
#include "CCDiameterEvaluation.h"
#include "CCExpansionEvaluation.h"

#endif
