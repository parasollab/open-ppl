#ifndef BASICPRM_H_
#define BASICPRM_H_

#include "MPStrategyMethod.h"

class XMLNodeReader;
class MPProblem;

struct BPSContainer : public MPSMContainer {
  BPSContainer () : MPSMContainer() {}
  map<string, pair<int,int> > m_nodeGenerationLabels;
  map<string, pair<double,int> > m_probGenerationLabels;
  vector<string> m_nodeConnectionLabels;
  vector<string> m_componentConnectionLabels;
  vector<string> m_evaluatorLabels;
  int m_currentIteration;
};


class BasicPRM : public MPStrategyMethod{ 
 public:
   BasicPRM(BPSContainer _cont) : MPStrategyMethod(_cont) {
     m_nodeGenerationLabels = _cont.m_nodeGenerationLabels;
     m_probGenerationLabels = _cont.m_probGenerationLabels;
     m_nodeConnectionLabels = _cont.m_nodeConnectionLabels;
     m_componentConnectionLabels = _cont.m_componentConnectionLabels;
     m_evaluatorLabels = _cont.m_evaluatorLabels;
     m_currentIteration = _cont.m_currentIteration;
   };

   BasicPRM(XMLNodeReader& _node, MPProblem* _problem);
   virtual ~BasicPRM();

   virtual void ParseXML(XMLNodeReader& _node);
   virtual void PrintOptions(ostream& _os);

   virtual void Initialize();
   virtual void Run();
   virtual void Finalize();


 protected:
   //helper functions for operator()
   void ConnectNodes(vector<VID>& _allNodesVID, vector<VID>& _thisIterationNodesVID);
   void ConnectComponents();

   //data
   map<string, pair<int, int> > m_nodeGenerationLabels;
   map<string, pair<double, int> > m_probGenerationLabels;
   vector<string> m_nodeConnectionLabels;
   vector<string> m_componentConnectionLabels;
   vector<string> m_evaluatorLabels;
   string m_vcMethod;
   int m_currentIteration;
   bool m_useProbability;
   string m_inputMapFilename;
   enum {NODE_GENERATION, NODE_CONNECTION, 
     COMPONENT_CONNECTION, MAP_EVALUATION} m_startAt;

 private:
   template <typename OutputIterator>
     void GenerateNodes(OutputIterator _allOut, OutputIterator _thisIterationOut);

   string PickNextSampler();

};

#endif
