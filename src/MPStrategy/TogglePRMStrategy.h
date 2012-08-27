#ifndef TogglePRMStrategy_h
#define TogglePRMStrategy_h

#include "MPProblem/RoadmapGraph.h" //for VID typedef
#include "MetricUtils.h"
#include "MPStrategy/MPStrategyMethod.h"

class TogglePRMStrategy : public MPStrategyMethod 
{
 public:
   TogglePRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem);
   virtual ~TogglePRMStrategy();

   virtual void ParseXML(XMLNodeReader& in_Node);
   virtual void PrintOptions(ostream& out_os);

   virtual void Initialize();
   virtual void Run();
   virtual void Finalize();

 protected:
   //helper functions for operator()
   void Connect(pair<string, VID> vid, vector<VID>& allVID, vector<VID>& allNodesVID,
          vector<VID>& allCollisionNodesVID, deque<pair<string, CfgType> >& queue);

   //data
   map<string, pair<int,int> > m_NodeGenerationLabels;
   vector<string> m_NodeConnectionLabels;
   vector<string> m_ColNodeConnectionLabels;
   //vector<string> m_ComponentConnectionLabels;
   //vector<string> m_ColComponentConnectionLabels;
   vector<string> m_EvaluatorLabels;
   string vcMethod;
   int m_CurrentIteration;
   bool priority;

 private:
   template <typename OutputIterator>
      void GenerateNodes(OutputIterator allOut,
      OutputIterator thisIterationOut, OutputIterator allCollisionOut, OutputIterator
      thisIterationCollisionOut, deque<pair<string, CfgType> >& queue);

 
};

#endif
