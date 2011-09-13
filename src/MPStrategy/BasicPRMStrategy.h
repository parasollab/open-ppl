#ifndef BasicPRMStrategy_h
#define BasicPRMStrategy_h

#include "Roadmap.h"
#include "MPProblem/RoadmapGraph.h" //for VID typedef
#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ValidityChecker.hpp"
#include "ConnectMap.h"
#include "LocalPlanners.h"
#include "MPStrategy/MPStrategyMethod.h"
#include "MPProblem/MPProblem.h"
#include "MPCharacterizer.h"
#include "MPRegion.h"
#include "Sampler.h"

class BPSContainer : public MPSMContainer {
  public:
    BPSContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
    map<string, pair<int,int> > m_NodeGenerationLabels;
    map<string, pair<double,int> > m_ProbGenerationLabels;
    vector<string> m_NodeConnectionLabels;
    vector<string> m_ComponentConnectionLabels;
    vector<string> m_EvaluatorLabels;
    int m_CurrentIteration;
    Clock_Class MapGenClock;
    MPSMContainer parent;

};


class BasicPRMStrategy : public MPStrategyMethod 
{
 public:
   BasicPRMStrategy(BPSContainer cont) : MPStrategyMethod(cont.parent) {
     m_NodeGenerationLabels = cont.m_NodeGenerationLabels;
     m_ProbGenerationLabels = cont.m_ProbGenerationLabels;
     m_NodeConnectionLabels = cont.m_NodeConnectionLabels;
     m_ComponentConnectionLabels = cont.m_ComponentConnectionLabels;
     m_EvaluatorLabels = cont.m_EvaluatorLabels;
     m_CurrentIteration = cont.m_CurrentIteration;
     MapGenClock = cont.MapGenClock;
   };

   BasicPRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem);
   virtual ~BasicPRMStrategy();

   virtual void ParseXML(XMLNodeReader& in_Node);
   virtual void PrintOptions(ostream& out_os);

   virtual void Initialize(int in_RegionID);
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID);


 protected:
   //helper functions for operator()
   void ConnectNodes(MPRegion<CfgType, WeightType>* region, vector<VID>& allNodesVID, vector<VID>& thisIterationNodesVID);
   void ConnectComponents(MPRegion<CfgType, WeightType>* region);
   bool EvaluateMap(int in_RegionID);

   //data
   map<string, pair<int, int> > m_NodeGenerationLabels;
   map<string, pair<double, int> > m_ProbGenerationLabels;
   vector<string> m_NodeConnectionLabels;
   vector<string> m_ComponentConnectionLabels;
   vector<string> m_EvaluatorLabels;
   string vcMethod;
   int m_CurrentIteration;
   bool useProbability;
   Clock_Class MapGenClock;
   string inputMapFilename;
   enum {NODE_GENERATION, NODE_CONNECTION, COMPONENT_CONNECTION, MAP_EVALUATION} startAt;

 private:
   template <typename OutputIterator>
      void GenerateNodes(MPRegion<CfgType, WeightType>* region, OutputIterator allOut, OutputIterator thisIterationOut);

   string PickNextSampler();

};

#endif
