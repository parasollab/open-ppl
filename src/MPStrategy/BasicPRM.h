#ifndef BasicPRM_h
#define BasicPRM_h

#include "MPStrategyMethod.h"

class XMLNodeReader;
class MPProblem;
template<class CFG, class WEIGHT> class MPRegion;

struct BPSContainer : public MPSMContainer {
    BPSContainer () : MPSMContainer() {}
    map<string, pair<int,int> > m_nodeGenerationLabels;
    map<string, pair<double,int> > m_probGenerationLabels;
    vector<string> m_nodeConnectionLabels;
    vector<string> m_componentConnectionLabels;
    vector<string> m_evaluatorLabels;
    int m_currentIteration;
};


class BasicPRM : public MPStrategyMethod 
{
 public:
   BasicPRM(BPSContainer cont) : MPStrategyMethod(cont) {
     m_nodeGenerationLabels = cont.m_nodeGenerationLabels;
     m_probGenerationLabels = cont.m_probGenerationLabels;
     m_nodeConnectionLabels = cont.m_nodeConnectionLabels;
     m_componentConnectionLabels = cont.m_componentConnectionLabels;
     m_evaluatorLabels = cont.m_evaluatorLabels;
     m_currentIteration = cont.m_currentIteration;
   };

   BasicPRM(XMLNodeReader& _node, MPProblem* _problem);
   virtual ~BasicPRM();

   virtual void ParseXML(XMLNodeReader& _node);
   virtual void PrintOptions(ostream& _os);

   virtual void Initialize(int _regionID);
   virtual void Run(int _regionID);
   virtual void Finalize(int _regionID);


 protected:
   //helper functions for operator()
   void ConnectNodes(MPRegion<CfgType, WeightType>* _region, vector<VID>& _allNodesVID, vector<VID>& _thisIterationNodesVID);
   void ConnectComponents(MPRegion<CfgType, WeightType>* _region);
   bool EvaluateMap(int _regionID);

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
   enum {NODE_GENERATION, NODE_CONNECTION, COMPONENT_CONNECTION, MAP_EVALUATION} m_startAt;

 private:
   template <typename OutputIterator>
      void GenerateNodes(MPRegion<CfgType, WeightType>* _region, OutputIterator _allOut, OutputIterator _thisIterationOut);

   string PickNextSampler();

};

#endif
