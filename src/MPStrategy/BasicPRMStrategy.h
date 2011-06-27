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
  vector<pair<string, int> > m_NodeGenerationLabels;
  vector<pair<string, double> > m_ProbGenerationLabels;
   vector<string> m_NodeConnectionLabels;
   vector<string> m_ComponentConnectionLabels;
   vector<string> m_EvaluatorLabels;
   string m_LPMethod;
   string dm_label;
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
   m_LPMethod = cont.m_LPMethod;
   dm_label = cont.dm_label;
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
   vector<pair<string, int> > m_NodeGenerationLabels;
   vector<pair<string, double> > m_ProbGenerationLabels;
   vector<string> m_NodeConnectionLabels;
   vector<string> m_ComponentConnectionLabels;
   vector<string> m_EvaluatorLabels;
   string m_LPMethod;
   string dm_label;
   string vcMethod;
   int m_CurrentIteration;
   bool useProbability;

 private:
   template <typename OutputIterator>
      void GenerateNodes(MPRegion<CfgType, WeightType>* region, OutputIterator allOut, OutputIterator thisIterationOut);
   string PickNextSampler();

   Clock_Class MapGenClock;

};

#include "MPStrategy.h"
#include "MapEvaluator.h"
//implentations of template functions
template <typename OutputIterator>
void BasicPRMStrategy::GenerateNodes(MPRegion<CfgType, WeightType>* region, 
    OutputIterator allOut, OutputIterator thisIterationOut){
  Clock_Class NodeGenClock;
  CDInfo cdInfo;
  Stat_Class * pStatClass = region->GetStatClass();
  stringstream clockName; 
  clockName << "Iteration " << m_CurrentIteration << ", Node Generation"; 
  NodeGenClock.StartClock(clockName.str().c_str());
  string Callee("BasicPRMStrategy::GenerateNodes");

  typedef vector<pair<string, int> >::iterator GIT;
  vector<CfgType> outNodes;
  if(!useProbability){
    for(GIT git = m_NodeGenerationLabels.begin(); git != m_NodeGenerationLabels.end(); ++git){
      Sampler<CfgType>::SamplerPointer pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(git->first);
      pNodeGenerator->print(cout);
      vector<CfgType> inNodes(git->second);

      //generate nodes for this node generator method
      Clock_Class NodeGenSubClock;
      stringstream generatorClockName; 
      generatorClockName << "Iteration " << m_CurrentIteration << ", " << git->first;
      NodeGenSubClock.StartClock(generatorClockName.str().c_str());

      cout << "\n\t";

      do{
        pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),*pStatClass,inNodes.begin(),inNodes.end(),git->second*2+400,
            back_inserter(outNodes));
      }while(outNodes.size()<=0&&m_CurrentIteration==1);

      cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
      NodeGenSubClock.StopPrintClock();

    }
  }
  else{
    string NextNodeGen = PickNextSampler();
    Sampler<CfgType>::SamplerPointer pNodeGenerator; 
    pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(NextNodeGen);
    pNodeGenerator->print(cout);
    vector<CfgType> inNodes(1);

    //generate nodes for this node generator method
    Clock_Class NodeGenSubClock;
    stringstream generatorClockName; 
    generatorClockName << "Iteration " << m_CurrentIteration << ", " << NextNodeGen;
    NodeGenSubClock.StartClock(generatorClockName.str().c_str());

    cout << "\n\t";
    pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),*pStatClass,inNodes.begin(),inNodes.end(),100, back_inserter(outNodes));

    cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;

    cout << "\n\t";
    NodeGenSubClock.StopPrintClock();
  }
  //add valid nodes to roadmap
  typedef vector<CfgType>::iterator CIT;
  for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
    if(!(*cit).IsLabel("VALID")){
      !(GetMPProblem()->GetValidityChecker()->IsValid(GetMPProblem()->GetValidityChecker()->GetVCMethod(vcMethod), *cit, GetMPProblem()->GetEnvironment(), *(region->GetStatClass()), cdInfo, true, &Callee));
    }
    if((*cit).IsLabel("VALID") && ((*cit).GetLabel("VALID"))) {
      if(!region->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
        VID vid = region->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
        //store value and increment iterator
        *thisIterationOut++ = vid;
        *allOut++ = vid;
      }
    }
  }
  NodeGenClock.StopPrintClock();
}

#endif
