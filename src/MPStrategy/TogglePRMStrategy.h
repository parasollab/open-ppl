#ifndef TogglePRMStrategy_h
#define TogglePRMStrategy_h

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

extern bool done;

class TogglePRMStrategy : public MPStrategyMethod 
{
 public:
   TogglePRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool isInherited=false);
   virtual ~TogglePRMStrategy();

   virtual void ParseXML(XMLNodeReader& in_Node);
   virtual void PrintOptions(ostream& out_os);

   virtual void Initialize(int in_RegionID);
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID);


 protected:
   //helper functions for operator()
   void Connect(MPRegion<CfgType, WeightType>* region, pair<string, VID> vid, vector<VID>& allVID, vector<VID>& allNodesVID,
   vector<VID>& allCollisionNodesVID, deque<pair<string, VID> >& queue);
   bool EvaluateMap(int in_RegionID);

   //data
   vector<pair<string, int> > m_NodeGenerationLabels;
   vector<string> m_NodeConnectionLabels;
   vector<string> m_ColNodeConnectionLabels;
   vector<string> m_ComponentConnectionLabels;
   vector<string> m_ColComponentConnectionLabels;
   vector<string> m_EvaluatorLabels;
   string m_LPMethod;
   string dm_label;
   int m_CurrentIteration;

 private:
   template <typename OutputIterator>
      void GenerateNodes(MPRegion<CfgType, WeightType>* region, OutputIterator allOut,
      OutputIterator thisIterationOut, OutputIterator allCollisionOut, OutputIterator
      thisIterationCollisionOut, deque<pair<string, VID> >& queue);

   Clock_Class MapGenClock;
};

#include "MPStrategy.h"
#include "MapEvaluator.h"
//implentations of template functions
template <typename OutputIterator>
void TogglePRMStrategy::GenerateNodes(MPRegion<CfgType, WeightType>* region, 
                                  OutputIterator allOut, OutputIterator thisIterationOut,
                                  OutputIterator allCollisionOut, OutputIterator thisIterationCollisionOut,
                                  deque<pair<string, VID> >& queue){
   Clock_Class NodeGenClock;
   CDInfo cdInfo;
   Stat_Class * pStatClass = region->GetStatClass();
   stringstream clockName; 
   clockName << "Iteration " << m_CurrentIteration << ", Node Generation"; 
   NodeGenClock.StartClock(clockName.str().c_str());

   typedef vector<pair<string, int> >::iterator GIT;
   for(GIT git = m_NodeGenerationLabels.begin(); git != m_NodeGenerationLabels.end(); ++git){
      Sampler<CfgType>::SamplerPointer pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(git->first);
      vector<CfgType> outNodes, outCollisionNodes;
      vector<CfgType> inNodes(git->second);
        
      //generate nodes for this node generator method
      Clock_Class NodeGenSubClock;
      stringstream generatorClockName; 
      generatorClockName << "Iteration " << m_CurrentIteration << ", " << git->first;
      NodeGenSubClock.StartClock(generatorClockName.str().c_str());
    
      cout << "\n\t";

      do{
         pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),*pStatClass,inNodes.begin(),inNodes.end(),git->second*2+20,
         back_inserter(outNodes), back_inserter(outCollisionNodes));
      }while(outNodes.size()<=0&&m_CurrentIteration==1);

      cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
      NodeGenSubClock.StopPrintClock();
        
      //add valid nodes to roadmap
      typedef vector<CfgType>::iterator CIT;
      for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
         //if(region->GetRoadmap()->m_pRoadmap->get_num_vertices()+
            //region->GetBlockRoadmap()->m_pRoadmap->get_num_vertices()>=1000){done=true;return;}
         //out nodes mean valid then add them to the real roadmap
         if((*cit).IsLabel("VALID") && ((*cit).GetLabel("VALID"))) {
            if(!region->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
               //if(region->GetRoadmap()->m_pRoadmap->get_num_vertices() >= 1000) return;
              VID vid = region->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
              //store value and increment iterator
              *thisIterationOut++ = vid;
              *allOut++ = vid;
              queue.push_front(make_pair("valid", vid));
              //queue.push_back(make_pair("valid", vid));
            }
         }
         //else invalid add to block map
         else{
            if(!region->GetBlockRoadmap()->m_pRoadmap->IsVertex(*cit)) {
              VID vid = region->GetBlockRoadmap()->m_pRoadmap->AddVertex(*cit);
              //store value and increment iterator
              *thisIterationOut++ = vid;
              *allOut++ = vid;
              queue.push_back(make_pair("invalid", vid));
            }
         }
      }
      for(CIT cit=outCollisionNodes.begin(); cit!=outCollisionNodes.end(); ++cit){
         //outCollisionNodes mean INVALID then add to block map
         if((*cit).IsLabel("VALID") && !((*cit).GetLabel("VALID"))) {
            if(!region->GetBlockRoadmap()->m_pRoadmap->IsVertex(*cit)) {
              VID vid = region->GetBlockRoadmap()->m_pRoadmap->AddVertex(*cit);
              //store value and increment iterator
              *thisIterationCollisionOut++ = vid;
              *allCollisionOut++ = vid;
              queue.push_back(make_pair("invalid", vid));
            }
         }
         //else valid add to real map
         else{
            if(!region->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
              VID vid = region->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
              //store value and increment iterator
              *thisIterationCollisionOut++ = vid;
              *allCollisionOut++ = vid;
              queue.push_front(make_pair("valid", vid));
              //queue.push_back(make_pair("valid", vid));
            }
         }  
      }
   }
   NodeGenClock.StopPrintClock();
}

#endif
