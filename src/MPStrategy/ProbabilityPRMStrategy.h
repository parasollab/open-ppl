#ifndef ProbabilityPRMStrategy_h
#define ProbabilityPRMStrategy_h

#include "MPProblem/MPProblem.h"
#include "MPStrategy/BasicPRMStrategy.h"
#include "MPRegion.h"
#include "Sampler.h"
#include "MPStrategy/MPStrategy.h"

class ProbabilityPRMStrategy : public BasicPRMStrategy
{
 public:
   ProbabilityPRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem);
   virtual ~ProbabilityPRMStrategy();

   virtual void ParseXML(XMLNodeReader& in_Node);
   virtual void PrintOptions(ostream& out_os);

   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID){}

 private:
   //helper functions for operator()
   template <typename OutputIterator>
      void GenerateNodes(MPRegion<CfgType, WeightType>* region, OutputIterator allOut, OutputIterator thisIterationOut);
   string PickNextSampler();

   //data
   vector<pair<string, double> > m_NodeGenerationLabels;
};

//implentations of template functions
template <typename OutputIterator>
void ProbabilityPRMStrategy::GenerateNodes(MPRegion<CfgType, WeightType>* region, 
                                             OutputIterator allOut, OutputIterator thisIterationOut){
   Clock_Class NodeGenClock;
   CDInfo cdInfo;
   Stat_Class * pStatClass = region->GetStatClass();
   stringstream clockName; 
   clockName << "Iteration " << m_CurrentIteration << ", Node Generation"; 
   NodeGenClock.StartClock(clockName.str().c_str());
   string NextNodeGen = PickNextSampler();
   Sampler<CfgType>::SamplerPointer pNodeGenerator; 
   pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(NextNodeGen);
   vector<CfgType> outNodes;
   vector<CfgType> inNodes(1);
        
   //generate nodes for this node generator method
   Clock_Class NodeGenSubClock;
   stringstream generatorClockName; 
   generatorClockName << "Iteration " << m_CurrentIteration << ", " << NextNodeGen;
   NodeGenSubClock.StartClock(generatorClockName.str().c_str());
    
   cout << "\n\t";
   pNodeGenerator->GetSampler()->Sample(pNodeGenerator,GetMPProblem()->GetEnvironment(),*pStatClass,inNodes.begin(),inNodes.end(),100, back_inserter(outNodes));
   
   cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;

   cout << "\n\t";
   NodeGenSubClock.StopPrintClock();
        
   //add valid nodes to roadmap
   typedef vector<CfgType>::iterator CIT;
   for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
      if(!(*cit).IsLabel("VALID")){
         cit->isCollision(GetMPProblem()->GetEnvironment(),*(region->GetStatClass()),GetMPProblem()->GetCollisionDetection(), cdInfo);
      }
      if((*cit).IsLabel("VALID") && ((*cit).GetLabel("VALID"))) {
         if(!region->GetRoadmap()->m_pRoadmap->IsVertex(*cit))
         {
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
