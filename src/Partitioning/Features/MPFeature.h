#ifndef FEATURE_H_
#define FEATURE_H_

#include "Roadmap.h"
#include "CfgTypes.h"
#include "Weight.h"
typedef RoadmapGraph<CfgType,WeightType>::vertex_descriptor VID;
class MPFeature {
 public:
   MPFeature(){m_pProblem=NULL;}
   MPFeature(MPProblem* mp){m_pProblem=mp;}
   ~MPFeature(){}
   
   virtual void ParseXML(XMLNodeReader& in_Node)=0;

   virtual vector<double> Collect(vector<VID>& vids)=0;
   
   string GetLabel(){return m_label;}
   void SetLabel(string s){m_label=s;}
   
   MPProblem* GetMPProblem(){return m_pProblem;}
 
 private:
   string m_label;
   MPProblem* m_pProblem;
};

#endif
