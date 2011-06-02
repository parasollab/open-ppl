#ifndef CFGFEATURE_H_
#define CFGFEATURE_H_

#include "MPFeature.h"

class CfgFeature : public MPFeature {
 public:
   CfgFeature();
   CfgFeature(int index);
   CfgFeature(XMLNodeReader& in_Node, MPProblem* in_pProblem);
   virtual ~CfgFeature(){}

   virtual void ParseXML(XMLNodeReader& in_Node);

   virtual vector<double> Collect(vector<VID>& vids);

 private:
   int m_index;
};

#endif
