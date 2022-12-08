#include "Features.h"
#include "CfgFeature.h"
#include "VisibilityFeature.h"
#include "ClearanceFeature.h"

Features::Features():MPBaseObject(){
   //push_back all different Features into Feature
   all.push_back(new CfgFeature());
   all.push_back(new VisibilityFeature());
   all.push_back(new ClearanceFeature());
}

Features::Features(XMLNode& in_Node, MPProblem* mp): MPBaseObject(in_Node, mp){
  ParseXML(in_Node);
}

void Features::ParseXML(XMLNode& in_Node){
   //read from the xml and push onto selected
   XMLNode::childiterator citr;
   for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
      if(citr->getName()=="CfgFeature"){
         CfgFeature* cf = new CfgFeature(*citr, GetMPProblem());
         selected.push_back(cf);
      }
      else if(citr->getName()=="VisibilityFeature"){
         VisibilityFeature* vf = new VisibilityFeature(*citr, GetMPProblem());
         selected.push_back(vf);
      }
      else if(citr->getName()=="ClearanceFeature"){
         ClearanceFeature* clf = new ClearanceFeature(*citr, GetMPProblem());
         selected.push_back(clf);
      }
      else if (citr->getName()=="CSpaceClearanceFeature"){
         CSpaceClearanceFeature* csc = new CSpaceClearanceFeature(*citr, GetMPProblem());
         selected.push_back(csc);
      }
      citr->warnUnrequestedAttributes();
   }
}

MPFeature* Features::GetFeature(string s){
   typedef vector<MPFeature*>::iterator FIT;
   for(FIT fit = selected.begin(); fit!=selected.end(); fit++){
      if((*fit)->GetLabel()==s){
         return *fit;
      }
   }
   return NULL;
}

int Features::GetFeatureIndex(string s){
   int i = 0;
   typedef vector<MPFeature*>::iterator FIT;
   for(FIT fit = selected.begin(); fit!=selected.end(); fit++){
      if((*fit)->GetLabel()==s){
         return i;
      }
      i++;
   }
   return -1;
}

vector<vector<double> > Features::Collect(vector<VID>& vids){
   vector<vector<double> > data;
   typedef vector<MPFeature*>::iterator FIT;
   for(FIT fit = selected.begin(); fit!=selected.end(); fit++){
      data.push_back((*fit)->Collect(vids));
   }
   return data;
}

vector<vector<double> > Features::Collect(vector<string> features, vector<VID>& vids){
   vector<vector<double> > data;
   typedef vector<string>::iterator SIT;
   for(SIT sit = features.begin(); sit!=features.end(); sit++){
      data.push_back(GetFeature(*sit)->Collect(vids));
   }
   return data;
}
