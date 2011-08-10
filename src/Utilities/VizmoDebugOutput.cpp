#include "VizmoDebugOutput.h"
#include <fstream>

ofstream* vdo = NULL;

void VDInit(string filename){
  if(filename!=""){
    vdo = new ofstream(filename.c_str());
  }
}

void VDClose(){
  if(vdo!=NULL){
    vdo->close();
    delete vdo;
  }
}

void VDAddNode(CfgType cfg){
  if(vdo!=NULL){
    (*vdo) << "AddNode " << cfg << endl;
  }
}

void VDRemoveNode(CfgType cfg){
  if(vdo!=NULL){
    (*vdo) << "RemoveNode " << cfg << endl;
  }
}

void VDAddEdge(CfgType cfg1, CfgType cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddEdge " << cfg1 << " " << cfg2 << endl;
  }
}

void VDRemoveEdge(CfgType cfg1, CfgType cfg2){
  if(vdo!=NULL){
    (*vdo) << "RemoveEdge " << cfg1 << " " << cfg2 << endl;
  }
}

void VDAddTempCfg(CfgType cfg, bool valid){
  if(vdo!=NULL){
    (*vdo) << "AddTempCfg " << cfg << " " << valid << endl;
  }
}

void VDAddTempRay(CfgType cfg){
  if(vdo!=NULL){
    (*vdo) << "AddTempRay " << cfg << endl;
  }
}

void VDAddTempEdge(CfgType cfg1, CfgType cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddTempEdge " << cfg1 << " " << cfg2 << endl;
  }
}

void VDComment(string s){
  if(vdo!=NULL){
    (*vdo) << "Comment " << s << endl;
  }
}

void VDClearAll(){
  if(vdo!=NULL){
    (*vdo) << "ClearAll " << endl;
  }
}

void VDClearLastTemp(){
  if(vdo!=NULL){
    (*vdo) << "ClearLastTemp " << endl;
  }
}

void VDClearComments(){
  if(vdo!=NULL){
    (*vdo) << "ClearComments " << endl;
  }
}

void VDQuery(CfgType cfg1, CfgType cfg2){
  if(vdo!=NULL){
    (*vdo) << "Query " << cfg1 << " " << cfg2 << endl;
  }
}
