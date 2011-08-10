#include "VizmoDebugOutput.h"

ofstream* vdo = NULL;

void VDInit(string filename){
  if(filename!=""){
    vdo = new ofstream(filename.c_str());
  }
};

void VDClose(){
  if(vdo!=NULL){
    vdo->close();
    delete vdo;
  }
}

void VDComment(string s){
  if(vdo!=NULL){
    (*vdo) << "Comment " << s << endl;
  }
};

void VDClearAll(){
  if(vdo!=NULL){
    (*vdo) << "ClearAll " << endl;
  }
};

void VDClearLastTemp(){
  if(vdo!=NULL){
    (*vdo) << "ClearLastTemp " << endl;
  }
};

void VDClearComments(){
  if(vdo!=NULL){
    (*vdo) << "ClearComments " << endl;
  }
};

;
