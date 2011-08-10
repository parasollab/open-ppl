#ifndef VIZMODEBUGOUTPUT_H_
#define VIZMODEBUGOUTPUT_H_

#include <fstream>
#include <iostream>
#include <string>

using namespace std;

extern ofstream* vdo;

void VDInit(string filename);

void VDClose();

template<class CFG>
void VDAddNode(CFG cfg){
  if(vdo!=NULL){
    (*vdo) << "AddNode " << cfg << endl;
  }
};

template<class CFG>
void VDRemoveNode(CFG cfg){
  if(vdo!=NULL){
    (*vdo) << "RemoveNode " << cfg << endl;
  }
};

template<class CFG>
void VDAddEdge(CFG cfg1, CFG cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddEdge " << cfg1 << " " << cfg2 << endl;
  }
};

template<class CFG>
void VDRemoveEdge(CFG cfg1, CFG cfg2){
  if(vdo!=NULL){
    (*vdo) << "RemoveEdge " << cfg1 << " " << cfg2 << endl;
  }
};

template<class CFG>
void VDAddTempCfg(CFG cfg, bool valid){
  if(vdo!=NULL){
    (*vdo) << "AddTempCfg " << cfg << " " << valid << endl;
  }
};

template<class CFG>
void VDAddTempRay(CFG cfg){
  if(vdo!=NULL){
    (*vdo) << "AddTempRay " << cfg << endl;
  }
};

template<class CFG>
void VDAddTempEdge(CFG cfg1, CFG cfg2){
  if(vdo!=NULL){
    (*vdo) << "AddTempEdge " << cfg1 << " " << cfg2 << endl;
  }
};

void VDComment(string s);

void VDClearAll();

void VDClearLastTemp();

void VDClearComments();

template<class CFG>
void VDQuery(CFG cfg1, CFG cfg2){
  if(vdo!=NULL){
    (*vdo) << "Query " << cfg1 << " " << cfg2 << endl;
  }
};

#endif
