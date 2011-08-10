#ifndef VIZMODEBUGOUTPUT_H_
#define VIZMODEBUGOUTPUT_H_

#include <iostream>
#include "Cfg.h"
#include "CfgTypes.h"

using namespace std;

void VDInit(string filename);
void VDClose();
void VDAddNode(CfgType cfg);
void VDRemoveNode(CfgType cfg);
void VDAddEdge(CfgType cfg1, CfgType cfg2);
void VDRemoveEdge(CfgType cfg1, CfgType cfg2);
void VDAddTempCfg(CfgType cfg, bool valid = true);
void VDAddTempRay(CfgType cfg);
void VDAddTempEdge(CfgType cfg1, CfgType cfg2);
void VDComment(string s);
void VDClearAll();
void VDClearLastTemp();
void VDClearComments();
void VDQuery(CfgType cfg1, CfgType cfg2);


#endif
