#ifndef PPL_ROADMAP_GRAPH_H_
#define PPL_ROADMAP_GRAPH_H_

#include "GenericStateGraph.h"
#include "Cfg.h"

class GeneralRoadmap : GenericStateGraph<Cfg, DefaultWeight<Cfg>> {};

#endif
