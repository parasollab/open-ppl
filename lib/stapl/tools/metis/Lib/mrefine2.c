/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * mrefine2.c
 *
 * This file contains the driving routines for multilevel refinement
 *
 * Started 7/24/97
 * George
 *
 */

#include <metis.h>


/*************************************************************************
* This function is the entry point of refinement
**************************************************************************/
void MocRefine2Way2(CtrlType *ctrl, GraphType *orggraph, GraphType *graph, float *tpwgts, 
       float *ubvec)
{

  IFSET(ctrl->dbglvl, DBG_TIME, starttimer(ctrl->UncoarsenTmr));

  /* Compute the parameters of the coarsest graph */
  MocCompute2WayPartitionParams(ctrl, graph);

  for (;;) {
    ASSERT(CheckBnd(graph));

    IFSET(ctrl->dbglvl, DBG_TIME, starttimer(ctrl->RefTmr));
    switch (ctrl->RType) {
      case RTYPE_FM:
        MocBalance2Way2(ctrl, graph, tpwgts, ubvec);
        MocFM_2WayEdgeRefine2(ctrl, graph, tpwgts, ubvec, 8); 
        break;
      default:
        errexit("Unknown refinement type: %d\n", ctrl->RType);
    }
    IFSET(ctrl->dbglvl, DBG_TIME, stoptimer(ctrl->RefTmr));

    if (graph == orggraph)
      break;

    graph = graph->finer;
    IFSET(ctrl->dbglvl, DBG_TIME, starttimer(ctrl->ProjectTmr));
    MocProject2WayPartition(ctrl, graph);
    IFSET(ctrl->dbglvl, DBG_TIME, stoptimer(ctrl->ProjectTmr));
  }

  IFSET(ctrl->dbglvl, DBG_TIME, stoptimer(ctrl->UncoarsenTmr));
}


