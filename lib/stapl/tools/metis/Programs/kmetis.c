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
 * kmetis.c
 *
 * This file contains the driving routine for kmetis
 *
 * Started 8/28/94
 * George
 *
 */

#include <metis.h>



/*************************************************************************
* Let the game begin
**************************************************************************/
main(int argc, char *argv[])
{
  int i, nparts, options[10];
  idxtype *part;
  float rubvec[MAXNCON], lbvec[MAXNCON];
  GraphType graph;
  char filename[256];
  int numflag = 0, wgtflag = 0, edgecut;
  metis_timer TOTALTmr, METISTmr, IOTmr;

  if (argc != 3) {
    printf("Usage: %s <GraphFile> <Nparts>\n",argv[0]);
    exit(0);
  }
    
  strcpy(filename, argv[1]);
  nparts = atoi(argv[2]);

  if (nparts < 2) {
    printf("The number of partitions should be greater than 1!\n");
    exit(0);
  }

  cleartimer(TOTALTmr);
  cleartimer(METISTmr);
  cleartimer(IOTmr);

  starttimer(TOTALTmr);
  starttimer(IOTmr);
  ReadGraph(&graph, filename, &wgtflag);
  if (graph.nvtxs <= 0) {
    printf("Empty graph. Nothing to do.\n");
    exit(0);
  }
  stoptimer(IOTmr);

  printf("**********************************************************************\n");
  printf("%s", METISTITLE);
  printf("Graph Information ---------------------------------------------------\n");
  printf("  Name: %s, #Vertices: %d, #Edges: %d, #Parts: %d\n", filename, graph.nvtxs, graph.nedges/2, nparts);
  if (graph.ncon > 1)
    printf("  Balancing Constraints: %d\n", graph.ncon);
  printf("\nK-way Partitioning... -----------------------------------------------\n");

  part = idxmalloc(graph.nvtxs, "main: part");
  options[0] = 0;

  starttimer(METISTmr);
  if (graph.ncon == 1) {
    METIS_PartGraphKway(&graph.nvtxs, graph.xadj, graph.adjncy, graph.vwgt, graph.adjwgt, 
          &wgtflag, &numflag, &nparts, options, &edgecut, part);
  }
  else {
    for (i=0; i<graph.ncon; i++)
      rubvec[i] = HORIZONTAL_IMBALANCE;

    METIS_mCPartGraphKway(&graph.nvtxs, &graph.ncon, graph.xadj, graph.adjncy, graph.vwgt, 
          graph.adjwgt, &wgtflag, &numflag, &nparts, rubvec, options, &edgecut, part);
  }
  stoptimer(METISTmr);

  ComputePartitionBalance(&graph, nparts, part, lbvec);

  printf("  %d-way Edge-Cut: %7d, Balance: ", nparts, edgecut);
  for (i=0; i<graph.ncon; i++)
    printf("%5.2f ", lbvec[i]);
  printf("\n");

  starttimer(IOTmr);
  WritePartition(filename, part, graph.nvtxs, nparts); 
  stoptimer(IOTmr);
  stoptimer(TOTALTmr);

  printf("\nTiming Information --------------------------------------------------\n");
  printf("  I/O:          \t\t %7.3f\n", gettimer(IOTmr));
  printf("  Partitioning: \t\t %7.3f   (KMETIS time)\n", gettimer(METISTmr));
  printf("  Total:        \t\t %7.3f\n", gettimer(TOTALTmr));
  printf("**********************************************************************\n");


  GKfree((void**)&graph.xadj, (void**)&graph.adjncy, &graph.vwgt, &graph.adjwgt, &part, LTERM);
}  


