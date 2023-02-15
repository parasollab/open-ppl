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
 * oemetis.c
 *
 * This file contains the driving routine for multilevel method
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
  int i, options[10];
  idxtype *perm, *iperm;
  GraphType graph;
  char filename[256];
  int numflag = 0, wgtflag;
  metis_timer TOTALTmr, METISTmr, IOTmr, SMBTmr;


  if (argc != 2) {
    printf("Usage: %s <GraphFile>\n",argv[0]);
    exit(0);
  }
    
  strcpy(filename, argv[1]);

  cleartimer(TOTALTmr);
  cleartimer(METISTmr);
  cleartimer(IOTmr);
  cleartimer(SMBTmr);

  starttimer(TOTALTmr);
  starttimer(IOTmr);
  ReadGraph(&graph, filename, &wgtflag);
  if (graph.nvtxs <= 0) {
    printf("Empty graph. Nothing to do.\n");
    exit(0);
  }
  if (graph.ncon != 1) {
    printf("Ordering can only be applied to graphs with one constraint.\n");
    exit(0);
  }
  stoptimer(IOTmr);

  /* Ordering does not use weights! */
  GKfree((void**)&graph.vwgt, (void**)&graph.adjwgt, LTERM);

  printf("**********************************************************************\n");
  printf("%s", METISTITLE);
  printf("Graph Information ---------------------------------------------------\n");
  printf("  Name: %s, #Vertices: %d, #Edges: %d\n\n", filename, graph.nvtxs, graph.nedges/2);
  printf("Edge-Based Ordering... ----------------------------------------------\n");

  perm = idxmalloc(graph.nvtxs, "main: perm");
  iperm = idxmalloc(graph.nvtxs, "main: iperm");
  options[0] = 0;

  starttimer(METISTmr);
  METIS_EdgeND(&graph.nvtxs, graph.xadj, graph.adjncy, &numflag, options, perm, iperm);
  stoptimer(METISTmr);

  starttimer(IOTmr);
  WritePermutation(filename, iperm, graph.nvtxs); 
  stoptimer(IOTmr);

  starttimer(SMBTmr);
  ComputeFillIn(&graph, iperm);
  stoptimer(SMBTmr);

  stoptimer(TOTALTmr);

  printf("\nTiming Information --------------------------------------------------\n");
  printf("  I/O:                     \t %7.3f\n", gettimer(IOTmr));
  printf("  Ordering:                \t %7.3f   (OEMETIS time)\n", gettimer(METISTmr));
  printf("  Symbolic Factorization:  \t %7.3f\n", gettimer(SMBTmr));
  printf("  Total:                   \t %7.3f\n", gettimer(TOTALTmr));
  printf("**********************************************************************\n");


  GKfree((void**)&graph.xadj, (void**)&graph.adjncy, &perm, &iperm, LTERM);
}  


