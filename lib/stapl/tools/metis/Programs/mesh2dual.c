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
 * mesh2dual.c
 *
 * This file reads in the element node connectivity array of a mesh and writes
 * out its dual in the format suitable for Metis.
 *
 * Started 9/29/97
 * George
 *
 */

#include <metis.h>



/*************************************************************************
* Let the game begin
**************************************************************************/
main(int argc, char *argv[])
{
  int i, j, ne, nn, etype, numflag=0;
  idxtype *elmnts, *xadj, *adjncy;
  metis_timer IOTmr, DUALTmr;
  char fileout[256], etypestr[4][5] = {"TRI", "TET", "HEX", "QUAD"};

  if (argc != 2) {
    printf("Usage: %s <meshfile>\n",argv[0]);
    exit(0);
  }

  cleartimer(IOTmr);
  cleartimer(DUALTmr);

  starttimer(IOTmr);
  elmnts = ReadMesh(argv[1], &ne, &nn, &etype);
  stoptimer(IOTmr);

  printf("**********************************************************************\n");
  printf("%s", METISTITLE);
  printf("Mesh Information ----------------------------------------------------\n");
  printf("  Name: %s, #Elements: %d, #Nodes: %d, Etype: %s\n\n", argv[1], ne, nn, etypestr[etype-1]);
  printf("Forming Dual Graph... -----------------------------------------------\n");

  xadj = idxmalloc(ne+1, "main: xadj");
  adjncy = idxmalloc(10*ne, "main: adjncy");

  starttimer(DUALTmr);
  METIS_MeshToDual(&ne, &nn, elmnts, &etype, &numflag, xadj, adjncy);
  stoptimer(DUALTmr);

  printf("  Dual Information: #Vertices: %d, #Edges: %d\n", ne, xadj[ne]/2);

  sprintf(fileout, "%s.dgraph", argv[1]);
  starttimer(IOTmr);
  WriteGraph(fileout, ne, xadj, adjncy);
  stoptimer(IOTmr);


  printf("\nTiming Information --------------------------------------------------\n");
  printf("  I/O:          \t\t %7.3f\n", gettimer(IOTmr));
  printf("  Dual Creation:\t\t %7.3f\n", gettimer(DUALTmr));
  printf("**********************************************************************\n");

  GKfree((void**)&elmnts, (void**)&xadj, &adjncy, LTERM);

}


