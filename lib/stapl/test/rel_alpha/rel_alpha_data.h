/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

// the size of these arrays depends on commmand line parameters
// -data tiny/small/medium/big/huge.  they have at least 1000 elements

extern int prime_nums[];
extern int rand_nums[];
extern size_t data_cnt;

// these arrays always have the same size, which is specified in their name

extern int prime1kto10k[];
extern int prime1000[];
extern int rand1000_01[], rand1000_02[], rand1000_03[], rand1000_04[];
extern int rand1000_05[], rand1000_06[], rand1000_07[], rand1000_08[];

extern int fibo20[];

extern int graf_vert_cnt;
extern int graf_edge_cnt;
extern int **graf_edge_data;
