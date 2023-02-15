/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#define DATA_CNT 10000000
int init_count = DATA_CNT;

int prime_nums[DATA_CNT] = {
#include "../10M_prime"
};

int rand_nums[DATA_CNT] = {
#include "../10M_rand"
};
