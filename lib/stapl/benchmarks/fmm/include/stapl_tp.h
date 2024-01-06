/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TP_H
#define STAPL_TP_H
#include "stapl_entry.h"
#include <fstream>
#include "types.h"
#include "tree_stapl.h"

// STAPL Data Types
//typedef stapl::vector<StaplVectorOfBodies> array_array_body_tp;
//typedef stapl::vector_view<array_array_body_tp> array_array_body_vw_tp;

//STL Data Types
typedef std::vector<Body>                            BodyVector;
typedef stapl::vector_view<BodyVector>               StaplVectorViewOfSTDVector;
typedef std::vector<std::vector<Body>>               VectorOfVectorBodies;
typedef std::vector<std::vector<Cell>>               VectorOfVectorCells;
typedef std::vector<size_t>                          PartitioningVector;
typedef stapl::lightweight_vector<real_t>            RealVector;

#endif
