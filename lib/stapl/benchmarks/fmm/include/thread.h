/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_THREAD_H
#define STAPL_BENCHMARKS_FMM_THREAD_H

#if TBB
#if DAG_RECORDER == 2
#define TO_TBB 1
#include <tpswitch/tpswitch.h>
#else
#include <tbb/task_group.h>
#include <tbb/task_scheduler_init.h>
using namespace tbb;
#endif

#elif MTHREAD
#define TO_MTHREAD_NATIVE 1
#include <tpswitch/tpswitch.h>

#elif QTHREAD
#define TO_QTHREAD 1
#include <tpswitch/tpswitch.h>

#endif

#if TO_TBB || TO_MTHREAD_NATIVE || TO_QTHREAD
#define num_threads(E)

#elif TBB
#define mk_task_group                 task_group tg;
#define wait_tasks                    tg.wait()
#define create_taskc(E)               tg.run(E)
#define create_taskc_if (x, E)         if (x) { create_taskc(E); } else { E(); }
#define num_threads(E)                task_scheduler_init init(E);

#elif OPENMP
#define PRAGMA_OMP(x)                 _Pragma( #x )
#define mk_task_group
#define wait_tasks                    PRAGMA_OMP(omp taskwait)
#define create_taskc(E)               PRAGMA_OMP(omp task) E()
#define create_taskc_if (x, E)         if (x) { create_taskc(E); } else { E(); }
#define num_threads(E)                omp_set_num_threads(E);

#else
#define mk_task_group
#define wait_tasks
#define create_taskc(E)               E()
#define create_taskc_if(x, E)         E()
#define num_threads(E)

#endif

#endif // STAPL_BENCHMARKS_FMM_THREAD_H
