#!/bin/sh

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

run_command=$1
TEST_OUTPUT_PREFIX='(STAPL RTS)'

#
# core support
#

eval $run_command ./test_affinity
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_affinity - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_affinity - [passed]"
fi

eval $run_command ./test_alignment
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_alignment - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_alignment - [passed]"
fi

eval $run_command ./test_rmi_handle
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_rmi_handle - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_rmi_handle - [passed]"
fi


#
# point-to-point
#

eval $run_command ./test_async_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_async_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_async_rmi - [passed]"
fi

eval $run_command ./test_opaque_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_opaque_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_opaque_rmi - [passed]"
fi

eval $run_command ./test_try_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_try_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_try_rmi - [passed]"
fi

eval $run_command ./test_sync_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_sync_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_sync_rmi - [passed]"
fi


#
# asynchronous result
#

eval $run_command ./test_promise
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_promise - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_promise - [passed]"
fi

eval $run_command ./test_future
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_future - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_future - [passed]"
fi


#
# synchronization
#

eval $run_command ./test_rmi_barrier
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_rmi_barrier - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_rmi_barrier - [passed]"
fi

eval $run_command ./test_rmi_fence
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_rmi_fence - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_rmi_fence - [passed]"
fi

eval $run_command ./test_rmi_synchronize
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_rmi_synchronize - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_rmi_synchronize - [passed]"
fi


#
# collective
#

eval $run_command ./test_allgather_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_allgather_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_allgather_rmi - [passed]"
fi

eval $run_command ./test_allreduce_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_allreduce_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_allreduce_rmi - [passed]"
fi

eval $run_command ./test_allreduce_object
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_allreduce_object - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_allreduce_object - [passed]"
fi

eval $run_command ./test_broadcast_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_broadcast_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_broadcast_rmi - [passed]"
fi


#
# one-sided
#

eval $run_command ./test_async_rmi_all
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_async_rmi_all - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_async_rmi_all - [passed]"
fi

eval $run_command ./test_async_rmi_range
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_async_rmi_range - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_async_rmi_range - [passed]"
fi

eval $run_command ./test_opaque_rmi_all
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_opaque_rmi_all - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_opaque_rmi_all - [passed]"
fi

eval $run_command ./test_reduce_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_reduce_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_reduce_rmi - [passed]"
fi


#
# communication patterns
#

eval $run_command ./test_async_patterns
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_async_patterns - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_async_patterns - [passed]"
fi

eval $run_command ./test_comm_patterns
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_comm_patterns - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_comm_patterns - [passed]"
fi

eval $run_command ./test_list_ranking
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_list_ranking - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_list_ranking - [passed]"
fi


#
# special objects
#

eval $run_command ./test_args
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_args - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_args - [passed]"
fi

eval $run_command ./test_big_arguments
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_big_arguments - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_big_arguments - [passed]"
fi

eval $run_command ./test_bind
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_bind - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_bind - [passed]"
fi

eval $run_command ./test_p_object
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_p_object - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_p_object - [passed]"
fi


#
# other
#

eval $run_command ./test_construct
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_construct - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_construct - [passed]"
fi

eval $run_command ./test_async_construct
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_async_construct - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_async_construct - [passed]"
fi

eval $run_command ./test_bind_rmi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_bind_rmi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_bind_rmi - [passed]"
fi

# SEE gforge TODO 1371
#eval $run_command ./test_restore
#if test $? != 0
#then
#  echo $TEST_OUTPUT_PREFIX" test_restore - [FAILED]"
#else
#  echo $TEST_OUTPUT_PREFIX" test_restore - [passed]"
#fi

eval $run_command ./test_immutable_ref
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_immutable_ref - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_immutable_ref - [passed]"
fi

eval $run_command ./test_immutable_shared
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_immutable_shared - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_immutable_shared - [passed]"
fi

eval $run_command ./test_orphan_location
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_orphan_location - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_orphan_location - [passed]"
fi

eval $run_command ./test_range
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_range - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_range - [passed]"
fi

eval $run_command ./test_immutable_range
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_immutable_range - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_immutable_range - [passed]"
fi

eval $run_command ./test_zero_copy
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_zero_copy - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_zero_copy - [passed]"
fi

eval $run_command ./test_defer 10000 10
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_defer - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_defer - [passed]"
fi
