# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/paragraph,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

test: all

test-debug: CXXFLAGS+=-g -DSTAPL_DEBUG
test-debug: all

all: compile run

compile : write_repeat_pos \
          mem_leak_test \
          p_for_all_pos \
          nested_migration_pos \
          non_blocking_pos \
          viewless_pos \
          termination_detection_pos \
          overlapped_execution_pos \
          scheduler_pos \
          task_migration_pos \
          set_num_succs_pos \
          filtered_dataflow_pos \
          filtered_dataflow_pos2 \
          aggregated_dataflow_pos \
          nested_one_sided_pos \
          strict_priority_scheduling_pos \
          pg_p2p_data_flow_pos \
          pg_p2p_data_flow_nD_pos \
          nested_filtered_dataflow_pos \
          single_round_td_time_pos

run:
	$(MAKE) -l 15.0 \
          viewless \
          write_repeat \
          p_for_all \
          nested_migration \
          non_blocking \
          termination_detection \
          overlapped_execution \
          scheduler \
          task_migration \
          set_num_succs \
          filtered_dataflow \
          filtered_dataflow2 \
          aggregated_dataflow \
          nested_one_sided \
          strict_priority_scheduling \
          pg_p2p_data_flow \
          pg_p2p_data_flow_nD \
	  nested_filtered_dataflow_pos \
          single_round_td_time

clean :
	rm -f \
        add_subrange_pos \
        write_repeat_pos \
        viewless_pos \
        mem_leak_test \
        p_for_all_pos \
        nested_migration_pos \
        non_blocking_pos \
        termination_detection_pos \
        overlapped_execution_pos \
        scheduler_pos \
        task_migration_pos \
        set_num_succs_pos \
        filtered_dataflow_pos \
        filtered_dataflow_pos2 \
        aggregated_dataflow_pos \
        nested_one_sided_pos \
        strict_priority_scheduling_pos \
        pg_p2p_data_flow_pos \
        pg_p2p_data_flow_nD_pos \
        nested_filtered_dataflow_pos \
        single_round_td_time_pos \
        *.o *~

p_for_all : p_for_all_pos
	$(call staplrun,1) ./p_for_all_pos
	$(call staplrun,4) ./p_for_all_pos

viewless : viewless_pos
	$(call staplrun,1) ./viewless_pos
	$(call staplrun,4) ./viewless_pos

add_subrange : add_subrange_pos
	$(call staplrun,1) ./add_subrange_pos
	$(call staplrun,4) ./add_subrange_pos

write_repeat : write_repeat_pos
	$(call staplrun,1) ./write_repeat_pos
	$(call staplrun,4) ./write_repeat_pos

nested_migration : nested_migration_pos
	$(call staplrun,4) ./nested_migration_pos

non_blocking : non_blocking_pos
	$(call staplrun,4) ./non_blocking_pos

termination_detection : termination_detection_pos
	$(call staplrun,1) ./termination_detection_pos
	$(call staplrun,4) ./termination_detection_pos

overlapped_execution : overlapped_execution_pos
	$(call staplrun,1) ./overlapped_execution_pos
	$(call staplrun,4) ./overlapped_execution_pos

scheduler : scheduler_pos
	$(call staplrun,1) ./scheduler_pos < serial.input
	$(call staplrun,4) ./scheduler_pos < serial.input

task_migration : task_migration_pos
	$(call staplrun,1) ./task_migration_pos
	$(call staplrun,4) ./task_migration_pos

set_num_succs : set_num_succs_pos
	$(call staplrun,1) ./set_num_succs_pos
	$(call staplrun,4) ./set_num_succs_pos

filtered_dataflow : filtered_dataflow_pos
	$(call staplrun,1) ./filtered_dataflow_pos

filtered_dataflow2 : filtered_dataflow_pos2
	$(call staplrun,1) ./filtered_dataflow_pos2

aggregated_dataflow : aggregated_dataflow_pos
	$(call staplrun,1) ./aggregated_dataflow_pos
	$(call staplrun,4) ./aggregated_dataflow_pos

nested_one_sided : nested_one_sided_pos
	$(call staplrun,1) ./nested_one_sided_pos
	$(call staplrun,4) ./nested_one_sided_pos

strict_priority_scheduling : strict_priority_scheduling_pos
	$(call staplrun,1) ./strict_priority_scheduling_pos 8
	$(call staplrun,4) ./strict_priority_scheduling_pos 8

pg_p2p_data_flow : pg_p2p_data_flow_pos
	$(call staplrun,4) ./pg_p2p_data_flow_pos 2 1600 16 100

pg_p2p_data_flow_nD : pg_p2p_data_flow_nD_pos
	$(call staplrun,4) ./pg_p2p_data_flow_nD_pos 128 4 16 1,1 2,2

nested_filtered_dataflow : nested_filtered_dataflow_pos
	$(call staplrun,1) ./nested_filtered_dataflow_pos 1000
	$(call staplrun,4) ./nested_filtered_dataflow_pos 1000

single_round_td_time: single_round_td_time_pos
	$(call staplrun,1) ./single_round_td_time_pos
	$(call staplrun,4) ./single_round_td_time_pos

