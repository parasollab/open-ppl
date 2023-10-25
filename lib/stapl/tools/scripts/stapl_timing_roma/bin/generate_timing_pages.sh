#!/bin/sh
export PATH=/bin:/usr/bin:/usr/local/bin
export d=`date +%m-%d-%Y`
if [ -e /scratch/roma/stapl_nightly_timing/hydra_dir/$d.0025.done ]
#if [ -e /scratch/roma/stapl_nightly_timing/hydra_dir/03-07-2010.0025.done ]
then
  #the sleep is needed so the new times can be inserted into the db.
  sleep 300
  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/timings.php machine hydra set nightly type palgorithm container p_array > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_parray.php
  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/timings.php machine hydra set nightly type palgorithm container p_vector > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_pvector.php
#  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/timings.php machine hydra set nightly type palgorithm container p_matrix > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_pmatrix.php

  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_array num_ts 10 class nonmutating > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_array_nonmutating.php
  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_array num_ts 10 class mutating > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_array_mutating.php
  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_array num_ts 10 class numeric > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_array_numeric.php

  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_vector num_ts 10 class nonmutating > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_vector_nonmutating.php
  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_vector num_ts 10 class mutating > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_vector_mutating.php
  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_vector num_ts 10 class numeric > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_vector_numeric.php

#  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_matrix num_ts 10 class nonmutating > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_matrix_nonmutating.php
#  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_matrix num_ts 10 class mutating > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_matrix_mutating.php
#  /usr/bin/php -f /research/www/groups/rwergergroup/intranet/stapl_perf/draw_algo_graphs.php mid 32 machine hydra set nightly type palgorithm container p_matrix num_ts 10 class numeric > /research/www/groups/rwergergroup/intranet/stapl_perf/timing_hydra_nightly_palgorithm_p_matrix_numeric.php

  /usr/local/bin/update-www -r /research/www/groups/rwergergroup/intranet/stapl_perf
else
  exit 0
fi
