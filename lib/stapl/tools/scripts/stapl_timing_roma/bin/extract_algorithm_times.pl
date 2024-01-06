#!/usr/bin/perl
sub int_comp {
    $a <=> $b;
}

$system = pop @ARGV;
@tests = ("p_for_each", "p_find", "p_find_if", "p_adjacent_find",
"p_find_first_of", "p_count", "p_count_if", "p_equal",
"p_search", "p_search_n", "p_find_end", "p_copy",
"p_copy_n", "p_copy_backward", "p_swap_ranges", "p_transform",
"p_replace", "p_replace_if", "p_replace_copy", "p_replace_copy_if",
"p_fill", "p_fill_n", "p_generate", "p_generate_n",
"p_remove", "p_remove_if", "p_remove_copy", "p_remove_copy_if",
"p_unique", "p_unique_copy", "p_reverse", "p_reverse_copy",
"p_rotate", "p_rotate_copy", "p_stable_partition", "p_iota",
"p_accumulate", "p_inner_product", "p_partial_sum", "p_adjacent_difference"
);

#@containers = ("parray", "pvector", "pmatrix");
@containers = ("parray", "pvector");

require "/scratch/roma/stapl_nightly_timing/stapl/scripts/stapl_timing_roma/bin/db_funcs.pl";

print "before db init\n";
init_db();
print "after db init\n";

$workdir = $ENV{'PWD'};

foreach $dir (<@ARGV>) {
  $dir =~ m/(\d\d)\-(\d\d)\-(\d\d\d\d)\.(\d\d\d\d)/;
  $ts = $3.$1.$2.$4."00";
  $jackts = $ts;
  print "$ts ***\n";

  # Enter the directory with the files.
  chdir $dir;
  $ENV{'PWD'} .= "$workdir/$dir";

  # Build the array of processor counts
  my @procs;
  my %file_hash;
  my %datasize_hash;
  `rm -f p_*.o* *.done *.final`;
  @files = `ls *.out`;
  foreach $file (@files) {
    ($proc, $trash, $container, $datasize) = split("_",$file);
    chomp $proc;
    chomp $datasize;
    $datasize =~ s/\.out//;
    if ($proc != $procs[$#procs]) {
      $procs[$#procs + 1] = $proc;
    }
    $file_hash{$proc}{$container} = $file;
    $datasize_hash{$proc} = $datasize;
  }


  #initialize hash table of times
  my %times;
  foreach $testname (@tests) {
    foreach $container (@containers) {
      $times{$testname}{$container}{"stl"} = 0.0;
      for ($i = 0; $i <= $#procs; $i++) {
        $times{$testname}{$container}{$procs[$i]} = 0.0;
      }
    }
  }


  #read times from the files.
  foreach $proc (sort int_comp @procs) {
    foreach $container (@containers) {
      $file = $file_hash{$proc}{$container};
      open(TIMES,"<./$file");
      $testname = '';
      $cnt = -1;
      ($t1, $t2, $t3, $datasize) = split("_",$file);
      chomp $datasize;
      $datasize =~ s/\.out//;
      while ($line = <TIMES>) {
        # new test found
        if ($line =~ /test_/) {
          #average previous test times
          if ($cnt > 0) {
            $times{$testname}{$container}{$proc} /= $cnt;
            if ($proc eq "1p") {
              $times{$testname}{$container}{"stl"} /= $cnt;
            }
          }
          $cnt = 0;
  
          #grab the new test name
          chomp $line;
          $testname = $line;
          $testname =~ s/test_//; 
          $testname =~ s/: //; 
          $testname =~ s/://; 
        } elsif (($testname ne "p_power") && ($testname ne "p_swap") &&
                 ($testname ne "p_iter_swap")) {
          #Split the timing information -- example line below
          #Passed Sequential time: 2.5e-05 Parallel time: 0.00238 Speedup: 0.01049
          ($trash, $sequential, $parallel, $speedup) = split(": ", $line);
          ($seq_time, $trash) = split(" ", $sequential);
          ($par_time, $trash) = split(" ", $parallel);
          if ($proc eq "1p") {
            #insert STL times from 1 processor run
            $times{$testname}{$container}{"stl"} += $seq_time;
          }
          $times{$testname}{$container}{$proc} += $par_time;
          $cnt++;
        }
      }
      #average times of last test
      if ($cnt > 0) {
        $times{$testname}{$container}{$proc} /= $cnt;
      }
      close(TIMES);
    }
  }

  #insert times into the database
  foreach $container (@containers) {
    foreach $test ( sort keys %times ) {
      $stl_test = $test;
      $stl_test =~ s/p_//;
      print "insert_run(\"$stl_test\",\"$system\", \"timmie\", $jackts, \"Nightly Timing\");\n";
      $token = insert_run("$stl_test","$system", "timmie", $jackts, "Nightly Timing");
      insert_statistic($token, "data_type", "-1", "int"); # stat_id 2
      insert_statistic($token, "container_type", "-1", "vector"); # stat_id 83
      $datasize = $datasize_hash{$procs[0]};
      insert_statistic($token, "num_elements", "-1", "$datasize"); # stat_id 4
      $exec_time = $times{$test}{$container}{"stl"};
      insert_statistic($token, "exec_time", "-1", "$exec_time"); # stat_id 5
      insert_statistic($token, "timing_group", "-1", $ts); # stat_id 81
      $jackts = increment_timestamp($jackts);
      foreach $proc (sort int_comp @procs ) {
        print "insert_run(\"$test\",\"$system\", \"timmie\", $jackts, \"Nightly Timing\");\n";
        $token = insert_run("$test","$system", "timmie", $jackts, "Nightly Timing");
        print "insert_statistic($token, \"num_procs\", \"-1\", \"$proc\");\n"; # stat_id 1
        insert_statistic($token, "num_procs", "-1", "$proc"); # stat_id 1
        insert_statistic($token, "data_type", "-1", "int"); # stat_id 2
        $cont = $container;
        $cont =~ s/p/p_/;
        insert_statistic($token, "pcontainer_type", "-1", "$cont"); # stat_id 82
        $datasize = $datasize_hash{$proc};
        insert_statistic($token, "num_elements", "-1", "$datasize"); # stat_id 4
        $exec_time = $times{$test}{$container}{$proc};
        insert_statistic($token, "exec_time", "-1", "$exec_time" ); # stat_id 5
        insert_statistic($token, "timing_group", "-1", $ts); # stat_id 81
        $jackts = increment_timestamp($jackts);
      }
    }
  }

  chdir $workdir;
  $ENV{'PWD'} .= $workdir;
}

close_db();
